#include "shutter.h"

#include <string.h>

#include "board.h"
#include "sense.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "shutter";

/* ---- timing / control constants ---- */
#define TICK_MS            20
#define PWM_FREQ_HZ        16000
#define PWM_RES            LEDC_TIMER_12_BIT
#define PWM_MAX            ((1 << 12) - 1)
#define RAMP_UP_MS         300
#define RAMP_DOWN_MS       200
#define STAGGER_MS         6000       /* covering-wing offset */
#define DEFAULT_TRAVEL_MS  18000
#define CS_BLANK_MS        500        /* ignore inrush after start */
#define FROZEN_MS          2000       /* overcurrent this early = frozen = at end */
#define ENDSTOP_ZONE_FRAC  0.25f      /* overcurrent this close to target = arrived */
#define TIMEOUT_FACTOR     1.3f
#define LEARN_TIMEOUT_MS   45000
#define DEFAULT_STALL_MA   3500
#define MIN_STALL_MA       1200
#define MAX_STALL_MA       5500
#define CS_EMA_ALPHA       0.25f
#define OBSTACLE_PAUSE_MS  400

typedef enum { M_OFF, M_RAMP_UP, M_RUN, M_RAMP_DOWN } m_phase_t;

typedef struct {
    /* static config */
    gpio_num_t ina, inb, sel0;
    ledc_channel_t ch;
    adc_channel_t cs;
    /* calibration */
    uint32_t travel_ms;
    int stall_ma;
    /* runtime */
    m_phase_t phase;
    int dir;               /* +1 closing, -1 opening */
    float duty;            /* 0..1 */
    float pos_ms;          /* 0 = open .. travel_ms = closed */
    float target_ms;
    uint32_t elapsed_ms;   /* since this motor was started */
    float cs_filt_ma;
    bool overcurrent;      /* set by tick, consumed by pair logic */
    bool arrived;
    /* learn measurement */
    uint32_t meas_end_ms;
    float meas_i_sum;
    int meas_i_n;
} motor_t;

typedef enum { P_IDLE, P_MOVE, P_PAUSE, P_REVERSE, P_LEARN } pair_state_t;

typedef enum { CMD_OPEN, CMD_CLOSE, CMD_STOP, CMD_GOTO, CMD_LEARN } cmd_op_t;
typedef struct { cmd_op_t op; uint8_t pct; } cmd_t;

static motor_t s_m[2] = {
    { .ina = PIN_M1_INA, .inb = PIN_M1_INB, .sel0 = PIN_M1_SEL0,
      .ch = LEDC_CHANNEL_0, .cs = ADC_CH_M1_CS,
      .travel_ms = DEFAULT_TRAVEL_MS, .stall_ma = DEFAULT_STALL_MA },
    { .ina = PIN_M2_INA, .inb = PIN_M2_INB, .sel0 = PIN_M2_SEL0,
      .ch = LEDC_CHANNEL_1, .cs = ADC_CH_M2_CS,
      .travel_ms = DEFAULT_TRAVEL_MS, .stall_ma = DEFAULT_STALL_MA },
};

static struct {
    pair_state_t st;
    int dir;                /* direction of the active move */
    float origin_frac;      /* pair position when the move started */
    float target_frac;
    bool follower_started;
    uint32_t leader_run_ms;
    uint32_t pause_ms;      /* P_PAUSE countdown before reversing */
    int learn_step;         /* 0 = pre-close, 1 = open (measure), 2 = close (measure) */
    uint32_t learn_t_open[2], learn_t_close[2];
    float learn_i[2];
    bool calibrated;
} s_p;

static QueueHandle_t s_cmdq;
static shutter_event_cb_t s_cb;
static uint8_t s_last_reported_pct = 255;
static bool s_last_reported_moving;

/* ---------------- low level motor ---------------- */

static void motor_write_outputs(motor_t *m)
{
    bool opening = (m->dir < 0);
    bool active = (m->phase != M_OFF);
    /* opening drives OUTA, closing drives OUTB (swap motor wires if the
     * mechanics run backwards) */
    gpio_set_level(m->ina, active && opening);
    gpio_set_level(m->inb, active && !opening);
    /* route the active high side to the current-sense pin */
    gpio_set_level(m->sel0, opening ? SEL0_LEVEL_FOR_OUTA : !SEL0_LEVEL_FOR_OUTA);

    uint32_t duty = active ? (uint32_t)(m->duty * PWM_MAX) : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->ch);
}

static void motor_start(motor_t *m, int dir, float target_ms)
{
    m->dir = dir;
    m->target_ms = target_ms;
    m->phase = M_RAMP_UP;
    m->duty = 0.0f;
    m->elapsed_ms = 0;
    m->cs_filt_ma = 0.0f;
    m->overcurrent = false;
    m->arrived = false;
    m->meas_end_ms = 0;
    m->meas_i_sum = 0.0f;
    m->meas_i_n = 0;
    motor_write_outputs(m);
}

static void motor_hard_stop(motor_t *m)
{
    m->phase = M_OFF;
    m->duty = 0.0f;
    motor_write_outputs(m);
}

static void motor_soft_stop(motor_t *m)
{
    if (m->phase == M_RAMP_UP || m->phase == M_RUN) {
        m->phase = M_RAMP_DOWN;
    }
}

/* Returns true while the motor still runs. */
static bool motor_tick(motor_t *m, uint32_t dt)
{
    if (m->phase == M_OFF) {
        return false;
    }
    m->elapsed_ms += dt;

    switch (m->phase) {
    case M_RAMP_UP:
        m->duty += (float)dt / RAMP_UP_MS;
        if (m->duty >= 1.0f) {
            m->duty = 1.0f;
            m->phase = M_RUN;
        }
        break;
    case M_RAMP_DOWN:
        m->duty -= (float)dt / RAMP_DOWN_MS;
        if (m->duty <= 0.0f) {
            m->duty = 0.0f;
            m->phase = M_OFF;
        }
        break;
    default:
        break;
    }

    /* dead-reckoned position, speed roughly proportional to duty */
    m->pos_ms += m->dir * (float)dt * m->duty;
    if (m->pos_ms < 0.0f) m->pos_ms = 0.0f;
    if (m->pos_ms > m->travel_ms) m->pos_ms = m->travel_ms;

    /* current supervision at (near) full speed, after the inrush blanking */
    if (m->phase == M_RUN && m->elapsed_ms > CS_BLANK_MS) {
        int ma = sense_cs_mv_to_ma(sense_read_mv(m->cs));
        if (ma >= 0) {
            if (m->cs_filt_ma <= 0.0f) {
                m->cs_filt_ma = ma;
            } else {
                m->cs_filt_ma += CS_EMA_ALPHA * (ma - m->cs_filt_ma);
            }
            m->meas_i_sum += ma;
            m->meas_i_n++;
            if (m->cs_filt_ma > m->stall_ma) {
                m->overcurrent = true;
            }
        }
    }

    /* arrived at a soft (mid-travel) target? */
    if (m->phase == M_RUN || m->phase == M_RAMP_UP) {
        bool at_target = (m->dir > 0) ? (m->pos_ms >= m->target_ms)
                                      : (m->pos_ms <= m->target_ms);
        bool target_is_end = (m->target_ms <= 0.0f) ||
                             (m->target_ms >= (float)m->travel_ms);
        if (at_target && !target_is_end) {
            m->arrived = true;
            motor_soft_stop(m);
        }
        /* runaway guard: never run much longer than one full travel */
        if (m->elapsed_ms > (uint32_t)(m->travel_ms * TIMEOUT_FACTOR) + STAGGER_MS) {
            ESP_LOGW(TAG, "motor timeout, forcing stop");
            m->pos_ms = (m->dir > 0) ? m->travel_ms : 0.0f;
            m->arrived = true;
            motor_hard_stop(m);
        }
    }

    motor_write_outputs(m);
    return m->phase != M_OFF;
}

/* ---------------- calibration storage ---------------- */

static void cal_load(void)
{
    nvs_handle_t h;
    if (nvs_open("shutter", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint32_t t0 = 0, t1 = 0;
    int32_t s0 = 0, s1 = 0;
    uint32_t pos = 0;
    if (nvs_get_u32(h, "travel0", &t0) == ESP_OK && t0 > 3000 &&
        nvs_get_u32(h, "travel1", &t1) == ESP_OK && t1 > 3000) {
        s_m[0].travel_ms = t0;
        s_m[1].travel_ms = t1;
        s_p.calibrated = true;
    }
    if (nvs_get_i32(h, "stall0", &s0) == ESP_OK && s0 >= MIN_STALL_MA) {
        s_m[0].stall_ma = s0;
    }
    if (nvs_get_i32(h, "stall1", &s1) == ESP_OK && s1 >= MIN_STALL_MA) {
        s_m[1].stall_ma = s1;
    }
    if (nvs_get_u32(h, "pos_pct", &pos) == ESP_OK && pos <= 100) {
        s_m[0].pos_ms = s_m[0].travel_ms * pos / 100.0f;
        s_m[1].pos_ms = s_m[1].travel_ms * pos / 100.0f;
    }
    nvs_close(h);
    ESP_LOGI(TAG, "cal: travel %lu/%lu ms, stall %d/%d mA, calibrated=%d",
             (unsigned long)s_m[0].travel_ms, (unsigned long)s_m[1].travel_ms,
             s_m[0].stall_ma, s_m[1].stall_ma, s_p.calibrated);
}

static void cal_save(void)
{
    nvs_handle_t h;
    if (nvs_open("shutter", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u32(h, "travel0", s_m[0].travel_ms);
    nvs_set_u32(h, "travel1", s_m[1].travel_ms);
    nvs_set_i32(h, "stall0", s_m[0].stall_ma);
    nvs_set_i32(h, "stall1", s_m[1].stall_ma);
    nvs_commit(h);
    nvs_close(h);
}

static void pos_save(void)
{
    nvs_handle_t h;
    if (nvs_open("shutter", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u32(h, "pos_pct", shutter_position());
    nvs_commit(h);
    nvs_close(h);
}

/* ---------------- pair logic ---------------- */

static float pair_frac(void)
{
    float f0 = s_m[0].pos_ms / (float)s_m[0].travel_ms;
    float f1 = s_m[1].pos_ms / (float)s_m[1].travel_ms;
    return (f0 + f1) / 2.0f;
}

/* leader index for a direction: opening -> covering wing (M1) first,
 * closing -> covered wing (M2) first */
static int leader_for(int dir)
{
    return (dir < 0) ? 0 : 1;
}

static void pair_begin_move(float target_frac, pair_state_t st)
{
    float cur = pair_frac();
    int dir = (target_frac > cur) ? +1 : -1;
    s_p.dir = dir;
    s_p.origin_frac = cur;
    s_p.target_frac = target_frac;
    s_p.follower_started = false;
    s_p.leader_run_ms = 0;
    s_p.st = st;

    motor_t *lead = &s_m[leader_for(dir)];
    motor_start(lead, dir, target_frac * lead->travel_ms);
}

static void pair_start_follower(void)
{
    int fi = 1 - leader_for(s_p.dir);
    motor_t *m = &s_m[fi];
    if (m->phase == M_OFF && !m->arrived) {
        motor_start(m, s_p.dir, s_p.target_frac * m->travel_ms);
    }
    s_p.follower_started = true;
}

static void pair_all_hard_stop(void)
{
    motor_hard_stop(&s_m[0]);
    motor_hard_stop(&s_m[1]);
}

/* Handle an overcurrent event on one motor during a normal move.
 * Returns true if the pair must abort into obstacle reversal. */
static bool handle_overcurrent(motor_t *m)
{
    m->overcurrent = false;

    /* frozen shutter right after start: treat as already at the end */
    if (m->elapsed_ms < CS_BLANK_MS + FROZEN_MS) {
        ESP_LOGW(TAG, "early overcurrent -> treating as end of travel");
        m->pos_ms = (m->dir > 0) ? m->travel_ms : 0.0f;
        m->arrived = true;
        motor_hard_stop(m);
        return false;
    }

    /* close enough to the end / target: this is the end stop */
    float end_ms = (m->dir > 0) ? m->travel_ms : 0.0f;
    float dist = (end_ms > m->pos_ms) ? end_ms - m->pos_ms : m->pos_ms - end_ms;
    if (dist < m->travel_ms * ENDSTOP_ZONE_FRAC) {
        m->pos_ms = end_ms;
        m->arrived = true;
        motor_hard_stop(m);
        return false;
    }

    /* genuine obstacle mid travel */
    ESP_LOGW(TAG, "obstacle detected at %.0f%%", 100.0f * m->pos_ms / m->travel_ms);
    return true;
}

static void report(bool force)
{
    uint8_t pct = shutter_position();
    bool moving = (s_p.st != P_IDLE);
    if (s_cb && (force || pct != s_last_reported_pct || moving != s_last_reported_moving)) {
        s_last_reported_pct = pct;
        s_last_reported_moving = moving;
        s_cb(pct, moving);
    }
}

/* ---------------- learn cycle ---------------- */

static void learn_begin_step(int step)
{
    s_p.learn_step = step;
    float target = (step == 1) ? 0.0f : 1.0f;   /* 0=pre-close(1.0), 1=open, 2=close */
    if (step == 0) {
        target = 1.0f;
    }
    /* pretend we are at the far end so the full travel is attempted */
    s_m[0].pos_ms = (target > 0.5f) ? 0.0f : s_m[0].travel_ms;
    s_m[1].pos_ms = (target > 0.5f) ? 0.0f : s_m[1].travel_ms;
    pair_begin_move(target, P_LEARN);
}

static void learn_motor_done(int idx)
{
    motor_t *m = &s_m[idx];
    if (s_p.learn_step == 1) {
        s_p.learn_t_open[idx] = m->meas_end_ms;
    } else if (s_p.learn_step == 2) {
        s_p.learn_t_close[idx] = m->meas_end_ms;
        if (m->meas_i_n > 0) {
            float avg = m->meas_i_sum / m->meas_i_n;
            if (avg > s_p.learn_i[idx]) {
                s_p.learn_i[idx] = avg;
            }
        }
    }
    if (s_p.learn_step == 1 && m->meas_i_n > 0) {
        s_p.learn_i[idx] = m->meas_i_sum / m->meas_i_n;
    }
}

static void learn_finish(bool ok)
{
    if (ok) {
        for (int i = 0; i < 2; i++) {
            uint32_t t = (s_p.learn_t_open[i] + s_p.learn_t_close[i]) / 2;
            if (t < 3000 || t > LEARN_TIMEOUT_MS) {
                ok = false;
                break;
            }
            s_m[i].travel_ms = t;
            int stall = (int)(s_p.learn_i[i] * 2.0f);
            if (stall < MIN_STALL_MA) stall = MIN_STALL_MA;
            if (stall > MAX_STALL_MA) stall = MAX_STALL_MA;
            s_m[i].stall_ma = stall;
        }
    }
    if (ok) {
        s_p.calibrated = true;
        cal_save();
        ESP_LOGI(TAG, "learn ok: travel %lu/%lu ms, stall %d/%d mA",
                 (unsigned long)s_m[0].travel_ms, (unsigned long)s_m[1].travel_ms,
                 s_m[0].stall_ma, s_m[1].stall_ma);
    } else {
        ESP_LOGE(TAG, "learn cycle failed, keeping previous calibration");
    }
    /* the cycle ends fully closed */
    s_m[0].pos_ms = s_m[0].travel_ms;
    s_m[1].pos_ms = s_m[1].travel_ms;
    s_p.st = P_IDLE;
    pos_save();
    report(true);
}

/* ---------------- main task ---------------- */

static void process_cmd(const cmd_t *c)
{
    switch (c->op) {
    case CMD_STOP:
        if (s_p.st == P_MOVE || s_p.st == P_REVERSE) {
            motor_soft_stop(&s_m[0]);
            motor_soft_stop(&s_m[1]);
        }
        return;
    case CMD_LEARN:
        if (s_p.st == P_IDLE) {
            memset(s_p.learn_t_open, 0, sizeof(s_p.learn_t_open));
            memset(s_p.learn_t_close, 0, sizeof(s_p.learn_t_close));
            s_p.learn_i[0] = s_p.learn_i[1] = 0.0f;
            learn_begin_step(0);
        }
        return;
    case CMD_OPEN:
    case CMD_CLOSE:
    case CMD_GOTO: {
        if (s_p.st == P_LEARN) {
            return; /* don't interrupt calibration with moves */
        }
        float target = (c->op == CMD_OPEN) ? 0.0f :
                       (c->op == CMD_CLOSE) ? 1.0f : c->pct / 100.0f;
        if (s_p.st != P_IDLE) {
            pair_all_hard_stop();
        }
        float cur = pair_frac();
        if ((target > cur ? target - cur : cur - target) < 0.01f &&
            target > 0.001f && target < 0.999f) {
            return; /* already there (extremes are always re-driven) */
        }
        pair_begin_move(target, P_MOVE);
        return;
    }
    }
}

static void pair_tick(uint32_t dt)
{
    if (s_p.st == P_IDLE) {
        return;
    }

    if (s_p.st == P_PAUSE) {
        if (s_p.pause_ms > dt) {
            s_p.pause_ms -= dt;
        } else {
            pair_begin_move(s_p.origin_frac, P_REVERSE);
        }
        return;
    }

    /* stagger: start the follower once the leader has run long enough
     * (or has already finished its travel) */
    motor_t *lead = &s_m[leader_for(s_p.dir)];
    if (!s_p.follower_started) {
        s_p.leader_run_ms += dt;
        if (s_p.leader_run_ms >= STAGGER_MS || lead->phase == M_OFF) {
            pair_start_follower();
        }
    }

    bool any_running = false;
    for (int i = 0; i < 2; i++) {
        motor_t *m = &s_m[i];
        bool was_running = (m->phase != M_OFF);
        bool running = motor_tick(m, dt);

        if (m->overcurrent) {
            if (s_p.st == P_LEARN || s_p.st == P_REVERSE) {
                /* every overcurrent is an end stop here */
                m->overcurrent = false;
                m->meas_end_ms = m->elapsed_ms;
                m->pos_ms = (m->dir > 0) ? m->travel_ms : 0.0f;
                m->arrived = true;
                motor_hard_stop(m);
                if (s_p.st == P_LEARN) {
                    learn_motor_done(i);
                }
                running = false;
            } else if (handle_overcurrent(m)) {
                /* obstacle: stop everything, then reverse to the origin */
                pair_all_hard_stop();
                s_p.pause_ms = OBSTACLE_PAUSE_MS;
                s_p.st = P_PAUSE;
                report(true);
                return;
            } else {
                running = (m->phase != M_OFF);
            }
        }

        /* learn: a motor that stops by timeout means a failed calibration */
        if (s_p.st == P_LEARN && s_p.learn_step > 0 && was_running && !running &&
            m->meas_end_ms == 0 && m->arrived) {
            learn_finish(false);
            return;
        }
        any_running |= running;
    }

    if (!any_running && s_p.follower_started) {
        if (s_p.st == P_LEARN) {
            if (s_p.learn_step < 2) {
                learn_begin_step(s_p.learn_step + 1);
            } else {
                learn_finish(true);
            }
        } else {
            s_p.st = P_IDLE;
            pos_save();
            report(true);
        }
    }
}

static void shutter_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t report_ms = 0;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TICK_MS));

        cmd_t c;
        while (xQueueReceive(s_cmdq, &c, 0) == pdTRUE) {
            process_cmd(&c);
        }
        pair_tick(TICK_MS);

        report_ms += TICK_MS;
        if (report_ms >= 1000) {
            report_ms = 0;
            if (s_p.st != P_IDLE) {
                report(false);
            }
        }
    }
}

/* ---------------- public API ---------------- */

void shutter_init(void)
{
    /* direction / sense-select outputs */
    uint64_t mask = 0;
    for (int i = 0; i < 2; i++) {
        mask |= (1ULL << s_m[i].ina) | (1ULL << s_m[i].inb) | (1ULL << s_m[i].sel0);
    }
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* PWM */
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
    gpio_num_t pwm_pins[2] = { PIN_M1_PWM, PIN_M2_PWM };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ccfg = {
            .gpio_num = pwm_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_m[i].ch,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
        motor_hard_stop(&s_m[i]);
    }

    /* assume fully closed until learned/restored — safest for blind moves */
    s_m[0].pos_ms = s_m[0].travel_ms;
    s_m[1].pos_ms = s_m[1].travel_ms;
    cal_load();

    s_cmdq = xQueueCreate(8, sizeof(cmd_t));
    xTaskCreate(shutter_task, "shutter", 4096, NULL, 6, NULL);
}

void shutter_register_cb(shutter_event_cb_t cb) { s_cb = cb; }

static void push(cmd_op_t op, uint8_t pct)
{
    cmd_t c = { .op = op, .pct = pct };
    xQueueSend(s_cmdq, &c, 0);
}

void shutter_open(void)  { push(CMD_OPEN, 0); }
void shutter_close(void) { push(CMD_CLOSE, 100); }
void shutter_stop(void)  { push(CMD_STOP, 0); }

void shutter_goto_percent(uint8_t pct_closed)
{
    if (pct_closed > 100) pct_closed = 100;
    push(CMD_GOTO, pct_closed);
}

void shutter_start_learn(void) { push(CMD_LEARN, 0); }

uint8_t shutter_position(void)
{
    float f = pair_frac() * 100.0f;
    if (f < 0.0f) f = 0.0f;
    if (f > 100.0f) f = 100.0f;
    return (uint8_t)(f + 0.5f);
}

shutter_state_t shutter_get_state(void)
{
    switch (s_p.st) {
    case P_IDLE:    return SHUTTER_IDLE;
    case P_LEARN:   return SHUTTER_LEARNING;
    case P_PAUSE:
    case P_REVERSE: return SHUTTER_REVERSING;
    default:        return SHUTTER_MOVING;
    }
}

bool shutter_is_calibrated(void) { return s_p.calibrated; }
