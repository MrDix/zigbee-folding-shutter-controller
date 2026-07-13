/* Zigbee application layer: endpoints, clusters, commissioning. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void zb_app_start(void);
bool zb_app_joined(void);
void zb_app_factory_reset(void);

/* attribute updates (safe to call from any task) */
void zb_app_update_position(uint8_t pct_closed);
void zb_app_update_contact(uint8_t index, bool closed, bool tamper);
void zb_app_update_vin(int mv);
