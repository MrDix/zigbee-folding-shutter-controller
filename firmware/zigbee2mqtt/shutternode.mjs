/*
 * Zigbee2MQTT external converter for the ShutterNode
 * folding-shutter controller.
 *
 * Install: copy this file into your Zigbee2MQTT data directory under
 * `external_converters/` (create the folder if needed) and restart
 * Zigbee2MQTT. Written for Zigbee2MQTT 2.x.
 */
import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['ShutterNode'],
    model: 'ShutterNode',
    vendor: 'MrDix',
    description: 'Zigbee controller for 24 V DC folding-shutter window drives',
    extend: [
        m.deviceEndpoints({
            endpoints: {
                cover: 1,
                contact_1: 2, contact_2: 3, contact_3: 4, contact_4: 5,
                tamper_1: 6, tamper_2: 7, tamper_3: 8, tamper_4: 9,
            },
        }),
        m.windowCovering({controls: ['lift']}),
        ...[1, 2, 3, 4].map((n) => m.binary({
            name: `contact_${n}`,
            valueOn: [false, 0],
            valueOff: [true, 1],
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            description: `Window contact ${n} (false = window closed)`,
            endpointName: `contact_${n}`,
            access: 'STATE_GET',
            reporting: {min: 0, max: 3600, change: 1},
        })),
        ...[1, 2, 3, 4].map((n) => m.binary({
            name: `tamper_${n}`,
            valueOn: [true, 1],
            valueOff: [false, 0],
            cluster: 'genBinaryInput',
            attribute: 'presentValue',
            description: `Tamper loop ${n} (true = loop broken)`,
            endpointName: `tamper_${n}`,
            access: 'STATE_GET',
            reporting: {min: 0, max: 3600, change: 1},
        })),
        m.numeric({
            name: 'supply_voltage',
            cluster: 'genAnalogInput',
            attribute: 'presentValue',
            description: '24 V supply rail voltage',
            unit: 'V',
            precision: 1,
            access: 'STATE_GET',
            endpointNames: ['cover'],
        }),
    ],
};
