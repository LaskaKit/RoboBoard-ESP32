
# Laska Omni Car Example

![Laska OmniCar](laska_omni_car.jpg)

This example consists of 2 parts.

1. The vehicle with omni wheels.
2. The gamepad for vehicle control.

The vehicle and gamepad are communicating via ESP-NOW protocol.

## How to setup

1. Optain MAC addresses of gamepad and the roboboard using the sketch in `get_mac_address` directory.
2. Set the MAC address of the gamepad in the `fw_laska_omni_car` sketch and flash the roboboard.
3. Set the MAC address of the roboboard in the `fw_laska_omni_gamepad` sketch and flash the gamepad.
4. Enjoy!

### Optionally

You can setup your own LMK and PMK keys to keep the communication between roboboard and gamepad secure.
Note that keys on roboboard must be identical to those on gamepad.

## Controls

| *Input*                      | *Description*           |
| ---------------------------- | ----------------------- |
| left stick (left/right)      | left/right spin         |
| right stick (up/down)        | forward/backward motion |
| right stick (left/right)     | left/right motion       |
| L1                           | headlights              |
| triangle                     | starts the beacon       |
| square                       | stops the beacon        |
| cross                        | horn                    |
| P3 (1.5s hold)               | power off               |
