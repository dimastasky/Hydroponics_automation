# Hydroponics automation based on Arduino Mega 2560 r2

## What does it control?
- Temperature
- Humidity
- Light cycle
- pH (it shows and controls pH, but control alorithm should be modified)
- TDS (only shows)
- Water level (in future)

## Interface
- Display 16x2
- Keyboard 4x4

You will be prompted to configure your hydroponics when system starts.

## What is planned to be done?

### Interface
- Better interface with ability to modify config while system is running
- Ability to save setting to EEPROM or another memory

### Algorithms
- Better algorithms to control pH and TDS

### IoT
- Ability to collect data from sensors and system reactions on system state changes (for example reactions to added fertilizers)
- Ability to control device by WiFi, Bluetooth, LoRaWAN or another wireless technology
- Machine learning to analyse collected data

### Test every system module, especially pH and TDS control
