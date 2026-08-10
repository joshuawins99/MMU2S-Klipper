# MMU2S-Klipper
Python serial driver for running the stock MMU2S/3 firmware on Klipper. Tested on an MMU2S with a Prusa i3 MK3S+.

### Current Status
This driver is currently in an early state. Bugs are possible.

### Features
* Loading/Unloading Filament
* Support for normal T0, T1, etc. commands
* Filament Cutting
* Custom Tip Shaping/Unloading Macros
* Status/Error messaging
* Ability to Read/Write to Registers on MMU to configure options such as:
    * Bowden length
    * Slow Feedrate
    * Load Feedrate
    * Cut Length
* LCD Menus that mimic original Prusa options
* On screen Filament status
* Spool Join

### Currently Unsupported
* Filament Loading/Unloading retries (should currently just pause print)

### Prerequisites
* Printer running klipper
* Free USB port
* 5V USB to Serial Adapter

### Instructions
* Unplug connector at P1 on MMU board.
* Connect USB to Serial Adapter to the following pins:
    * Pin 2: TXD -> TXD (Serial Adapter)
    * Pin 3: RXD -> RXD (Serial Adapter)
    * Pin 4: GND -> GND (Serial Adapter)
    * Note: 5V is still required for the MCU on the MMU board. This is Pin 1 on the original connector and can be depinned and inserted into Pin 1 back on the MMU board.
* Copy ```mmu2s.py``` to the extras folder within klipper.
* Add ```mmu2s.cfg``` to klipper config directory and include it in printer.cfg.
    * Ensure that the ```serial: /dev/ttyUSB0``` matches what the serial adapter registers as. This may be different depending on configuration.
* Add these variables to saved variables cfg file. These are used for state tracking.
```
[Variables]
mmu_changing_slot = 5
mmu_loaded_slot = 5
mmu_previous_slot = 5
```
* ```mmu2s_display.cfg``` is for adding on screen options for MMU related tasks. Only useful if an lcd screen is being used.

### Exposed Macros
* MMU_UNLOAD TIP_SHAPE={1,0}
* MMU_PRELOAD SLOT={0-4}
* MMU_STATUS
* MMU_LOAD SLOT={0-4}
* MMU_CUT SLOT={0-4}
* MMU_REG_READ REG_ENTRY={register name}
* MMU_RESET
* MMU_SET_SPOOLJOIN ORDER={1,2,3,4,5} ORDER=1,2,3,4,5 syntax most significant first. Setting ORDER=-1 disables spool join
