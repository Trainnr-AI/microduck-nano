# Build and flash the firmware

Getting the firmware onto an Arduino UNO Q, and the wiring it assumes. Paths are relative to the `firmware/` folder unless they start with `firmware/`.

## Wiring the firmware assumes

| Connection | UNO Q | Notes |
|---|---|---|
| Servo adapter (Waveshare Bus Servo Adapter (A), jumper on A) | D1 (TX, PB6), D0 (RX, PB7), GND | 3.3 V logic, 1 Mbaud. Waveshare's page says to wire RX to RX and TX to TX: check the silkscreen. Never connect the servo supply to a UNO Q pin |
| BNO086 | Qwiic connector | 3.3 V only; I2C address 0x4B; this is `Wire1` in a sketch, not `Wire` |
| Camera | USB-C | See the host-mode note below |
| Power | VIN | 7 to 24 V per Arduino's power specification; a 3S LiPo is inside that |

From Arduino's documentation, relevant to this build:

- **D0 / D1 are `Serial1` on UNO Q core 0.55 and later, but `Serial` on older cores**, where `Serial1` is the Bridge's own
  link and must not be touched. The sketch picks the right one at compile time and reports it (`servo_uart_is_serial1`).
- **USB host mode is not automatic when the board is powered from VIN or 5V** instead of USB-C: the camera will not be
  powered or detected until two pins on the JCTL header are bridged ("Force USB Host Mode on UNO Q" in Arduino's docs).
  JCTL is 1.8 V logic.
- The same UART that drives D1 is the microcontroller's boot console. Whether it prints anything at start-up that a servo
  could read as a command has not been checked (bench checklist, step 4).


Verified on 2026-11-14 with a UNO Q 4 GB, Debian 13, twice: as shipped (Arduino App CLI 0.7.0, `arduino:zephyr` core
0.52.0) and after updating the board (App CLI 0.14.1, router 0.11.0, on-board `arduino-cli` 1.5.1, core 1.1.0). On the
PC: `arduino-cli` 1.6.0 and core 1.1.0. Arduino's tools change quickly: if a command below fails, check
`arduino-app-cli --help` on the board and Arduino's UNO Q documentation.

### 1. Reach the board

The board's Linux side is reached with `adb` over the USB-C cable (or `ssh arduino@<name>.local` once the board is on
your network). Install Android platform-tools (`winget install Google.PlatformTools`, `brew install
android-platform-tools`, `sudo apt-get install android-sdk-platform-tools`), then:

```sh
adb devices          # the board shows up as "<serial>  device"; give it up to a minute after plugging in
adb shell            # a shell on the board, user "arduino"
```

On Linux, add the udev rules from Arduino's user manual first, or `adb` fails with a permission error.

**Windows with WSL2.** A USB device plugged into Windows is not visible inside WSL. Either

- run the Windows `adb.exe` from the WSL shell (this is what was used here: unpack platform-tools for Windows and call
  `.../platform-tools/adb.exe` with the same arguments; for `adb push`, `cd` next to the folder and give it by a
  relative name), or
- hand the device to WSL with usbipd-win, from an administrator PowerShell:
  `winget install usbipd`, then `usbipd list`, `usbipd bind --busid <BUSID>`, `usbipd attach --wsl --busid <BUSID>`
  (the UNO Q is USB ID `2341:0078`). After that the Linux `adb` in WSL sees it.

### 2. Check that the sketch compiles (on the PC, optional)

```sh
arduino-cli core update-index
arduino-cli core install arduino:zephyr          # the UNO Q board is arduino:zephyr:unoq
arduino-cli compile --fqbn arduino:zephyr:unoq firmware/app/microduck/sketch
```

This only checks the build. Flashing is done from the board itself (next step), because the board's own tools match its
loader.

### 3. Put the app on the board and start it

```sh
adb shell rm -rf /home/arduino/ArduinoApps/microduck
adb push firmware/app/microduck /home/arduino/ArduinoApps/microduck
adb shell 'export TMPDIR=/tmp; arduino-app-cli app start /home/arduino/ArduinoApps/microduck'
```

`app start` compiles the sketch on the board, flashes the microcontroller and starts the Python side in a container. Then:

```sh
adb shell arduino-app-cli app logs /home/arduino/ArduinoApps/microduck    # what main.py prints
adb shell arduino-app-cli app stop /home/arduino/ArduinoApps/microduck
```

With the default build on a bare board the log shows `motion_build: 0`, `joint_table_ready: 0`, `mode: DISARMED`. The
red LED flashes briefly once a second when disarmed (confirmed by eye on the board), and is programmed to stay on when
armed and blink fast on a fault (those two not yet seen).

Stopping the app also stops the sketch on the microcontroller: the red LED goes dark and the firmware's functions time
out until the app is started again. The board's blue and green LEDs are not driven by this firmware.

To flash only a sketch, without the app machinery (this is how a plain blink was flashed first):

```sh
adb shell 'export TMPDIR=/tmp; cd <sketch folder on the board> && arduino-cli compile --fqbn arduino:zephyr:unoq --upload .'
```

Add `-m default` when the folder has a `sketch.yaml` (it lists the Bridge's libraries).

### Admin rights on the board

As shipped, the board's `arduino` user has no password, so `sudo` refuses everything. Set one once, in a shell on
the board (`adb shell`), with `passwd`; it must be longer than a few characters. The user is already allowed to use
`sudo`.

After Arduino's software update the router's socket belongs to the group `arduino-router`, which has no members, so
a program run by `arduino` gets "permission denied". Add the user and restart the board:

```sh
sudo usermod -aG arduino-router arduino
sudo reboot
```

### A faster link (optional)

The link between the two processors runs at 115200 baud as shipped, which costs about 15 ms per control tick
([measurements](../../runtime/README.md)). 460800 baud works on the board tested and cuts that to about 6 ms.
Both ends must agree:

```sh
# on the board, once: change the router service (adds one override file; "default" removes it again)
adb push firmware/tools/set-link-speed.sh /home/arduino/
adb shell 'sudo sh /home/arduino/set-link-speed.sh 460800'

# in the sketch: set DUCK_BRIDGE_BAUD to 460800 in duck_build_config.h, then put the app on the board as above
```

This is board-wide: every other sketch, including Arduino App Lab's examples, expects 115200 and will not talk to
Linux until `set-link-speed.sh default` is run. 921600 baud and above did not work. The repository's default stays
115200 so that an unchanged board works.

### Things that went wrong on the way

- **`/data/local/tmp: no such file or directory`** when running `arduino-cli` in an `adb shell`: adb sets `TMPDIR` to an
  Android path. `export TMPDIR=/tmp` first.
- **The built-in examples did not build offline.** A board that has never been online cannot download libraries; the
  shipped examples ask for `ArxTypeTraits 0.3.1` while the board's cache holds `0.3.2`. This app's `sketch.yaml` asks for
  the versions the factory image has, so it builds without a network. After a system update, compare it with the
  `sketch.yaml` of a fresh `arduino-app-cli app new`.
- **A sketch that builds can still fail to start, silently.** With core 0.52.0 the first version of this firmware
  compiled and flashed, but none of its Bridge functions ever appeared. Narrowed down on the board: calling `cosf`,
  `asinf` or `sqrtf` from `<math.h>` was enough to stop a sketch from starting, although the build's own symbol check
  passed. The likely cause (not proven) is a loader on the microcontroller older than the installed core. Before that,
  the same core refused to link `lroundf` and the C++ exception support that `std::vector` seemed to pull in. The
  firmware therefore carries its own small math functions (`src/duck/fmath.cpp`) and uses fixed-size arrays. If the
  firmware's functions are "not available" after a flash, suspect this first, and check with `tools/router_call.py`.
- **`Failed to write memory at 0x00000000`** from OpenOCD appeared once in an upload log; the app still started with the
  new sketch running. If a flash really fails, start the app again.
- **Update the board first.** A board as shipped (core 0.52.0, App CLI 0.7.0) is far behind. Give it Wi-Fi
  (`nmcli dev wifi connect "<network>" password "<password>"` in a shell on the board, or Arduino App Lab), then run
  `arduino-app-cli system update --only-arduino --yes`. The command can end with a `permission denied` line about a
  `models` folder although the packages were upgraded: check with `arduino-app-cli version` and `arduino-cli core list`.
  Restart the board afterwards (unplug and replug; `reboot` needs the board's password): until then
  `arduino-app-cli version` still reports the old daemon. The firmware has been built, flashed and queried on both the
  shipped and the updated software; on core 1.1.0 it reports `servo_uart_is_serial1: 1`, on 0.52.0 it reported 0.
