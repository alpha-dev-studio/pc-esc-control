# ESC Control

Run a brushless motor's ESC from your PC with a **Raspberry Pi Pico** or an **Arduino Uno / Nano** as the
signal generator. A small desktop app gives you a throttle slider, hold-to-ramp buttons, calibration, a
bidirectional (forward / reverse) mode, and a built-in console. The board produces a standard PWM throttle
signal, so it works with **any ESC that accepts one** — BLHeli_S, BLHeli_32 in PWM mode, SimonK, and most
brushed-motor ESCs.

The app works out which board it is talking to and labels everything accordingly.

> **Test status.** PWM output on the Pico and Uno with a BLHeli_S ESC has been run on real hardware.
> *ALWAYS REMOVE THE PROPELLOR WHILE TESTING*
---

## Features

- **Throttle slider** — click, drag or scroll for 0–100 %, plus ±1 / ±5 and jump-to buttons.
- **Hold-to-ramp** — hold ▲ / ▼ (or the **R** / **F** keys) to ramp smoothly at an adjustable rate.
- **STOP** — Space or Esc cuts throttle immediately. USE WITH CAUTION YOUR MOTOR WILL FLY AWAY IF ITS NOT ANCHORED
- **Motor direction** — *Normal* (one direction) or *Bidirectional* for an ESC set to 3D / bidirectional mode,
  with PPM min / centre / max and FORWARD / REVERSE buttons.
- **Calibration** — set the pulse for 0 %, for the first 1 % where the motor actually starts, and for 100 %;
  ramp limit and failsafe timeout. A raw pulse test helps you find those numbers.
- **Save to the board** — calibration survives a power cycle (Pico flash / Uno EEPROM).
- **Board detection** — Pico or Uno is detected on connect; names, units and messages follow.
- **Console** — every board command can be typed into the app; `ports`, `connect`, `disconnect`, `clear` and
  `quit` are handled by the app itself.
- **Failsafe** — if the board stops hearing from the app it brings the throttle back to idle.

## Supported hardware

| | Raspberry Pi Pico | Arduino Uno / Nano |
|---|---|---|
| Chip | RP2040 | ATmega328P |
| Signal pin | **GP17** (physical pin 22) | **D9** |
| Settings storage | Flash (emulated EEPROM) | EEPROM |
| Sketch | `firmware/pico/pico.ino` | `firmware/uno/uno.ino` |
| Connecting | Instant | Board resets when the port opens; the app waits ~2.6 s |

Both speak the same serial protocol, so the app is identical for either.

## Wiring

| Board pin | ESC wire |
|---|---|
| GP17 (Pico) or D9 (Uno / Nano) | Signal |
| GND | Ground |

- Power the ESC from its **battery or a bench supply**, never from the board's USB.
- Do **not** connect the ESC's 5 V (BEC) wire to the board while USB is plugged in.
- Most ESC signal inputs accept 3.3 V (Pico) and 5 V (Uno) logic; check yours if unsure.

## INSTALLATION

### BOARDS

#### PICO
Drag and drop the pico firmware .uf2 file from releases on the RPI-RP2 drive
For RPI-RP2 drive to show up first you have to plug the pico in with the bootsel button pressed

#### ARDUINO UNO

**Arduino Uno**

1. Open `firmware/uno/uno.ino` in the Arduino IDE. The `Servo` and `EEPROM` libraries come with it.
2. Choose board **Arduino Uno**.
3. Pick the port and press **Upload**.

> The Arduino IDE needs a sketch's folder name to match its `.ino` name, which is why the files are
> `pico/pico.ino` and `uno/uno.ino`.

### APP
Grab the app from releases and open it

Pick the COM port, press **Connect**. The header shows the detected board once it answers.

#### Using the app

##### Normal mode

For an ESC set to one direction of rotation. **0 %** is the *Off* pulse (default 1000 µs); **1 %** is the *Start*
pulse (default 1187 µs, the lowest pulse where the motor actually spins); **100 %** is the *Max* pulse
(default 1830 µs, where it stops speeding up). Everything in between is linear.

To calibrate (propeller off):

1. Use **Raw pulse test** to try pulse widths. Find the lowest where the motor reliably spins, and the point
   where more pulse no longer means more speed.
2. Enter them as **Start** and **Max**, press **Apply**.
3. Press **Save to board** so they survive a power cycle.

##### Bidirectional mode

First set the ESC itself to bidirectional in your configurator (BLHeliSuite, ESC-Configurator, ...). Then pick
**Bidirectional** under *Motor direction*.

- The stop pulse becomes the **PPM centre**; forward runs centre → **PPM max**, reverse runs centre → **PPM min**.
  Defaults are BLHeli_S's own: **min 1148, centre 1488, max 1832 µs**. If you've changed them in the ESC, change
  them here to match.
- The slider is still 0–100 %; **FORWARD / REVERSE** choose the direction. Reverse shows as a minus sign.
- Changing direction drops the throttle to centre first — it never flips under load.
- The Off / Start / Max rows and the raw pulse test are hidden or locked in this mode.
- Use **Save to board** to make it the boot mode.

> **Switching back to Normal is dangerous on an ESC that is still set to bidirectional.** Normal mode idles near
> 1000 µs, which a bidirectional ESC reads as full reverse. The app asks for confirmation. Only switch back if
> the ESC itself is set to one direction, and keep the propeller off.

##### Keyboard

| Key | Action |
|---|---|
| ← / → | −1 / +1 % |
| ↓ / ↑ | −5 / +5 % |
| PgDn / PgUp | −10 / +10 % |
| Hold **F** / **R** | Ramp down / up |
| Space / Esc | STOP |

##### Serial command reference

You can send these from the app's console, or from any serial terminal at **115200 baud** with `\n` line endings.

| Command | What it does |
|---|---|
| `<0-100>` | Set throttle % (silent — what the app sends) |
| `s <0-100>` | Set throttle % with a confirmation line |
| `us <800-2200>` | Set a raw pulse width in µs (debug) |
| `stop` | Cut throttle to idle immediately |
| `status` | Print the current state (see below) |
| `dirmode normal\|bidir` | Normal, or bidirectional (ESC set to 3D) |
| `dir fwd\|rev` | Bidirectional only: spin direction (throttle drops to centre first) |
| `ppmmin <us>` / `ppmctr <us>` / `ppmmax <us>` | Bidirectional: full-reverse / stop / full-forward pulse (defaults 1148 / 1488 / 1832) |
| `min <us>` | Normal: the 0 % pulse (default 1000) |
| `start <us>` | Normal: the 1 % pulse (default 1187) |
| `max <us>` | Normal: the 100 % pulse (default 1830) |
| `slew <1-100>` | Ramp limit, % of range per 20 ms (default 1) |
| `timeout <ms>` | Failsafe time, 200–60000, `0` disables (default 1000) |
| `debug on\|off` | Stream the status line twice a second |
| `save` | Store mode, calibration, ramp limit and timeout |
| `defaults` | Restore factory values and erase what was saved |
| `arm` | Hold the idle pulse for 3 s (for an ESC that wants an arming period) |
| `ping` | Refresh the failsafe timer (the app does this for you) |
| `reboot` | Restart the board |
| `bootsel` | Pico only: restart into the USB bootloader |
| `help` | List the commands |

Changing `min` / `start` / `max` / the PPM values, the mode or the direction, and `save` / `defaults`, all cut
throttle to idle first. A serial terminal must send `ping` (or a throttle command) at least once per failsafe
period, or use `timeout 0` for hands-on typing.


## MANUALLY BUIDLING APP AND FIRMWARE 

**Raspberry Pi Pico**

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (2.x).
2. *File → Preferences →* "Additional boards manager URLs", add
   `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
3. *Tools → Board → Boards Manager* → install **Raspberry Pi Pico/RP2040** (Earle F. Philhower, III).
4. Open `firmware/pico/pico.ino`, choose board **Raspberry Pi Pico**.

### BUILDING THE APP 

1. Install [Python for Windows](https://www.python.org/downloads/) (3.x).
2. Open the `app` folder and double-click **`build_exe.bat`**. It installs `pip`, PyInstaller and pyserial,
   closes any running copy of the app, and builds.
3. Your exe is `app/dist/esc_control_gui.exe`. Run the script again any time you change `esc_control.py`.

Pick the COM port, press **Connect**. The header shows the detected board once it answers.



## Safety notes

- **Propeller off** for every first test, every new calibration, and every mode change.
- The failsafe brings throttle back to idle *at the ramp rate* (`slew`), not instantly. At the default 1 % per
  20 ms, coming down from full throttle takes a few seconds. Raise `slew` if you want it faster, and keep the
  STOP button in reach.
- An Uno resets whenever the app opens its serial port, so the signal pin floats for a moment. Keep the
  ESC in a safe state while connecting.
- Normal vs Bidirectional must match the ESC's own setting. A mismatch can mean full reverse at idle.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Motor won't spin until high throttle | **Start** is too low — raise it in the calibration panel |
| ESC beeps and never arms | Mode mismatch (Normal vs Bidirectional), or the idle pulse isn't what the ESC expects — check the PPM values against the ESC's settings |
| "No data from board" | Wrong port, another program (e.g. Arduino Serial Monitor) holds it, or the sketch isn't running |
| Uno connects slowly | Expected: the board resets on connect and the app waits ~2.6 s |
| Labels say "board" instead of Pico / Uno | The app can't tell which board it is yet — it settles once the board answers `status` (older firmware without the `board=` field falls back to a guess from the USB ID) |
| Pico doesn't show as a port | Unplug, hold BOOTSEL, replug and re-flash `pico.ino` |
| Failsafe trips constantly in a terminal | A plain terminal isn't sending `ping`; use `timeout 0` |

## Limitations

- **PWM output only.** There is no OneShot, Multishot or DShot. In particular, an ESC flashed with **Bluejay**
  (which takes DShot only) can't be driven by this; use BLHeli_S firmware or any other PWM-capable ESC.
- One ESC per board.
- The Uno version has 2 KB of RAM, so it has the same commands but no `bootsel`.

## Repository layout

```
firmware/
  pico/pico.ino      Raspberry Pi Pico sketch (Arduino-Pico core)
  uno/uno.ino        Arduino Uno / Nano sketch
app/
  esc_control.py     desktop app (Python, Tkinter, pyserial)
  build_exe.bat      builds a standalone Windows exe with PyInstaller
requirements.txt     Python dependency for the app
```

## License

See `LICENSE`.
