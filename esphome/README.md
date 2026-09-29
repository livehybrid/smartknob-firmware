# SmartKnob Dev Kit on ESPHome

ESPHome firmware for the SeedLabs SmartKnob Dev Kit. It turns the knob into
a Home Assistant controller: each screen ("page") controls one entity, with
haptic detents to match. It replaces the SeedLabs firmware entirely.

| On the knob  | Does                                            |
|--------------|-------------------------------------------------|
| Turn         | Changes the value on the current page           |
| Short press  | The page's action (toggle, play/pause, ...)     |
| Long press   | Next page                                       |

> **Status: not yet run on hardware.** The full firmware builds with ESPHome
> 2026.9.0 without compiler warnings, and the haptics maths is unit tested
> on a PC (`tests/`). Treat the first flash as a test run: see
> [First power-up](#first-power-up).

## Contents

```
esphome/
  smartknob-devkit.yaml     device config: Wi-Fi, API, list of pages
  smartknob-starter.yaml    test image with no Wi-Fi details or keys inside
  secrets.yaml.example      copy to secrets.yaml
  packages/
    smartknob-base.yaml     hardware, haptics, display, sensors (always include)
    page-setup.yaml         raw readings for first-time checks
    page-light.yaml         brightness; press toggles
    page-media-volume.yaml  volume; press plays/pauses
    page-climate.yaml       target temperature; press toggles on/off
    page-cover.yaml         position; press opens/closes/stops
    page-switch.yaml        two-position on/off knob
  components/smartknob/     the haptics component (C++ and Python)
  tests/                    PC unit tests for the haptics maths
```

## Flashing

The first flash must be over USB: ESPHome uses a different partition table
from the SeedLabs firmware, so it cannot be installed over the air on top.
Later updates can be over Wi-Fi.

**Back up the SeedLabs firmware first** if you might want it back (restores
it exactly). Connect USB-C and run:
```
esptool.py --chip esp32s3 read_flash 0 0x1000000 smartknob-seedlabs-backup.bin
```
To restore later: `esptool.py --chip esp32s3 write_flash 0 smartknob-seedlabs-backup.bin`.

### Quickest: the starter image

`smartknob-starter.yaml` builds an image with no Wi-Fi details or keys in
it, so a built copy (`smartknob-starter.factory.bin`) can be flashed as-is:

1. Open <https://web.esphome.io> in Chrome or Edge, plug in the knob,
   choose **Connect** and pick its USB port, then **Install** and choose
   the `.factory.bin` file. Tick **Erase device** if you are replacing a
   build that was crash-looping: ESPHome keeps its boot-failure count in
   flash and would otherwise start in safe mode.
2. When it has finished, set up Wi-Fi from the same page (it asks over
   USB). If it does not offer to, join the **SmartKnob setup** Wi-Fi network
   from a phone and enter your Wi-Fi details on the page that opens.
3. [Add it to Home Assistant](#add-it-to-home-assistant).

The starter image has no encryption (ESPHome 2026.9.0 crashes in safe mode
when the API expects a key that is not built in), so use it on a trusted
network only and move to your own build, which has a key, after testing.
Its pages use placeholder entities.

### Your own build

1. `cp secrets.yaml.example secrets.yaml` and fill it in.
2. Edit the pages in `smartknob-devkit.yaml` (see [Pages](#pages)).
3. From this folder: `esphome run smartknob-devkit.yaml` and pick the USB
   port. If the port does not appear, hold the BOOT button while plugging in.

From Home Assistant's ESPHome Builder instead: copy `components/`,
`packages/` and `smartknob-devkit.yaml` into the add-on's config folder
(`/config/esphome/`) so the relative paths still resolve, then install over
USB from the Builder (Chrome or Edge) or download the factory image and use
<https://web.esphome.io>.

### Add it to Home Assistant

Add the device under Settings > Devices & services (it is usually
discovered), then **enable "Allow the device to perform Home Assistant
actions"** in the device's ESPHome integration options. Without that, the
knob shows values but cannot change anything.

## First power-up

Watch the logs (`esphome logs smartknob-devkit.yaml`) and keep your hands off
the knob for the first 30 s.

1. **Calibration** runs by itself about 3 s after the first boot (about 20 s,
   the knob turns back and forth). The result is saved and logged:
   ```
   Motor calibration saved: 7 pole pairs, direction 1, zero offset 1.234
   ```
   Re-run it any time with the **Calibrate motor** button in Home Assistant.
   To skip it on reflashes, pin the values in the YAML:
   ```yaml
   smartknob:
     motor:
       calibration:
         pole_pairs: 7
         direction: 1
         zero_offset: 1.234
   ```
   If it fails with "the knob did not turn as expected", check the motor
   cable and that nothing touches the knob, then try again.
2. **Direction**: on the Setup page, turning clockwise should count up. If it
   counts down, add `invert_direction: true` under `smartknob:`.
3. **Press sensor**: press the knob and watch **Press strain** in Home
   Assistant (or on the Setup page). Set **Press threshold** to roughly half
   of a firm press and **Release threshold** to about half of that. The
   defaults (30000 / 15000) are guesses. Until they are right, use the
   **Next page** button in Home Assistant to change pages.
4. **Display**: if red shows as blue, add `color_order: rgb` to the
   `display:` entry (the driver defaults to BGR); if colours look like a
   photo negative, set `invert_colors: false`; if the picture is rotated or
   mirrored, add `rotation: 90` (or 180, 270).

If anything gets hot, smells, or the device keeps rebooting when the motor
should move, unplug it and report the logs.

## Pages

Pages appear in the order their packages are listed. Each page package takes
`vars`:

```yaml
packages:
  base: !include packages/smartknob-base.yaml
  desk_lamp: !include
    file: packages/page-light.yaml
    vars:
      id: desk_lamp          # unique, letters/digits/underscores
      entity: light.desk_lamp
      name: Desk lamp        # shown on screen
```

| Package                  | Entity         | Turn                     | Press                 | Optional vars                         |
|--------------------------|----------------|--------------------------|-----------------------|---------------------------------------|
| `page-light.yaml`        | `light.*`      | brightness               | `light.toggle`        | `step` (5 %), `color`                 |
| `page-media-volume.yaml` | `media_player.*` | volume                 | play/pause            | `step` (2 %), `color`                 |
| `page-climate.yaml`      | `climate.*`    | target temperature       | `climate.toggle`      | `min_temp` (16), `max_temp` (28), `step` (0.5), `color` |
| `page-cover.yaml`        | `cover.*`      | position                 | `cover.toggle`        | `step` (5 %), `color`                 |
| `page-switch.yaml`       | anything on/off | on/off with a strong detent | `homeassistant.toggle` | `color`                         |
| `page-setup.yaml`        | none           | raw position             | none                  | none                                  |

While you turn, updates go to Home Assistant at most four times a second,
and Home Assistant's own updates are ignored for 1.5 s so the value does not
jump back.

The display turns off after 30 s without activity and wakes when a hand
comes near (proximity sensor), on a turn or on a press. Change the timeout
with `substitutions: display_timeout: 60s` in the device YAML.

Short and long presses are also sent to Home Assistant as events, for your
own automations:

```yaml
triggers:
  - trigger: event
    event_type: esphome.smartknob
    event_data:
      type: short_press   # or long_press
```

### Making your own page

Copy a page package. The pattern is:

* `lvgl: pages:` entry with `on_load: smartknob.set_profile` (the haptic feel
  and range for this page) and its widgets;
* `smartknob: on_position_change:` handler, guarded by
  `lvgl.page.is_showing`, that maps the knob position (`x`) to a value;
* `homeassistant` sensors to follow the entity, and `homeassistant.action`
  to change it.

Handlers from different packages are concatenated, so every page can add its
own `on_position_change`, `on_short_press` and so on. Keep them as lists.

## Component reference

```yaml
smartknob:
  id: knob
  motor:
    uh_pin: GPIO8           # TMC6300 gate inputs
    ul_pin: GPIO16
    vh_pin: GPIO18
    vl_pin: GPIO7
    wh_pin: GPIO17
    wl_pin: GPIO15
    pwm_frequency: 25kHz
    dead_time: 500ns
    supply_voltage: 5V
    voltage_limit: 2.89V    # default: supply / sqrt(3), the most SVPWM can apply
    calibration_voltage: 2.5V
    calibration: {pole_pairs: 7, direction: 1, zero_offset: 1.234}  # optional
  encoder:                  # MT6701 over SSI
    clk_pin: GPIO13
    data_pin: GPIO14
    cs_pin: GPIO11
    spi_host: spi3          # takes the whole SPI peripheral
    frequency: 4MHz
    filter: 300Hz           # low-pass for the detent engine; 0 = off
  strain_gauge:             # HX711, optional
    dout_pin: GPIO21
    clk_pin: GPIO1
    press_threshold: 30000  # raw counts from the idle baseline
    release_threshold: 15000
    direction: both         # both, positive or negative
    press_click: 2.5        # haptic click strength on press (V), 0 = none
    release_click: 1.5
    click_duration: 6ms
    max_velocity: 8         # rad/s: presses ignored while spinning faster
  invert_direction: false
  control_frequency: 1kHz   # up to 5kHz; the loop backs off if it overruns
  task_core: 1
  task_priority: 20
  auto_calibrate: true      # calibrate on first boot if nothing is stored
  long_press_time: 600ms
  initial_profile: {preset: unbounded_detents}
  on_position_change: ...   # x: new position (int32_t). Not fired for set_position.
  on_press: ...
  on_release: ...
  on_short_press: ...
  on_long_press: ...        # fires while still held
  on_calibration: ...       # success: bool
```

Actions:

| Action                           | Options                                                                 |
|----------------------------------|-------------------------------------------------------------------------|
| `smartknob.set_profile`          | `preset`, `min_position`, `max_position` (below min = unbounded), `position_width` (degrees), `detent_strength`, `endstop_strength`, `snap_point`, `snap_point_bias`, `detent_positions` (up to 8), `position`. All but `preset` and `detent_positions` accept lambdas. |
| `smartknob.set_position`         | `position`                                                              |
| `smartknob.click`                | `strength` (V, default 1.5), `duration` (default 6ms)                   |
| `smartknob.calibrate`            |                                                                         |
| `smartknob.set_haptics`          | `enabled`                                                               |
| `smartknob.set_press_thresholds` | `press`, `release`                                                      |

Presets (from Scott Bezek's original demos): `unbounded`, `unbounded_detents`,
`bounded_0_10`, `multi_rev`, `on_off`, `return_to_centre`, `fine`,
`fine_detents`, `coarse_strong`, `coarse_weak`, `magnetic`,
`return_to_centre_detents`. Explicit options override the preset's values.

Sensors (`platform: smartknob`): `position`, `strain`, `loop_time`.
Binary sensor: `pressed`. Lambdas can call `id(knob).get_position()`,
`is_pressed()`, `is_calibrated()`, `is_calibrating()`.

## Known issues

* **ESPHome 2026.9.0 safe mode with API encryption.** After 10 failed
  boots ESPHome starts in safe mode, which never creates the API server;
  the OTA component still reads its encryption key from it and crashes. The
  starter image has no encryption, so it is unaffected. Builds with an API
  key reach safe mode but crash on an over-the-air upload there, so recover
  those over USB.

## How it works

A FreeRTOS task pinned to core 1 runs the control loop at 1 kHz, paced by a
hardware timer (if it overruns its period repeatedly it sleeps a tick, so
ESPHome's main loop on the same core always gets time): read the MT6701 over SPI, filter, run the detent engine
(a port of Scott Bezek's algorithm, with one sign convention and
rate-independent maths), then apply the torque with voltage-mode FOC and
space-vector PWM on the MCPWM peripheral (complementary outputs with dead
time). The same task polls the HX711, so press detection and the press click
do not depend on the ESPHome main loop. ESPHome talks to the task only
through a command queue and a locked state snapshot. The motor coasts while
an OTA update writes flash.

Unit tests (PC, no hardware): `make -C tests`.
