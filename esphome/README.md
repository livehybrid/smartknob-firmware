# SmartKnob Dev Kit on ESPHome

ESPHome firmware for the SeedLabs SmartKnob Dev Kit. It turns the knob into
a Home Assistant controller: each screen ("page") controls one entity, with
haptic detents to match. It replaces the SeedLabs firmware entirely.

| On the knob            | Does                                              |
|-------------------------|---------------------------------------------------|
| Turn                    | Changes the value on the current page              |
| Short press              | The page's action (toggle, play/pause, ...)        |
| Long press (~350 ms)    | Next page                                          |
| Hold (~1.2 s)           | Opens the page menu: keep turning to browse by icon and name, short press to jump there |

> **Status: running on hardware.** Built and flashed daily against a real
> Home Assistant instance. The haptics maths is also unit tested on a PC
> (`tests/`). See [First power-up](#first-power-up) for a new build.

## Contents

```
esphome/
  smartknob-devkit.yaml       your build: Wi-Fi, API key, list of pages
  smartknob-starter.yaml      test image with no Wi-Fi details or keys inside
  secrets.yaml.example        copy to secrets.yaml
  packages/
    smartknob-base.yaml       hardware, haptics, display, screensaver, menu,
                               notifications, sensors (always include)
    page-settings.yaml        on-device settings: screensaver, timeouts,
                               haptics, auto brightness (no vars - fixed page)
    page-light.yaml           brightness; press toggles
    page-light-color.yaml     RGB hue wheel; press toggles (pairs with
                               page-light.yaml for the same entity)
    page-media-volume.yaml    volume; press plays/pauses
    page-media-group.yaml     multi-room grouping for three media_players
    page-climate.yaml         target temperature; press toggles on/off
    page-cover.yaml           position (or tilt); press opens/closes/stops
    page-switch.yaml          two-position on/off knob
  images/                     menu and weather icons (RGB565 PNGs)
  tools/gen_icons.py          regenerates them (needs Pillow)
  components/smartknob/       the haptics component (C++ and Python)
  tests/                      PC unit tests for the haptics maths
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
Its pages use placeholder entities - see [Pages](#pages) to point them at
your own.

### Your own build

1. `cp secrets.yaml.example secrets.yaml` and fill it in.
2. Edit the pages in `smartknob-devkit.yaml` (see [Pages](#pages)).
3. From this folder: `esphome run smartknob-devkit.yaml` and pick the USB
   port. If the port does not appear, hold the BOOT button while plugging in.

From Home Assistant's ESPHome Builder instead: copy `components/`,
`packages/`, `images/` and `smartknob-devkit.yaml` into the add-on's config
folder (`/config/esphome/`) so the relative paths still resolve, then
install over USB from the Builder (Chrome or Edge) or download the factory
image and use <https://web.esphome.io>.

### Add it to Home Assistant

Add the device under Settings > Devices & services (it is usually
discovered), then **enable "Allow the device to perform Home Assistant
actions"** in the device's ESPHome integration options. Without that, the
knob shows values but cannot change anything - every page that controls a
real entity, and the multi-room page, depend on this being on.

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
2. **Direction**: turn the knob on any page; watch **Position** in Home
   Assistant. Clockwise should count up. If it counts down, add
   `invert_direction: true` under `smartknob:`.
3. **Press sensor**: press the knob and watch **Press strain** in Home
   Assistant. Set **Press threshold** to roughly half of a firm press and
   **Release threshold** to about half of that - both are editable live from
   Home Assistant, or from the knob's own **Settings** page once the rest is
   working. The defaults (30000 / 15000) are guesses. Until they are right,
   use the **Next page** button in Home Assistant to change pages.
4. **Display**: if red shows as blue, add `color_order: rgb` to the
   `display:` entry (the driver defaults to BGR); if colours look like a
   photo negative, set `invert_colors: false`; if the picture is rotated or
   mirrored, add `rotation: 90` (or 180, 270).

If anything gets hot, smells, or the device keeps rebooting when the motor
should move, unplug it and report the logs.

## Pages

Pages appear in the order their packages are listed. Most page packages take
`vars`:

```yaml
packages:
  base: !include packages/smartknob-base.yaml
  settings: !include packages/page-settings.yaml   # no vars - see below
  desk_lamp: !include
    file: packages/page-light.yaml
    vars:
      id: desk_lamp          # unique, letters/digits/underscores
      entity: light.desk_lamp
      name: Desk lamp        # shown on screen
```

| Package                    | Entity            | Turn                          | Press                          | Optional vars |
|-----------------------------|--------------------|--------------------------------|----------------------------------|----------------|
| `page-settings.yaml`        | none (device config) | browse settings                | edit / toggle (see below)        | none - fixed page, not a template |
| `page-light.yaml`           | `light.*`          | brightness                     | `light.toggle`                   | `step` (5 %), `color` |
| `page-light-color.yaml`     | `light.*` (RGB)    | hue, full saturation           | `light.toggle`                   | `step` (5°) |
| `page-media-volume.yaml`    | `media_player.*`   | volume                         | play/pause                       | `step` (2 %), `color` |
| `page-media-group.yaml`     | three `media_player.*` | browse rooms                | join/leave the group, or ungroup all on the hub | `color` |
| `page-climate.yaml`         | `climate.*`        | target temperature             | `climate.toggle`                 | `min_temp` (16), `max_temp` (28), `step` (0.5), `color` |
| `page-cover.yaml`           | `cover.*`          | position (or tilt)             | `cover.toggle` (position pages only) | `step` (5 %), `color`, `attribute`, `service`, `data_key`, `toggle` |
| `page-switch.yaml`          | anything on/off    | on/off with a strong detent    | `homeassistant.toggle`           | `color` |

While you turn, updates go to Home Assistant at most four times a second,
and Home Assistant's own updates are ignored for 1.5 s so the value does not
jump back.

The display turns off after `display_timeout` seconds without activity (a
plain number of seconds, not a time period - e.g. `display_timeout: "60"`)
and wakes on a turn, a press, or a hand approaching (if **Wake on
approach** is on). It doesn't go straight to black: see
[Screensaver](#screensaver) below.

Short and long presses are also sent to Home Assistant as events, for your
own automations:

```yaml
triggers:
  - trigger: event
    event_type: esphome.smartknob
    event_data:
      type: short_press   # or long_press
```

### Cover tilt

`page-cover.yaml` defaults to position (`current_position` /
`cover.set_cover_position`). For a cover that also has tilt, add a second
instance pointed at the same entity with the tilt vars, and turn `toggle`
off on it so the two pages don't both react to a press:

```yaml
blind_tilt: !include
  file: packages/page-cover.yaml
  vars:
    id: blind_tilt
    entity: cover.office_blind
    name: Blind Tilt
    attribute: current_tilt_position
    service: cover.set_cover_tilt_position
    data_key: tilt_position
    toggle: "false"
```

### Multi-room audio

`page-media-group.yaml` is fixed at three rooms: a hub and two others that
can join or leave its group. It reads back membership from the hub's own
`group_members` attribute (matched as a substring, since ESPHome can't pull
list attributes directly) and calls Home Assistant's standard
`media_player.join`/`unjoin`, so it needs entities that support grouping
(they report a `group_members` attribute when they do). Turn to browse,
press the hub to ungroup everyone, press either other room to toggle it in
or out:

```yaml
multiroom: !include
  file: packages/page-media-group.yaml
  vars:
    id: multiroom
    name: Multi-room
    entity_1: media_player.kitchen   # the hub
    label_1: Kitchen
    entity_2: media_player.bar
    label_2: Bar
    entity_3: media_player.dining_room
    label_3: Dining Room
```

### Settings, from the knob

`page-settings.yaml` isn't a template - it's tied directly to a fixed set of
the device's own config entities, so it takes no vars and is included once:
Screensaver mode, screen brightness, active timeout, notification duration,
haptics, auto brightness. Turn to browse, press a number or select entry to
edit it (turn to adjust, press again to save), press a switch entry to
toggle it immediately. It writes straight to the same entities Home
Assistant sees (`number.set`/`select.set`/`switch.toggle`), so both sides
always agree, and everything on it is flash-backed (`restore_value`/
`restore_mode`), so it survives a device reboot. The more technical tuning
(press/release thresholds, approach distance, long-press time) stays
Home-Assistant-only - edit those from Home Assistant if you need them.

### Screensaver

While idle (after `display_timeout`), the display doesn't just turn off: a
full-screen digital clock, date and current weather icon/temperature take
over, at a dimmer, configurable brightness. Controlled by the **Screensaver**
select (`Off` / `Dim clock` / `Clock then blank`) and the **Screensaver
brightness**/**Screensaver blank delay** numbers, all editable from Home
Assistant or the knob's Settings page. Weather comes from a
`weather.*` entity set with `weather_entity` in substitutions.

### Notifications

Calling the device's `notify` service from Home Assistant shows a
full-screen red alert - title and message - over whatever's on screen,
including the screensaver, with a double haptic click and the LED ring
turned red. It's dismissed by a press, or automatically after
**Notification duration** (default 8 s, editable like the other timeouts):

```yaml
action: esphome.smartknob_notify   # esphome.<device name>_notify
data:
  title: Front door
  message: Left open for 10 minutes
```

### Making your own page

Copy a page package. The pattern is:

* `lvgl: pages:` entry with `on_load: smartknob.set_profile` (the haptic feel
  and range for this page) and its widgets;
* `smartknob: on_position_change:` handler, guarded by
  `lvgl.page.is_showing` and `!id(menu_open) && !id(notify_active)`, that
  maps the knob position (`x`) to a value;
* `homeassistant` sensors to follow the entity, and `homeassistant.action`
  to change it.

Handlers from different packages are concatenated, so every page can add its
own `on_position_change`, `on_short_press` and so on. Keep them as lists,
and keep any on-screen text short and close to vertical centre - on a round
display, a long line clips against the edge of the visible circle well
before it reaches the edge of the square canvas.

## Configuring entities

Which entity each page controls is set in YAML, at build time, via that
page's `vars:` in `smartknob-devkit.yaml` - there's currently no way to
change it from Home Assistant without editing the file and reflashing.
Making that runtime-configurable (e.g. the device reading entity IDs from
Home Assistant helpers instead) would be a real design change, not a small
addition - it isn't implemented here.

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
    max_velocity: 40         # rad/s: presses ignored while spinning faster
  invert_direction: false
  control_frequency: 1kHz   # up to 5kHz; the loop backs off if it overruns
  task_core: 1
  task_priority: 20
  auto_calibrate: true      # calibrate on first boot if nothing is stored
  long_press_time: 350ms    # must be less than menu_press_time
  menu_press_time: 1200ms
  initial_profile: {preset: unbounded_detents}
  on_position_change: ...   # x: new position (int32_t). Not fired for set_position.
  on_press: ...
  on_release: ...
  on_short_press: ...
  on_long_press: ...        # fires while still held
  on_menu_press: ...        # fires while still held, after long_press_time already fired
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
| `smartknob.set_long_press_time`  | `time`                                                                  |

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
* **The WLED colour page can't read back the light's current colour.** Home
  Assistant reports it as a list (`hs_color`/`rgb_color`), and ESPHome's
  `homeassistant.sensor` only takes single numeric attributes. Turning
  always starts from the last hue the page itself sent, not necessarily
  what the light is currently showing.

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
