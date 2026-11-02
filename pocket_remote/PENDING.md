# Pending items — pocket_remote

Status as of 2026-11-01. Board: Waveshare ESP32-S3-Touch-AMOLED-1.8.

The firmware runs against `SimCamera`; there is no real Pocket 4P yet.
Protocol reference: [OpenPocketCine handbook](https://opencapture.org/openpocketcine/docs/)
(the 4P is called "Pocket 4 Pro" there).

### Waiting for feedback on the board

- [ ] **Tap click** — does it still sound double after the reshaped click (soft start, 1.8 kHz, buffer-aligned)?
      The log showed one click per touch, so the cause was in the audio, not the touch handling.
- [ ] **Touch test** (SETUP → About → Touch test) — do marks land under the finger at the bottom-right?
      Taps aimed at SETUP registered at x ≈ 280–300, which is the GIMB tab (SETUP is 294–368).
- [ ] **Categorized SETUP menu** — is the layout clear; anything missing or in the wrong category?
- [ ] **Moves page** (GIMB → swipe left) — presets, record, replay, delete.
- [ ] **Double-tap wake** — reliable at a natural tapping speed?
- [ ] **Tilt jitter fix** — smoothing + gentler curve near center; still jittery?

### Proposed, not yet approved

- [ ] **Detect the current profile** — compare camera settings with saved profiles; show
      `Vlog` / `Vlog (modified)` / `Custom` on HOME, "✓ Active" on PROF and the sheet,
      and a "Save changes to Vlog" button.
- [ ] **"Applied - connect FrameTap now"** toast after release.
- [ ] **Zoom control** on the GIMB page (command `02/B8` documented; tele lens only partly verified).
- [ ] **Simulated "FrameTap connected"** switch, to design the "camera busy" screen.
- [ ] **Auto-connect** setting (on startup and after an unexpected drop; never after a release or
      manual Disconnect, so FrameTap keeps the camera). Deferred by request.

### Planned features (requested)

- [ ] **Auto panorama** — the gimbal steps across the scene and takes a photo at each position,
      ready to stitch on the Mac.
      - Settings: horizontal span (e.g. 90° / 180° / 270°), rows (1–3 for taller panoramas),
        overlap (~30%), settle time before each shot.
      - Uses the timed gimbal target (`04/14`) + photo shutter; switches to Photo mode first and
        locks exposure/WB so frames match.
      - Progress on the board ("photo 5 of 12"), cancel button; can be built in the simulator first.
- [ ] **Day-to-night timelapse ("holy grail")** — photos at an interval through sunset/sunrise while
      the board gradually ramps exposure so the result doesn't flicker or go dark.
      - Settings: interval, duration (or end time), target brightness; optional gimbal motion
        (reuse Moves keyframes) for a motion holy-grail.
      - Ramps ISO and shutter in small steps (never jumps) as light changes; needs a brightness
        reading — the camera's reported exposure/EV values if available, else a time-based
        sunset curve.
      - Locks WB (or ramps it warm→cool), shows frames taken / time left / battery estimate.
      - Open question for the real camera: what exposure values the status messages report.

### First tests when the real camera arrives (in this order)

1. [ ] Pair over **Bluetooth** and change one setting (ISO). Settings commands were only captured over Wi-Fi.
2. [ ] **FrameTap coexistence**: board over BLE with FrameTap connected, then over Wi-Fi.
       Does the camera hand over, refuse the board, or drop both?
3. [ ] **Apply profile → release → FrameTap reconnects**: does FrameTap reconnect by itself, and do settings persist?
4. [ ] **Gimbal**: find the real stick command (currently `04/??`), check that "straight ahead"
       fits the −48°…+225° pan / −44°…+70° tilt range, and whether attitude reports arrive over BLE.
5. [ ] **Model detection** (Pocket 4 vs 4P) from the BLE name/advert.
6. [ ] **Camera sleep / wake / snapshot** (DJI Osmo BLE protocol, documented only for Action 4 / 5 Pro):
       sleep via `00/1A` = 3; wake via a 2 s BLE advert `"WKP"` + camera MAC (reversed), only within
       30 min of sleeping; snapshot = wake advert then key report `00/11` key `0x03`.
       Check whether the Pocket 4P responds at all, and whether it works with FrameTap paired.
       Also check the claim that FrameTap's record button wakes a sleeping 4P (unconfirmed online).

### Real-camera implementation

- [ ] `RealCamera` implementing `Camera` (`src/camera.h`): BLE (service `fff0`, notify `fff4`, write `fff5`),
      app-level pairing `07/45`, Wi-Fi credentials `07/07` / `07/0E`, UDP DUML on `192.168.2.1:9004`.
- [ ] DUML framing and CRC8/CRC16 in C++.
- [ ] Read option lists from the camera (`camcap_video_format`) instead of the simulator's fixed lists.
- [ ] Proper "release": close BLE and the Wi-Fi datalink so the camera is free for FrameTap.
- [ ] Clear "Camera busy - disconnect FrameTap" message if the camera refuses the board.
- [ ] Profile fallback for missing options (e.g. D-Log2 → D-Log on a regular Pocket 4).
- [ ] Wi-Fi / BLE signal strength in the header.
- [ ] Expect flash use to grow from ~505 KB to ~1.6–1.8 MB with the Wi-Fi + BLE stacks (6.25 MB available).

### Parked until the camera arrives

- [ ] **Media / file transfer** — preferred: board sends "join shared Wi-Fi" (`07/47`) so the camera joins
      the home network and the Mac downloads at full speed. Extras: clip list with star/delete on the
      board; download of single short clips to the microSD card (~1–2 MB/s).
- [ ] **Multi-camera control** over BLE (board supports ~3 connections by default) with "apply to all".

### Smaller follow-ups

- [ ] Rename profiles (currently auto-named "Custom N").
- [ ] Touch debounce (40 ms) turned out not to be the double-click cause; keep or remove.
- [ ] Gimbal box is drawn to scale over the documented range, so the crosshair sits near the left;
      revisit once the real "straight ahead" position is known.
