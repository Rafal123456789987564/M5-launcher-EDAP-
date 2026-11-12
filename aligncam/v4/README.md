# ALIGNCAM V4 — Precision (experimental)

**Phone-based optical wheel target tracking with guided setup.**

Run both phones in Chrome/Edge/Safari at the same HTTPS address:
`https://rafal123456789987564.github.io/M5-launcher-EDAP-/aligncam/v4/`

## Main functions

- Two live smartphone camera feeds, paired by 6-digit room PIN using WebRTC.
- Automatic identification of 12 coded ArUco DICT_4X4_1000 markers per target.
- Three distinct A4 landscape boards: LEFT (IDs 0–11), RIGHT (IDs 20–31) and REF (IDs 40–51).
- Red full-frame warning when the detected target is for the wrong side.
- Pose estimation using planar homography and fitted camera intrinsics (when sufficient diverse views exist), plus tracking quality gates.
- Smartphone gravity-sensor-based target upright and vertical-tilt indications when supported; unavailable readings are flagged rather than invented.
- Two feeds side-by-side in landscape mode and readable angle/quality values beneath each feed.
- Guided setup with a small built-in animation and automatic REF capture from stable frames.
- Vehicle make/model/year labels for Volkswagen, Audi, SEAT and Škoda (NO invented alignment specifications).
- Separate FRONT and REAR capture plus JSON export.

## Print target boards

Open `targets.html` (or use the **PRINT 3 A4 TARGETS** control).
Print A4 LANDSCAPE at exactly **100% / actual size**, with all margins and scaling disabled.
Measure the 100 mm control line. Mount boards on rigid FLAT panels and avoid gloss.
The `ALIGNCAM_V4_TARGETS_A4_PRINT_100_PERCENT.pdf` contains the same three pages.

## Quick workflow

1. Print all three boards.
2. Connect both phones to Wi-Fi and open the website on each. Allow camera access.
3. Set LEFT phone to Create room; type its PIN on RIGHT phone and Join.
4. Enable the gravity sensor if prompted (used for true-vertical indications).
5. Keep a common REF target stationary. Capture it on both cameras before moving to wheel targets. When each reference stays stable, it is captured automatically.
6. Mount LEFT and RIGHT targets on the appropriate front wheels and choose FRONT. See yaw and vertical tilt, where supported.
7. Move the measurement plates to rear wheels and choose REAR, keeping common reference unchanged. Check signal quality before believing a reading.
8. Export results as JSON.

## Important accuracy limitations

This is experimental software; it is NOT a certified commercial wheel-alignment system. Two independent phone cameras and visually coded targets alone are insufficient to prove absolute toe and camber referenced to a vehicle's geometric center/thrust axis. Individual lens intrinsics, camera-to-IMU rotation, wheel runout, target-to-wheel parallelism and chassis reference must be calibrated and independently validated before actual wheel-angle adjustment. A single laser is useful for rough centering but does not automatically provide full 3D extrinsics. The on-screen *relative toe* is a relative target angle, not manufacturer specification. Red/green tracking is a quality indicator and not a declaration of correct vehicle alignment.

## License

Copyright (c) 2026 EDAP — personal/internal non-commercial evaluation only. No sale, commercial exploitation or redistribution without prior written permission. See `LICENSE.txt`. Third-party dependencies retain their original licenses.