# ALIGNCAM V4.7 — Long-range dual-phone optical tracker

English-language mobile web application hosted at:
https://rafal123456789987564.github.io/M5-launcher-EDAP-/aligncam/v4/

## What changed in V4.7

- New matching A4 sheets: [print REF/LEFT/RIGHT long-range targets](targets-long-range.html). Three landscape A4 pages at **100% scale**; no printed labels. The printed order is REF, LEFT, RIGHT.
- Each new sheet has **two large 130 mm ArUco 4x4 markers and four auxiliary 26 mm markers**, with unique IDs for each wheel/reference.
- An explicit target mode selector supports new long-range sheets, V4.3 12-marker sheets and the much older **CLASSIC** single large ArUco sheet (coarse tracking only).
- Camera requests 1920×1080 capture where supported; tracking works at a CPU-friendly 1280-pixel analysis width, including a padded ROI tracker with occasional full-frame reacquisition.
- Based on the permissively licensed js-aruco2 library: multi-window adaptive threshold recovery if a target cannot be recognized.
- Optical zoom through MediaStreamTrack.getCapabilities() / applyConstraints(), when the phone/browser actually implements zoom. AUTO zoom tries to enlarge insufficiently resolved targets before measurement; manual +/− buttons available. Continuous autofocus is requested when supported; otherwise the browser controls focus.
- Hampel/MAD + Kalman + One-Euro smoothing and marker consensus checks inherited from V4.2.
- Red target border is reserved for **WRONG TARGET**. Detection search and coarse/unstable measurements show warnings, not false positive "wrong target" alarms.
- REF and camera calibration are invalidated on optical zoom change. Calibration from old frame geometry must not be silently reused.

## How to use

1. Print [new A4 long-range sheets](targets-long-range.html), landscape, **100% actual size**, on rigid flat boards.
2. Open the application directly in Chrome on both phones using the same HTTPS URL.
3. Use target mode **LONG RANGE V4.7**.
4. Point the phone at a board; before pairing, either LEFT, RIGHT or REF can be shown for a quick recognition test.
5. Use **Create room** on the left phone, then enter its six-digit PIN on the right phone and press **Join**.
6. Place phones on stable mounts. Finish phone levelling. Show REF board, set **Reference ZERO** as guided. Do not move the cameras after REF capture.
7. Place LEFT and RIGHT boards flat on their respective wheel fixtures. Record and verify wheel-runout compensation separately.
8. A displayed angle is an *uncertified relative orientation estimate*, not guaranteed factory toe or camber. The app may show **COARSE — OPTICAL ZOOM** when image sampling is not sufficient.

## Test evidence and limitations

- Browser-side js-aruco2 was tested on synthetic images at 1 m, 2 m and 3 m using 1280×720 resolution and a modelled 780-pixel camera focal length. Both major markers decoded at each distance.
- The end-to-end numerical pipeline was tested on the same simulated markers (5, 2 and 2 inliers respectively); passing a synthetic detection test is **not a real-world calibration**.
- At 3 m the target can be decoded while angle estimation is *several degrees wrong* because of low image resolution, camera intrinsics and planar pose ambiguity. Higher optical magnification, a calibrated camera, rigid mounting and physical reference validation are necessary; **no filter can restore image information that the lens does not capture**.
- Browser zoom/focus APIs are optional and device-dependent. There is no guaranteed optical zoom through Chrome for every Android model.
- The camera is monocular at each wheel, so additional geometry/runout/reference calibration is required before using readings for safety-critical wheel-alignment work.

## Technical references (not copied source code)

- OpenCV ArUco detection and detector tuning: https://docs.opencv.org/5.0/tutorials/objdetect/aruco_detection/aruco_detection.html
- js-aruco2 (BSD/MIT): https://github.com/damianofalcioni/js-aruco2
- Open-source wheel alignment research and optical calibration concepts: https://github.com/AristarhUcolov/wheel-alignment . This reference project is AGPL-3.0. **No AGPL-licensed implementation was incorporated into ALIGNCAM's proprietary source.**
- MDN capture capabilities and track constraints: https://developer.mozilla.org/en-US/docs/Web/API/MediaStreamTrack/getCapabilities

## Licence

The original AlignCam files are provided only under [LICENSE.txt](LICENSE.txt) (personal/internal non-commercial evaluation, no commercial use or sale without written permission). Third-party library rights are preserved. No certified vehicle alignment accuracy is claimed.
