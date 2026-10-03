# Third-party notices

MacVR itself is MIT licensed (see `LICENSE`). The components below are owned
by their respective authors and governed by their own licenses. Anything marked
**downloaded at setup** is fetched to your Mac on first run and is never
bundled in this repository or in the release archives.

## Bundled with this repository

| Component | Source | License |
|---|---|---|
| OpenXR headers (`SiliconXR/include/openxr/`) | [KhronosGroup/OpenXR-SDK](https://github.com/KhronosGroup/OpenXR-SDK) 1.1.63 | Apache-2.0 OR MIT |
| OpenVR C API header (`SiliconXR/openvr_capi.h`) | [ValveSoftware/openvr](https://github.com/ValveSoftware/openvr) v1.23.7 | BSD-3-Clause (Copyright (c) 2015, Valve Corporation) |
| Quest controller meshes (`mac/Resources/controllers/*.glb`) | [webxr-input-profiles assets](https://github.com/immersive-web/webxr-input-profiles) 1.0.20, see `mac/Resources/controllers/SOURCE.txt` | W3C Software and Document License (full text in `mac/Resources/controllers/LICENSE`) |
| UI sounds (`mac/Resources/sounds/*.wav`) | Android Open Source Project (`frameworks/base/data/sounds/effects`) | Apache License 2.0 (see `mac/Resources/sounds/LICENSE-AOSP.txt`); synthesized PCM in `UISounds.swift` is the fallback |
| Welcome-tour music (`mac/Resources/sounds/oobe-music.mp3`) | Supplied by the project author | Free-use track, redistributed with the author's permission |
| Home panoramas (`mac/Resources/environments/*.jpg`) | [Poly Haven](https://polyhaven.com) | CC0 1.0 (see `mac/Resources/environments/LICENSE.txt`) |
| UI icons (`mac/Resources/icons/*.svg`) | [Lucide Icons](https://lucide.dev) | ISC License (full text in `mac/Resources/icons/LICENSE.txt`) |
| Khronos OpenXR loader (Android client, pinned Maven dependency) | [KhronosGroup/OpenXR-SDK](https://github.com/KhronosGroup/OpenXR-SDK) | Apache License 2.0 |

## Downloaded automatically on first run (not bundled, not redistributed)

| Component | Source | License |
|---|---|---|
| Sikarugir Template + `WS12WineSikarugir11.0` engine | [Sikarugir-App/Template](https://github.com/Sikarugir-App/Template) and [Sikarugir-App/Engines](https://github.com/Sikarugir-App/Engines) | Sikarugir/app terms; the Wine engine is upstream Wine, LGPL-2.1-or-later (sources: [Sikarugir-App/wine](https://github.com/Sikarugir-App/wine)) |
| OpenComposite (`vrclient_x64.dll`, OpenVR → OpenXR) | [upstream](https://gitlab.com/znixian/OpenOVR) via `https://znix.xyz/OpenComposite/download.php?arch=x64&branch=openxr` | GPL-3.0-only (upstream). Downloaded into your Wine bottle only; never linked into or shipped with MacVR |
| Steam client installer | [cdn.cloudflare.steamstatic.com/client/installer/SteamSetup.exe](https://cdn.cloudflare.steamstatic.com/client/installer/SteamSetup.exe) | Valve Steam Subscriber Agreement |
| Gradle + Android SDK components (APK build only) | Google / Gradle | Their respective licenses |

## Hand mesh

mac/Resources/hands/hand.obj: the "Splayed" mesh from "hand topology technical demonstration" (handref2.fbx),
supplied by the project owner; converted to metres and trimmed past the wrist. Rigged and posed at runtime by
mac/Sources/Hands.swift.

## 3D home models

Room 1107 and Kleeblatt were supplied by the project owner as model archives.
Credit: [fangzhangmnm](https://sketchfab.com/fangzhangmnm). These assets remain
their creator’s work and are not covered by MacVR’s MIT license.

## System frameworks (build/run only)

Xcode command-line tools (Swift, VideoToolbox, ScreenCaptureKit, SceneKit),
Rosetta 2 (for the Steam game path), and the Android SDK/NDK remain property
of their vendors and are never redistributed.
