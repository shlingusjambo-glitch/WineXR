# WineXR — Windows OpenXR runtime for Wine

`vr4mac_openxr.c` is a minimal OpenXR 1.0 runtime (x64 Windows DLL) that Steam
games load inside Wine. OpenVR (SteamVR) games reach it through OpenComposite.
It publishes rendered frames plus poses to the Mac app over shared memory
(`common/vr4mac.h`); the Mac app streams them to the headset.

This directory is published as its own repository ("WineXR"). The protocol
header `common/vr4mac.h` is canonical here; downstream repos vendor a copy.

## Build (needs mingw-w64)

```sh
./build.sh        # -> build/vr4mac_openxr.dll + build/test_xr.exe smoke test
```

## Install

The MacVR app copies the bundled DLL to `C:\VR4Mac\` in the Wine bottle and
registers it as the OpenXR `ActiveRuntime` on every launch (`Games.setup()`),
so a matching pair is always deployed together. Manual equivalent: copy
`build/vr4mac_openxr.dll` to `C:\VR4Mac\` and point the registry
`ActiveRuntime` at `C:\VR4Mac\vr4mac_openxr.json` (see `Games.swift`).

## Compatibility extensions

Beyond core OpenXR 1.0 the runtime answers: `XR_FB_display_refresh_rate`
(enumerate/get/request, fed from shm fps), `XR_FB_color_space` (enumerate/set,
supporting Quest, Rec.709, and unmanaged), quad layers (letterboxed into both
eyes when no stereo projection is submitted), MSAA swapchains (accepted,
rendered unaliased), duplicate-displayTime submits (skipped, heartbeat kept),
`XR_KHR_win32_convert_performance_counter_time`,
`XR_KHR_composition_layer_depth`, `_cylinder`, and `_color_scale_bias` (accepted),
`XR_EXT_local_floor` (maps to stage with floor offset, fully enumerated in
`xrEnumerateReferenceSpaces`), and `XR_KHR_visibility_mask` (empty mask); unknown reference
spaces are logged, not failed. These unstick real games (BONELAB waits on the
refresh-rate feature; Beat Saber needs first-run save files seeded — that part
lives in the Mac app's per-game setup).

## Shared-memory / wire contract

Single source of truth: [`common/vr4mac.h`](../common/vr4mac.h). Summary:

- File `/tmp/vr4mac/shm` (Wine: `Z:\tmp\vr4mac\shm`), `VR4_SHM_MAGIC`, `VR4_SHM_VERSION = 3`.
- Mac → runtime: `eye_w/eye_h`, `fps`, `client_connected`, seqlocked `VR4Tracking` (head, eyes, hands), `input_blocked` (dashboard open), per-app `render_scale` / `world_scale` (0 = 1.0; runtime rounds scaled views to 32 px and divides stage positions by world_scale).
- Runtime → Mac: heartbeats, haptics, `app_name`, double-buffered side-by-side frames (`frame_seq % 2`, `frame_eye_pose` echoed back for timewarp). `frame_rgba[i] = 1` means the buffer holds RGBA; the Mac converts to BGRA with vImage so no swizzle runs on the game thread.
- Wire (Mac ↔ Quest, TCP 9945, discovery UDP 9944): `HELLO` (`codecs: [hevc,h264]`, `audio`, `hevc_max_eye_w/h`) → `CONFIG` (`codec=hevc/h264`) → `TRACKING` (284-byte `VR4Tracking`) / `VIDEO` (Annex-B VPS/SPS/PPS + IDR, `VR4VideoHeader`) / `AUDIO` (48 kHz stereo s16le, 10 ms) / `HAPTICS` / `REQUEST_IDR`.

## Tests

`test_xr.c` / `test_openvr_loader.c` smoke-test the loader path; see `test_xr*` and `test_openvr_loader.sh`.
