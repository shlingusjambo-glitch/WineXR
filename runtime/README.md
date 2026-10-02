# WineXR — Windows OpenXR runtime for Wine

`vr4mac_openxr.c` is a compact OpenXR 1.0 runtime (x64 Windows DLL) that Steam
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

## Graphics

- `XR_KHR_D3D11_enable` — the full path: projection plus quad and cylinder
  layers, composited on the game's GPU.
- `XR_KHR_D3D12_enable` — projection layers. Swapchain images are committed
  resources handed out in `RENDER_TARGET` / `DEPTH_WRITE`; the runtime copies
  each frame with its own command list on the game's queue (fenced ring, never
  blocks). Quad/cylinder layers are logged and skipped.
- Vulkan (`XR_KHR_vulkan_enable*`) is not implemented; such games log the
  graphics binding they wanted.

Swapchain formats: RGBA8/BGRA8 (sRGB or UNORM; bytes are streamed as sRGB
either way) and D32/D24S8/D16/D32S8 depth for `XR_KHR_composition_layer_depth`
(accepted, not used). MSAA requests are accepted and rendered unaliased.

## Frames and layers

- The first stereo projection is copied side by side into one readback
  texture (one map + one contiguous copy per frame); the copy is flushed right
  away and published from a 4-deep ring as soon as the GPU is done, never
  waiting on it.
- Quad and cylinder layers after the projection are drawn on top in submit
  order with real 3D placement (reference/action spaces, eye visibility,
  premultiplied or straight alpha, `XR_KHR_composition_layer_color_scale_bias`).
  Layers before an opaque projection are hidden by it. Frames with only layers
  (menus, players, loading screens) show them in place on black. The
  compositor saves and restores every piece of D3D11 state it touches.
- Echoed `frame_eye_pose` values are stage space, whatever space the game
  rendered in. Duplicate-displayTime submits are skipped (heartbeat kept).
- Equirect, cube and passthrough layers are logged once and not shown.

## Input

Bindings resolve once, at `xrAttachSessionActionSets`, to a control on the
Quest's Touch controllers (or tracked hands). The controller profile is the
best suggested one in this order: Oculus Touch, Meta Touch Plus, Touch Pro,
Valve Index, HP Reverb G2, Vive Cosmos, Windows Mixed Reality, HTC Vive, KHR
simple; unknown profiles come last and are read like Touch. Remaps:

- `a`/`b` on the left hand are X/Y; `x`/`y` on the right hand are A/B.
- A Vive trackpad is the thumbstick (click = stick click or A/X); a trackpad
  next to a real stick (Index, WMR) stays idle, except WMR's pad click = A/X.
- `menu` on the right hand of a controller without face buttons (Vive,
  simple, WMR) is B; `system` on the left hand is the menu button.
- Touch Plus/Pro/proximity extras: trigger curl and proximity = trigger
  touch, thumb proximity = thumb on the rest or stick, force = trigger value.
- Re-suggesting a profile replaces its earlier bindings (spec); bindings no
  Quest control can feed are logged once and left unbound.
- `xrGetInputSourceLocalizedName` names the physical Quest control ("Left
  Hand Touch Controller X Button" for an Index left `a`).

## Hand tracking

Shared-memory v4 carries the Quest's 26 joints per hand under the tracking
seqlock (`hand_joints`, zeroed while that hand holds a controller).

- `XR_EXT_hand_tracking`: joint poses in any base space, typical joint radii,
  velocities from consecutive samples (`XrHandJointVelocitiesEXT`).
  `XR_EXT_hand_joints_motion_range` is accepted (data is always unobstructed).
- `XR_EXT_hand_tracking_data_source`: reports `UNOBSTRUCTED`; a tracker that
  only asked for controller data never activates.
- `XR_FB_hand_tracking_aim`: the Mac's shoulder-to-pinch aim ray, pinch
  strengths for all four fingers (thumb-tip distance), index pinching = the
  Mac's pinch, palm toward the face = system gesture, left-palm pinch = menu.
- `XR_EXT_hand_interaction`: when a game suggests the hand-interaction profile,
  a hand that puts its controller down switches to it (with an interaction
  profile changed event) and back when picked up: pinch/aim-activate value and
  ready, grasp from curled fingers, pinch pose between thumb and index tips,
  poke pose at the index tip.
- Controller-only games keep working with tracked hands: the Mac synthesizes
  `track.hand` (pinch = trigger, aim ray, palm grip).

## Other extensions

`XR_FB_display_refresh_rate` (fed from shm fps), `XR_FB_color_space`,
`XR_KHR_win32_convert_performance_counter_time`, `XR_EXT_local_floor` (the
local origin on the floor), `XR_KHR_visibility_mask` (nothing hidden: empty
hidden mesh, full-view visible mesh and outline), `XR_EXT_performance_settings`
(hints, logged), `XR_KHR_locate_spaces`, `XR_EXT_debug_utils` (no-op). No
`XR_EXT_palm_pose`: a fake palm (= grip) shifted OpenComposite's hands.

These unstick real games (BONELAB waits on the refresh-rate feature; Beat
Saber needs first-run save files seeded — that part lives in the Mac app's
per-game setup).

## Logging

`Z:\tmp\vr4mac\runtime.log`: requested extensions, suggested profiles, the
attached profile, swapchains, the first frames' layers, pacing every 5 s.
Unsupported functions, layer types, extensions and unmapped bindings are
logged once each. xrWaitFrame sleeps while no tracking sample is due, so a
quiet headset doesn't cost a busy core.

## Shared-memory / wire contract

Single source of truth: [`common/vr4mac.h`](../common/vr4mac.h). Summary:

- File `/tmp/vr4mac/shm` (Wine: `Z:\tmp\vr4mac\shm`; `VR4_SHM=<windows path>` overrides it for tests), `VR4_SHM_MAGIC`, `VR4_SHM_VERSION = 4`.
- Mac → runtime: `eye_w/eye_h`, `fps`, `client_connected`, seqlocked `VR4Tracking` (head, eyes, hands) and `hand_joints`, `input_blocked` (dashboard open), per-app `render_scale` / `world_scale` (0 = 1.0; runtime rounds scaled views to 32 px and divides stage positions by world_scale).
- Runtime → Mac: heartbeats, haptics, `app_name`, double-buffered side-by-side frames (`frame_seq % 2`, `frame_eye_pose` echoed back in stage space for timewarp). `frame_rgba[i] = 1` means the buffer holds RGBA; the Mac converts to BGRA with vImage so no swizzle runs on the game thread.
- Wire (Mac ↔ Quest, TCP 9945, discovery UDP 9944): `HELLO` (`codecs: [hevc,h264]`, `audio`, `hevc_max_eye_w/h`) → `CONFIG` (`codec=hevc/h264`) → `TRACKING` (284-byte `VR4Tracking`, plus `VR4HandJoints[2]` while a hand is tracked) / `VIDEO` (Annex-B VPS/SPS/PPS + IDR, `VR4VideoHeader`) / `AUDIO` (48 kHz stereo s16le, 10 ms) / `HAPTICS` / `REQUEST_IDR`.

## Tests

`test_xr.exe` plays both sides: a D3D11/D3D12 game, and the Mac app writing
tracking and hand joints into a private shared-memory block in `%TEMP%` (it
never touches the live `/tmp/vr4mac/shm`). It checks projection pixels and
stage-space eye poses, duplicate-submit skipping, quad/cylinder composition
(over a projection, hidden under it, alone, per eye, colour scale/bias), depth
swapchains, Index/Vive/hand-interaction bindings and profile switching, joint
poses/velocities/aim state, the visibility mask, `xrLocateSpaces`, that an
idle xrWaitFrame doesn't spin, and a D3D12 frame. Run it in a throwaway
prefix (exit code 0 = pass; the runtime log lands next to the shm file). It
passes on Game Porting Toolkit's Wine (D3DMetal); the Sikarugir engine can't
spawn processes outside its launcher, so it can't build a fresh prefix.

```sh
WINEPREFIX=/tmp/winexr-test wine64 build/test_xr.exe   # with vr4mac_openxr.dll next to it
```

`test_openvr_loader.c` probes a real game's OpenVR entry DLL; see `test_openvr_loader.sh`.
