// Smoke test: behaves like a D3D11 OpenXR game against vr4mac_openxr.dll while playing the Mac app's part in a private
// shared-memory block (VR4_SHM; the live /tmp/vr4mac/shm is never touched). Covers the frame path (projection, quad and
// cylinder layers, stage-space eye poses), hand tracking, interaction-profile remaps and the compatibility extensions.
// Run under Wine: wine test_xr.exe  (exit code 0 = pass)
#define COBJMACROS
#include <windows.h>
#include <d3d11.h>
#define WIDL_C_INLINE_WRAPPERS
#include <d3d12.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define XR_NO_PROTOTYPES
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#define XR_USE_GRAPHICS_API_D3D12
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include "../common/vr4mac.h"

#define CHECK(x) do { XrResult r_ = (x); if (XR_FAILED(r_)) { printf("FAIL %s = %d (line %d)\n", #x, r_, __LINE__); exit(1); } } while (0)
#define EXPECT(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); exit(1); } } while (0)
#define CLOSE(a, b) (fabsf((a) - (b)) < 1e-3f)
static PFN_xrGetInstanceProcAddr gipa;
static XrInstance inst;
#define FNS(_) _(xrCreateInstance) _(xrDestroyInstance) _(xrGetSystem) _(xrGetSystemProperties) _(xrEnumerateViewConfigurationViews) \
    _(xrGetD3D11GraphicsRequirementsKHR) _(xrCreateSession) _(xrDestroySession) _(xrBeginSession) _(xrCreateSwapchain) _(xrDestroySwapchain) \
    _(xrEnumerateSwapchainImages) _(xrAcquireSwapchainImage) _(xrWaitSwapchainImage) _(xrReleaseSwapchainImage) _(xrWaitFrame) \
    _(xrBeginFrame) _(xrEndFrame) _(xrLocateViews) _(xrCreateReferenceSpace) _(xrPollEvent) _(xrStringToPath) _(xrCreateActionSet) \
    _(xrCreateAction) _(xrSuggestInteractionProfileBindings) _(xrAttachSessionActionSets) _(xrSyncActions) _(xrGetActionStateBoolean) \
    _(xrGetActionStateFloat) _(xrGetActionStateVector2f) _(xrGetActionStatePose) _(xrCreateActionSpace) _(xrLocateSpace) _(xrLocateSpaces) \
    _(xrGetCurrentInteractionProfile) _(xrEnumerateBoundSourcesForAction) _(xrGetInputSourceLocalizedName) _(xrCreateHandTrackerEXT) \
    _(xrLocateHandJointsEXT) _(xrGetVisibilityMaskKHR) _(xrPerfSettingsSetPerformanceLevelEXT) _(xrEnumerateSwapchainFormats) _(xrGetD3D12GraphicsRequirementsKHR)
#define DECL(n) static PFN_##n n;
#define LOAD(n) CHECK(gipa(inst, #n, (PFN_xrVoidFunction *)&n));
FNS(DECL)

static VR4Shm *shm;
static XrSession ses; static XrSystemId sys; static XrSpace stage, view, local;
static ID3D11Device *dev; static ID3D11DeviceContext *ctx;
static uint32_t w, h;
static const float RED[4] = {1, 0, 0, 1}, GREEN[4] = {0, 1, 0, 1};
#define PX_RED 0xff0000ffu
#define PX_GREEN 0xff00ff00u
#define PX_BLACK 0xff000000u

static void put_track(const VR4Tracking *t, const VR4HandJoints *j) {   // the Mac app's seqlocked write
    shm->track_seq++; __atomic_thread_fence(__ATOMIC_SEQ_CST);
    shm->track = *t;
    if (j) memcpy(shm->hand_joints, j, sizeof shm->hand_joints); else memset(shm->hand_joints, 0, sizeof shm->hand_joints);
    __atomic_thread_fence(__ATOMIC_SEQ_CST); shm->track_seq++;
}
static VR4Pose pose(float x, float y, float z) { VR4Pose p = {x, y, z, 0, 0, 0, 1}; return p; }
static VR4Tracking head_at(float yaw, uint64_t t) {   // head at (0.3, 1.6, -0.2) turned by yaw, eyes 64 mm apart, 0.8 rad half-FOV
    VR4Tracking k; memset(&k, 0, sizeof k);
    float s = sinf(yaw / 2), c = cosf(yaw / 2);
    k.time_ns = t; k.head = pose(0.3f, 1.6f, -0.2f); k.head.qy = s; k.head.qw = c;
    for (int e = 0; e < 2; e++) {
        float off = e ? 0.032f : -0.032f;
        k.eye[e].pose = pose(0.3f + off * cosf(yaw), 1.6f, -0.2f - off * sinf(yaw)); k.eye[e].pose.qy = s; k.eye[e].pose.qw = c;
        VR4Fov f = {-0.8f, 0.8f, 0.8f, -0.8f}; k.eye[e].fov = f;
    }
    return k;
}
static XrPath path(const char *s) { XrPath p; CHECK(xrStringToPath(inst, s, &p)); return p; }

typedef struct { XrSwapchain sc; ID3D11Texture2D *img[3]; uint32_t w, h; } Chain;
static Chain chain(uint32_t cw, uint32_t ch) {
    Chain c = {0}; uint32_t n; c.w = cw; c.h = ch;
    XrSwapchainCreateInfo ci = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 1, cw, ch, 1, 1, 1};
    CHECK(xrCreateSwapchain(ses, &ci, &c.sc));
    XrSwapchainImageD3D11KHR imgs[3] = {{XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}};
    CHECK(xrEnumerateSwapchainImages(c.sc, 3, &n, (XrSwapchainImageBaseHeader *)imgs));
    for (uint32_t i = 0; i < n; i++) c.img[i] = imgs[i].texture;
    return c;
}
static void paint(Chain *c, const float rgba[4]) {   // acquire, clear, release
    uint32_t idx; CHECK(xrAcquireSwapchainImage(c->sc, NULL, &idx));
    XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, NULL, XR_INFINITE_DURATION}; CHECK(xrWaitSwapchainImage(c->sc, &wi));
    D3D11_RENDER_TARGET_VIEW_DESC rd = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, D3D11_RTV_DIMENSION_TEXTURE2D};
    ID3D11RenderTargetView *rtv;
    EXPECT(SUCCEEDED(ID3D11Device_CreateRenderTargetView(dev, (ID3D11Resource *)c->img[idx], &rd, &rtv)), "RTV");
    ID3D11DeviceContext_ClearRenderTargetView(ctx, rtv, rgba);
    ID3D11RenderTargetView_Release(rtv);
    CHECK(xrReleaseSwapchainImage(c->sc, NULL));
}
static XrTime frame(const XrCompositionLayerBaseHeader *const *layers, uint32_t n, XrTime displayTime) {   // 0 = use the predicted time
    XrFrameState fs = {XR_TYPE_FRAME_STATE};
    CHECK(xrWaitFrame(ses, NULL, &fs)); CHECK(xrBeginFrame(ses, NULL));
    XrFrameEndInfo fe = {XR_TYPE_FRAME_END_INFO, NULL, displayTime ? displayTime : fs.predictedDisplayTime, XR_ENVIRONMENT_BLEND_MODE_OPAQUE, n, layers};
    CHECK(xrEndFrame(ses, &fe));
    return fe.displayTime;
}
static XrCompositionLayerProjectionView pviews[2], pviewsArr[2], pviews12[2];
static XrCompositionLayerProjection projection_into(Chain *c, XrSpace space, XrCompositionLayerProjectionView out[2]) {   // both eyes side by side in one swapchain
    XrView v[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}}; uint32_t n;
    XrViewState vs = {XR_TYPE_VIEW_STATE};
    XrViewLocateInfo li = {XR_TYPE_VIEW_LOCATE_INFO, NULL, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, space};
    CHECK(xrLocateViews(ses, &li, &vs, 2, &n, v));
    for (int e = 0; e < 2; e++) {
        XrCompositionLayerProjectionView pv = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW, NULL, v[e].pose, v[e].fov, {c->sc, {{(int32_t)(e * w), 0}, {(int32_t)w, (int32_t)h}}, 0}};
        out[e] = pv;
    }
    XrCompositionLayerProjection p = {XR_TYPE_COMPOSITION_LAYER_PROJECTION, NULL, 0, space, 2, out};
    return p;
}
static XrCompositionLayerProjection projection(Chain *c, XrSpace space) { return projection_into(c, space, pviews); }
static const uint32_t *pixels(void) { return (const uint32_t *)vr4_frame(shm, shm->frame_seq); }
typedef struct { uint32_t x, y, v; } Probe;   // pixel of the side-by-side frame
/// Submits frames until a published frame matches every probe (readback is asynchronous, up to 4 frames behind).
static void expect_frame(const char *what, const XrCompositionLayerBaseHeader *const *layers, uint32_t n, const Probe *p, int np) {
    for (int attempt = 0; attempt < 12; attempt++) {
        uint32_t seq = shm->frame_seq;
        frame(layers, n, 0);
        if (shm->frame_seq == seq) continue;
        const uint32_t *px = pixels(); int ok = shm->frame_w[shm->frame_seq % 2] == 2 * w;
        for (int i = 0; i < np; i++) ok &= px[p[i].y * 2 * w + p[i].x] == p[i].v;
        if (ok) { printf("%s ok\n", what); return; }
    }
    const uint32_t *px = pixels();
    printf("FAIL %s:", what);
    for (int i = 0; i < np; i++) printf(" (%u,%u)=%08x want %08x", p[i].x, p[i].y, px[p[i].y * 2 * w + p[i].x], p[i].v);
    printf("\n"); exit(1);
}

static XrActionSet set;
static XrAction action(const char *name, XrActionType type) {
    XrPath subs[2] = {path("/user/hand/left"), path("/user/hand/right")};
    XrActionCreateInfo ci = {XR_TYPE_ACTION_CREATE_INFO, NULL, "", type, 2, subs, ""};
    strcpy(ci.actionName, name); strcpy(ci.localizedActionName, name);
    XrAction a; CHECK(xrCreateAction(set, &ci, &a)); return a;
}
static void suggest(const char *profile, const XrActionSuggestedBinding *b, uint32_t n) {
    XrInteractionProfileSuggestedBinding sb = {XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING, NULL, path(profile), n, b};
    CHECK(xrSuggestInteractionProfileBindings(inst, &sb));
}
static float get_float(XrAction a, const char *sub, XrBool32 *active) {
    XrActionStateGetInfo gi = {XR_TYPE_ACTION_STATE_GET_INFO, NULL, a, path(sub)}; XrActionStateFloat st = {XR_TYPE_ACTION_STATE_FLOAT};
    CHECK(xrGetActionStateFloat(ses, &gi, &st)); if (active) *active = st.isActive; return st.currentState;
}
static int get_bool(XrAction a, const char *sub, XrBool32 *active) {
    XrActionStateGetInfo gi = {XR_TYPE_ACTION_STATE_GET_INFO, NULL, a, path(sub)}; XrActionStateBoolean st = {XR_TYPE_ACTION_STATE_BOOLEAN};
    CHECK(xrGetActionStateBoolean(ses, &gi, &st)); if (active) *active = st.isActive; return st.currentState;
}
static XrVector2f get_vec2(XrAction a, const char *sub) {
    XrActionStateGetInfo gi = {XR_TYPE_ACTION_STATE_GET_INFO, NULL, a, path(sub)}; XrActionStateVector2f st = {XR_TYPE_ACTION_STATE_VECTOR2F};
    CHECK(xrGetActionStateVector2f(ses, &gi, &st)); return st.currentState;
}
static XrPath profile_of(const char *hand) {
    XrInteractionProfileState st = {XR_TYPE_INTERACTION_PROFILE_STATE};
    CHECK(xrGetCurrentInteractionProfile(ses, path(hand), &st)); return st.interactionProfile;
}
static int drain_events(XrStructureType type) {   // how many events of `type` were queued
    XrEventDataBuffer ev = {XR_TYPE_EVENT_DATA_BUFFER}; int n = 0;
    while (xrPollEvent(inst, &ev) == XR_SUCCESS) { n += ev.type == type; ev.type = XR_TYPE_EVENT_DATA_BUFFER; }
    return n;
}
static void sync(void) {
    XrActiveActionSet as = {set, XR_NULL_PATH}; XrActionsSyncInfo si = {XR_TYPE_ACTIONS_SYNC_INFO, NULL, 1, &as};
    CHECK(xrSyncActions(ses, &si));
}

static void start(const char *name, const void *binding) {   // instance + session on the given D3D11 or D3D12 device
    XrNegotiateLoaderInfo li = {XR_LOADER_INTERFACE_STRUCT_LOADER_INFO, XR_LOADER_INFO_STRUCT_VERSION, sizeof li, 1, 1, XR_MAKE_VERSION(1, 0, 0), XR_MAKE_VERSION(1, 1, 0)};
    XrNegotiateRuntimeRequest rr = {XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST, XR_RUNTIME_INFO_STRUCT_VERSION, sizeof rr};
    PFN_xrNegotiateLoaderRuntimeInterface neg = (PFN_xrNegotiateLoaderRuntimeInterface)GetProcAddress(GetModuleHandleA("vr4mac_openxr.dll"), "xrNegotiateLoaderRuntimeInterface");
    CHECK(neg(&li, &rr));
    gipa = rr.getInstanceProcAddr;
    CHECK(gipa(XR_NULL_HANDLE, "xrCreateInstance", (PFN_xrVoidFunction *)&xrCreateInstance));
    const char *ext[] = {XR_KHR_D3D11_ENABLE_EXTENSION_NAME, XR_EXT_HAND_TRACKING_EXTENSION_NAME, XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME,
                         XR_EXT_HAND_INTERACTION_EXTENSION_NAME, XR_EXT_HAND_TRACKING_DATA_SOURCE_EXTENSION_NAME, XR_KHR_VISIBILITY_MASK_EXTENSION_NAME,
                         XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME, XR_KHR_COMPOSITION_LAYER_CYLINDER_EXTENSION_NAME, XR_KHR_LOCATE_SPACES_EXTENSION_NAME,
                         XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME, XR_KHR_D3D12_ENABLE_EXTENSION_NAME};
    XrInstanceCreateInfo ci = {XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(ci.applicationInfo.applicationName, name); ci.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ci.enabledExtensionCount = sizeof ext / sizeof *ext; ci.enabledExtensionNames = ext;
    CHECK(xrCreateInstance(&ci, &inst));
    FNS(LOAD)
    XrSystemGetInfo sg = {XR_TYPE_SYSTEM_GET_INFO, NULL, XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    CHECK(xrGetSystem(inst, &sg, &sys));
    XrViewConfigurationView vcv[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}}; uint32_t n;
    CHECK(xrEnumerateViewConfigurationViews(inst, sys, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &n, vcv));
    w = vcv[0].recommendedImageRectWidth; h = vcv[0].recommendedImageRectHeight;
    XrGraphicsRequirementsD3D11KHR req = {XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    CHECK(xrGetD3D11GraphicsRequirementsKHR(inst, sys, &req));
    XrSessionCreateInfo sci = {XR_TYPE_SESSION_CREATE_INFO, binding, 0, sys};
    CHECK(xrCreateSession(inst, &sci, &ses));
    drain_events(0);
    XrSessionBeginInfo bi = {XR_TYPE_SESSION_BEGIN_INFO, NULL, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
    CHECK(xrBeginSession(ses, &bi));
    XrReferenceSpaceCreateInfo rs = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO, NULL, XR_REFERENCE_SPACE_TYPE_STAGE, {{0, 0, 0, 1}, {0, 0, 0}}};
    CHECK(xrCreateReferenceSpace(ses, &rs, &stage));
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW; CHECK(xrCreateReferenceSpace(ses, &rs, &view));
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL; CHECK(xrCreateReferenceSpace(ses, &rs, &local));
    XrActionSetCreateInfo as = {XR_TYPE_ACTION_SET_CREATE_INFO, NULL, "game", "Game", 0};
    CHECK(xrCreateActionSet(inst, &as, &set));
}

int main(int argc, char **argv) {
    // Sikarugir engine wine disconnects console stdout: VR4_TEST_LOG=C:\path\file redirects it.
    const char *logf = getenv("VR4_TEST_LOG");
    if (logf && !freopen(logf, "w", stdout)) fprintf(stderr, "warning: log redirect %s failed\n", logf);
    int frames = argc > 1 ? atoi(argv[1]) : 10;
    if (frames < 1) frames = 10;   // garbage argv must not trivially pass the published-count check below
    char shmPath[MAX_PATH]; DWORD tplen = GetTempPathA(MAX_PATH, shmPath);
    EXPECT(tplen > 0 && tplen + strlen("vr4test_shm") < MAX_PATH, "temp path len %lu", (unsigned long)tplen);
    strcat(shmPath, "vr4test_shm");
    HANDLE f = CreateFileA(shmPath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, 0, NULL);
    shm = MapViewOfFile(CreateFileMappingA(f, NULL, PAGE_READWRITE, 0, VR4_SHM_SIZE, NULL), FILE_MAP_ALL_ACCESS, 0, 0, VR4_SHM_SIZE);
    EXPECT(shm, "shm %s", shmPath);
    shm->magic = VR4_SHM_MAGIC; shm->version = VR4_SHM_VERSION; shm->eye_w = 256; shm->eye_h = 256; shm->fps = 90;
    VR4Tracking trk = head_at(0.5f, 1000000);
    put_track(&trk, NULL);
    SetEnvironmentVariableA("VR4_SHM", shmPath);
    EXPECT(LoadLibraryA("vr4mac_openxr.dll"), "load dll");
    EXPECT(SUCCEEDED(D3D11CreateDevice(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, NULL, 0, D3D11_SDK_VERSION, &dev, NULL, &ctx)), "D3D11CreateDevice");
    XrGraphicsBindingD3D11KHR gb11 = {XR_TYPE_GRAPHICS_BINDING_D3D11_KHR, NULL, dev};
    start("VR4Mac smoke test", &gb11);

    // ---- actions: an Index + Vive game that also suggests the hand-interaction profile (Touch is never suggested)
    XrAction jump = action("jump", XR_ACTION_TYPE_BOOLEAN_INPUT), trig = action("trig", XR_ACTION_TYPE_FLOAT_INPUT);
    XrAction move = action("move", XR_ACTION_TYPE_VECTOR2F_INPUT), grab = action("grab", XR_ACTION_TYPE_FLOAT_INPUT);
    XrAction aimA = action("aim", XR_ACTION_TYPE_POSE_INPUT), poke = action("poke", XR_ACTION_TYPE_POSE_INPUT);
    XrActionSuggestedBinding index[] = {{jump, path("/user/hand/left/input/a/click")}, {jump, path("/user/hand/right/input/a/click")},
        {trig, path("/user/hand/right/input/trigger/value")}, {move, path("/user/hand/left/input/thumbstick")}, {move, path("/user/hand/right/input/trackpad")},
        {grab, path("/user/hand/right/input/squeeze/value")}, {aimA, path("/user/hand/left/input/aim/pose")}, {aimA, path("/user/hand/right/input/aim/pose")}};
    XrActionSuggestedBinding vive[] = {{jump, path("/user/hand/left/input/trackpad/click")}};
    XrActionSuggestedBinding hands[] = {{trig, path("/user/hand/right/input/pinch_ext/value")}, {grab, path("/user/hand/right/input/grasp_ext/value")},
        {aimA, path("/user/hand/right/input/aim/pose")}, {poke, path("/user/hand/right/input/poke_ext/pose")}};
    suggest("/interaction_profiles/htc/vive_controller", vive, 1);
    suggest("/interaction_profiles/valve/index_controller", index, 1);   // replaced by the full list below
    suggest("/interaction_profiles/valve/index_controller", index, sizeof index / sizeof *index);
    suggest("/interaction_profiles/ext/hand_interaction_ext", hands, sizeof hands / sizeof *hands);
    XrSessionActionSetsAttachInfo at = {XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO, NULL, 1, &set};
    CHECK(xrAttachSessionActionSets(ses, &at));
    EXPECT(drain_events(XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) == 1, "profile event after attach");

    // ---- frames: projection (async readback), RGBA kept for the Mac's swizzle
    Chain eyes = chain(2 * w, h), quadc = chain(w, h);
    paint(&eyes, RED); paint(&quadc, GREEN);
    {   // MSAA swapchains are accepted (rendered unaliased) instead of rejected
        XrSwapchainCreateInfo ci = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 2, w, h, 1, 1, 1};
        XrSwapchain ms; CHECK(xrCreateSwapchain(ses, &ci, &ms)); CHECK(xrDestroySwapchain(ms));
    }
    XrCompositionLayerProjection proj = projection(&eyes, local);
    const XrCompositionLayerBaseHeader *projOnly[] = {(XrCompositionLayerBaseHeader *)&proj};
    uint32_t seq0 = shm->frame_seq; XrTime lastT = 0;
    for (int i = 0; i < frames; i++) lastT = frame(projOnly, 1, 0);
    uint32_t b = shm->frame_seq % 2;
    printf("eye %ux%u, published %u of %d, frame %ux%u, px %08x %08x\n", w, h, shm->frame_seq - seq0, frames, shm->frame_w[b], shm->frame_h[b], pixels()[0], pixels()[w + 5]);
    EXPECT(shm->frame_seq - seq0 + 4 >= (uint32_t)frames && pixels()[0] == PX_RED && pixels()[w + 5] == PX_RED && shm->frame_w[b] == 2 * w &&
           shm->frame_h[b] == h && shm->frame_rgba[b] == 1, "projection");
    // echoed eye poses are stage space even though the game rendered in LOCAL (origin turned by the start yaw)
    for (int e = 0; e < 2; e++) {
        VR4Pose p = shm->frame_eye_pose[b][e], want = trk.eye[e].pose;
        EXPECT(CLOSE(p.px, want.px) && CLOSE(p.py, want.py) && CLOSE(p.pz, want.pz) && CLOSE(fabsf(p.qy), fabsf(want.qy)) && CLOSE(fabsf(p.qw), fabsf(want.qw)),
               "eye %d pose %.3f %.3f %.3f q %.3f/%.3f, want %.3f %.3f %.3f q %.3f/%.3f", e, p.px, p.py, p.pz, p.qy, p.qw, want.px, want.py, want.pz, want.qy, want.qw);
    }
    printf("stage eye poses ok\n");
    {   // re-submitting the same displayTime publishes nothing (duplicate-submit skip)
        uint32_t s = shm->frame_seq; frame(projOnly, 1, lastT);
        EXPECT(shm->frame_seq == s, "dedup: seq %u -> %u", s, shm->frame_seq);
        printf("dedup ok\n");
    }

    // ---- layers: head-locked quad / cylinder over the projection, quad alone on black, per-eye visibility
    uint32_t cy = h / 2, cl = w / 2, cr = w + w / 2, side = w / 2 + (uint32_t)(0.4f * w / 2);   // centre of each eye; 0.4 rad... right of centre
    XrCompositionLayerQuad quad = {XR_TYPE_COMPOSITION_LAYER_QUAD, NULL, 0, view, XR_EYE_VISIBILITY_BOTH, {quadc.sc, {{0, 0}, {(int32_t)w, (int32_t)h}}, 0},
                                   {{0, 0, 0, 1}, {0, 0, -1}}, {0.5f, 0.5f}};
    const XrCompositionLayerBaseHeader *withQuad[] = {(XrCompositionLayerBaseHeader *)&proj, (XrCompositionLayerBaseHeader *)&quad};
    Probe pq[] = {{cl, cy, PX_GREEN}, {cr, cy, PX_GREEN}, {2, 2, PX_RED}, {side, cy, PX_RED}};
    expect_frame("quad over projection", withQuad, 2, pq, 4);
    const XrCompositionLayerBaseHeader *quadFirst[] = {(XrCompositionLayerBaseHeader *)&quad, (XrCompositionLayerBaseHeader *)&proj};
    Probe pu[] = {{cl, cy, PX_RED}, {cr, cy, PX_RED}};
    expect_frame("quad under opaque projection hidden", quadFirst, 2, pu, 2);
    XrCompositionLayerCylinderKHR cyl = {XR_TYPE_COMPOSITION_LAYER_CYLINDER_KHR, NULL, 0, view, XR_EYE_VISIBILITY_BOTH, {quadc.sc, {{0, 0}, {(int32_t)w, (int32_t)h}}, 0},
                                         {{0, 0, 0, 1}, {0, 0, 0}}, 1.0f, 1.0f, 2.0f};   // +-0.5 rad wide, 0.5 m tall at 1 m
    const XrCompositionLayerBaseHeader *withCyl[] = {(XrCompositionLayerBaseHeader *)&proj, (XrCompositionLayerBaseHeader *)&cyl};
    Probe pc[] = {{cl, cy, PX_GREEN}, {side, cy, PX_GREEN}, {2, 2, PX_RED}};
    expect_frame("cylinder over projection", withCyl, 2, pc, 3);
    quad.eyeVisibility = XR_EYE_VISIBILITY_LEFT;
    Probe pv[] = {{cl, cy, PX_GREEN}, {cr, cy, PX_RED}};
    expect_frame("left-eye-only quad", withQuad, 2, pv, 2);
    quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    const XrCompositionLayerBaseHeader *quadOnly[] = {(XrCompositionLayerBaseHeader *)&quad};
    Probe po[] = {{cl, cy, PX_GREEN}, {cr, cy, PX_GREEN}, {2, 2, PX_BLACK}};
    expect_frame("quad alone, placed in 3D", quadOnly, 1, po, 3);
    XrCompositionLayerColorScaleBiasKHR csb = {XR_TYPE_COMPOSITION_LAYER_COLOR_SCALE_BIAS_KHR, NULL, {0, 0, 0, 1}, {1, 0, 0, 0}};   // green -> red
    quad.next = &csb;
    Probe pb[] = {{cl, cy, PX_RED}, {2, 2, PX_BLACK}};
    expect_frame("colour scale/bias", quadOnly, 1, pb, 2);
    quad.next = NULL;
    {   // drawing layers leaves the game's D3D11 pipeline state as it was
        D3D11_VIEWPORT vp = {1, 2, 30, 40, 0, 1}, got[16]; UINT nvp = 16; D3D11_PRIMITIVE_TOPOLOGY topo;
        D3D11_RENDER_TARGET_VIEW_DESC rd = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, D3D11_RTV_DIMENSION_TEXTURE2D};
        ID3D11RenderTargetView *rtv, *grtv = NULL;
        EXPECT(SUCCEEDED(ID3D11Device_CreateRenderTargetView(dev, (ID3D11Resource *)quadc.img[0], &rd, &rtv)), "RTV");
        ID3D11DeviceContext_RSSetViewports(ctx, 1, &vp); ID3D11DeviceContext_OMSetRenderTargets(ctx, 1, &rtv, NULL);
        ID3D11DeviceContext_IASetPrimitiveTopology(ctx, D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
        frame(withQuad, 2, 0);
        ID3D11DeviceContext_RSGetViewports(ctx, &nvp, got); ID3D11DeviceContext_OMGetRenderTargets(ctx, 1, &grtv, NULL);
        ID3D11DeviceContext_IAGetPrimitiveTopology(ctx, &topo);
        EXPECT(grtv == rtv && got[0].Width == 30 && got[0].TopLeftY == 2 && topo == D3D11_PRIMITIVE_TOPOLOGY_LINELIST, "game state not restored");
        ID3D11RenderTargetView_Release(grtv); ID3D11DeviceContext_OMSetRenderTargets(ctx, 0, NULL, NULL); ID3D11RenderTargetView_Release(rtv);
        printf("game D3D11 state restored ok\n");
    }
    {   // one texture array for both eyes (Unity single-pass instanced): slice e is eye e
        XrSwapchainCreateInfo ci = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 1, w, h, 1, 2, 1};
        Chain arr = {0}; CHECK(xrCreateSwapchain(ses, &ci, &arr.sc));
        XrSwapchainImageD3D11KHR im[3] = {{XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}};
        uint32_t n, idx; CHECK(xrEnumerateSwapchainImages(arr.sc, 3, &n, (XrSwapchainImageBaseHeader *)im));
        CHECK(xrAcquireSwapchainImage(arr.sc, NULL, &idx));
        for (int e = 0; e < 2; e++) {
            D3D11_RENDER_TARGET_VIEW_DESC rd = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, D3D11_RTV_DIMENSION_TEXTURE2DARRAY};
            rd.Texture2DArray.FirstArraySlice = e; rd.Texture2DArray.ArraySize = 1;
            ID3D11RenderTargetView *rtv;
            EXPECT(SUCCEEDED(ID3D11Device_CreateRenderTargetView(dev, (ID3D11Resource *)im[idx].texture, &rd, &rtv)), "array RTV");
            ID3D11DeviceContext_ClearRenderTargetView(ctx, rtv, e ? GREEN : RED); ID3D11RenderTargetView_Release(rtv);
        }
        CHECK(xrReleaseSwapchainImage(arr.sc, NULL));
        XrCompositionLayerProjection pa = projection_into(&arr, stage, pviewsArr);
        for (int e = 0; e < 2; e++) { pviewsArr[e].subImage.imageRect.offset.x = 0; pviewsArr[e].subImage.imageArrayIndex = e; }
        const XrCompositionLayerBaseHeader *la[] = {(XrCompositionLayerBaseHeader *)&pa};
        Probe ps[] = {{cl, cy, PX_RED}, {cr, cy, PX_GREEN}};
        expect_frame("array swapchain (single-pass instanced)", la, 1, ps, 2);
        CHECK(xrDestroySwapchain(arr.sc));
    }

    // ---- depth swapchains (XR_KHR_composition_layer_depth) are real depth targets
    {
        int64_t fmts[16]; uint32_t nf; CHECK(xrEnumerateSwapchainFormats(ses, 16, &nf, fmts));
        int hasDepth = 0; for (uint32_t i = 0; i < nf; i++) hasDepth |= fmts[i] == DXGI_FORMAT_D32_FLOAT;
        XrSwapchainCreateInfo ci = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, DXGI_FORMAT_D32_FLOAT, 1, w, h, 1, 1, 1};
        XrSwapchain ds; CHECK(xrCreateSwapchain(ses, &ci, &ds));
        XrSwapchainImageD3D11KHR di[3] = {{XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}};
        uint32_t n; CHECK(xrEnumerateSwapchainImages(ds, 3, &n, (XrSwapchainImageBaseHeader *)di));
        D3D11_DEPTH_STENCIL_VIEW_DESC dd = {DXGI_FORMAT_D32_FLOAT, D3D11_DSV_DIMENSION_TEXTURE2D}; ID3D11DepthStencilView *dsv = NULL;
        EXPECT(hasDepth && SUCCEEDED(ID3D11Device_CreateDepthStencilView(dev, (ID3D11Resource *)di[0].texture, &dd, &dsv)), "depth swapchain");
        ID3D11DepthStencilView_Release(dsv); CHECK(xrDestroySwapchain(ds));
        printf("depth swapchain ok\n");
    }

    // ---- hand tracking: right hand tracked and pinching, left hand on its controller pressing X
    VR4HandJoints j[2]; memset(j, 0, sizeof j);
    j[1].tracked = 1;
    for (int i = 0; i < VR4_HAND_JOINTS; i++) j[1].joint[i] = pose(0.2f + 0.01f * i, 1.2f, -0.3f);
    j[1].joint[0] = pose(0.2f, 1.2f, -0.3f);
    j[1].joint[5] = pose(0.3f, 1.25f, -0.35f); j[1].joint[10] = pose(0.31f, 1.25f, -0.35f);   // thumb/index tips 1 cm apart
    j[1].joint[15] = pose(0.2f, 1.2f, -0.27f); j[1].joint[20] = pose(0.2f, 1.2f, -0.33f); j[1].joint[25] = pose(0.23f, 1.2f, -0.3f);   // fist: 3 cm from palm
    trk.hand[0].flags = VR4_HAND_ACTIVE | VR4_HAND_POSE_VALID; trk.hand[0].buttons = VR4_BTN_X; trk.hand[0].stick_x = 0.5f; trk.hand[0].stick_y = -0.25f;
    trk.hand[0].aim = pose(-0.2f, 1.1f, -0.3f); trk.hand[0].grip = trk.hand[0].aim;
    trk.hand[1].flags = VR4_HAND_ACTIVE | VR4_HAND_POSE_VALID | VR4_HAND_TRACKED | VR4_HAND_PINCH_READY; trk.hand[1].trigger = 1;
    trk.hand[1].aim = pose(0.25f, 1.2f, -0.35f); trk.hand[1].grip = j[1].joint[0];
    trk.time_ns += 11000000; put_track(&trk, j);
    sync();
    XrPath idxP = path("/interaction_profiles/valve/index_controller"), handP = path("/interaction_profiles/ext/hand_interaction_ext");
    XrPath profL = profile_of("/user/hand/left"), profR = profile_of("/user/hand/right");
    EXPECT(profL == idxP && profR == handP, "profiles left %llu right %llu", (unsigned long long)profL, (unsigned long long)profR);
    EXPECT(drain_events(XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) == 1, "profile event when the right hand put its controller down");
    XrBool32 act;
    EXPECT(get_bool(jump, "/user/hand/left", &act) && act, "Index left a = X button");
    EXPECT(!get_bool(jump, "/user/hand/right", &act) && !act, "right hand has no jump binding while hand-tracked");
    EXPECT(CLOSE(get_float(trig, "/user/hand/right", &act), 1) && act, "pinch_ext/value from the pinch");
    EXPECT(CLOSE(get_float(grab, "/user/hand/right", NULL), 1), "grasp_ext/value from curled fingers");
    XrVector2f mv = get_vec2(move, "/user/hand/left");
    EXPECT(CLOSE(mv.x, 0.5f) && CLOSE(mv.y, -0.25f), "Index thumbstick = stick (%f %f)", mv.x, mv.y);
    {   // poke pose = index fingertip
        XrActionSpaceCreateInfo sci = {XR_TYPE_ACTION_SPACE_CREATE_INFO, NULL, poke, path("/user/hand/right"), {{0, 0, 0, 1}, {0, 0, 0}}};
        XrSpace ps; CHECK(xrCreateActionSpace(ses, &sci, &ps));
        XrSpaceLocation l = {XR_TYPE_SPACE_LOCATION}; CHECK(xrLocateSpace(ps, stage, 0, &l));
        EXPECT((l.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) && CLOSE(l.pose.position.x, 0.31f) && CLOSE(l.pose.position.z, -0.35f), "poke pose");
    }
    {   // bound sources and the physical control's name
        XrBoundSourcesForActionEnumerateInfo ei = {XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO, NULL, jump};
        XrPath src[8]; uint32_t n; CHECK(xrEnumerateBoundSourcesForAction(ses, &ei, 8, &n, src));
        EXPECT(n == 1 && src[0] == path("/user/hand/left/input/a/click"), "bound sources %u", n);
        XrInputSourceLocalizedNameGetInfo ni = {XR_TYPE_INPUT_SOURCE_LOCALIZED_NAME_GET_INFO, NULL, src[0],
            XR_INPUT_SOURCE_LOCALIZED_NAME_USER_PATH_BIT | XR_INPUT_SOURCE_LOCALIZED_NAME_INTERACTION_PROFILE_BIT | XR_INPUT_SOURCE_LOCALIZED_NAME_COMPONENT_BIT};
        char name[128]; CHECK(xrGetInputSourceLocalizedName(ses, &ni, sizeof name, &n, name));
        EXPECT(!strcmp(name, "Left Hand Touch Controller X Button"), "localized name '%s'", name);
    }
    printf("interaction profiles ok\n");

    XrSystemHandTrackingPropertiesEXT htp = {XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
    XrSystemProperties sp = {XR_TYPE_SYSTEM_PROPERTIES, &htp}; CHECK(xrGetSystemProperties(inst, sys, &sp));
    EXPECT(htp.supportsHandTracking, "supportsHandTracking");
    XrHandTrackerEXT ht[2];
    for (int hd = 0; hd < 2; hd++) {
        XrHandTrackingDataSourceEXT srcs[] = {XR_HAND_TRACKING_DATA_SOURCE_UNOBSTRUCTED_EXT};
        XrHandTrackingDataSourceInfoEXT dsi = {XR_TYPE_HAND_TRACKING_DATA_SOURCE_INFO_EXT, NULL, 1, srcs};
        XrHandTrackerCreateInfoEXT ci = {XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT, &dsi, hd ? XR_HAND_RIGHT_EXT : XR_HAND_LEFT_EXT, XR_HAND_JOINT_SET_DEFAULT_EXT};
        CHECK(xrCreateHandTrackerEXT(ses, &ci, &ht[hd]));
    }
    XrHandJointLocationEXT jl[XR_HAND_JOINT_COUNT_EXT]; XrHandJointVelocityEXT jv[XR_HAND_JOINT_COUNT_EXT];
    XrHandTrackingDataSourceStateEXT dss = {XR_TYPE_HAND_TRACKING_DATA_SOURCE_STATE_EXT};
    XrHandTrackingAimStateFB aim = {XR_TYPE_HAND_TRACKING_AIM_STATE_FB, &dss};
    XrHandJointVelocitiesEXT vels = {XR_TYPE_HAND_JOINT_VELOCITIES_EXT, &aim, XR_HAND_JOINT_COUNT_EXT, jv};
    XrHandJointLocationsEXT locs = {XR_TYPE_HAND_JOINT_LOCATIONS_EXT, &vels, 0, XR_HAND_JOINT_COUNT_EXT, jl};
    XrHandJointsLocateInfoEXT hli = {XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT, NULL, stage, 0};
    CHECK(xrLocateHandJointsEXT(ht[0], &hli, &locs));
    EXPECT(!locs.isActive && !dss.isActive && !jl[0].locationFlags, "left hand holds a controller: inactive");
    CHECK(xrLocateHandJointsEXT(ht[1], &hli, &locs));
    EXPECT(locs.isActive && dss.isActive && dss.dataSource == XR_HAND_TRACKING_DATA_SOURCE_UNOBSTRUCTED_EXT, "right hand active");
    EXPECT(CLOSE(jl[10].pose.position.x, 0.31f) && CLOSE(jl[10].pose.position.y, 1.25f) && jl[10].radius > 0.004f && jl[10].radius < 0.02f &&
           (jl[10].locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT), "index tip joint");
    EXPECT((aim.status & XR_HAND_TRACKING_AIM_COMPUTED_BIT_FB) && (aim.status & XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB) &&
           (aim.status & XR_HAND_TRACKING_AIM_DOMINANT_HAND_BIT_FB) && CLOSE(aim.pinchStrengthIndex, 1) && CLOSE(aim.aimPose.position.x, 0.25f),
           "aim state 0x%llx index %.2f", (unsigned long long)aim.status, aim.pinchStrengthIndex);
    for (int i = 0; i < VR4_HAND_JOINTS; i++) j[1].joint[i].px += 0.01f;   // 1 cm in 10 ms = 1 m/s
    trk.time_ns += 10000000; put_track(&trk, j);
    sync();
    CHECK(xrLocateHandJointsEXT(ht[1], &hli, &locs));
    EXPECT((jv[3].velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) && fabsf(jv[3].linearVelocity.x - 1) < 0.01f && fabsf(jv[3].linearVelocity.y) < 0.01f,
           "joint velocity %.3f %.3f", jv[3].linearVelocity.x, jv[3].linearVelocity.y);
    {   // a tracker that only accepts controller-driven data never activates
        XrHandTrackingDataSourceEXT srcs[] = {XR_HAND_TRACKING_DATA_SOURCE_CONTROLLER_EXT};
        XrHandTrackingDataSourceInfoEXT dsi = {XR_TYPE_HAND_TRACKING_DATA_SOURCE_INFO_EXT, NULL, 1, srcs};
        XrHandTrackerCreateInfoEXT ci = {XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT, &dsi, XR_HAND_RIGHT_EXT, XR_HAND_JOINT_SET_DEFAULT_EXT};
        XrHandTrackerEXT ctl; CHECK(xrCreateHandTrackerEXT(ses, &ci, &ctl));
        XrHandJointLocationsEXT l2 = {XR_TYPE_HAND_JOINT_LOCATIONS_EXT, NULL, 0, XR_HAND_JOINT_COUNT_EXT, jl};
        CHECK(xrLocateHandJointsEXT(ctl, &hli, &l2)); EXPECT(!l2.isActive, "controller-only data source inactive");
    }
    printf("hand tracking ok\n");

    // ---- controller picked back up: the right hand returns to the Index profile, fed by the Touch trigger
    trk.hand[1].flags = VR4_HAND_ACTIVE | VR4_HAND_POSE_VALID; trk.hand[1].trigger = 0.75f; trk.hand[1].buttons = VR4_BTN_A;
    trk.time_ns += 11000000; put_track(&trk, NULL);
    sync();
    EXPECT(profile_of("/user/hand/right") == idxP && drain_events(XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) == 1, "right hand back on Index");
    EXPECT(CLOSE(get_float(trig, "/user/hand/right", NULL), 0.75f) && get_bool(jump, "/user/hand/right", NULL), "Index trigger/a from Touch");
    XrVector2f pad = get_vec2(move, "/user/hand/right");
    EXPECT(pad.x == 0 && pad.y == 0, "Index trackpad stays idle next to a real stick");
    printf("controller handover ok\n");

    // ---- compatibility extensions
    {
        XrVisibilityMaskKHR m = {XR_TYPE_VISIBILITY_MASK_KHR};
        CHECK(xrGetVisibilityMaskKHR(ses, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR, &m));
        EXPECT(m.vertexCountOutput == 0 && m.indexCountOutput == 0, "hidden mesh empty");
        XrVector2f v[4]; uint32_t idx[6];
        m.vertexCapacityInput = 4; m.vertices = v; m.indexCapacityInput = 6; m.indices = idx;
        CHECK(xrGetVisibilityMaskKHR(ses, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, XR_VISIBILITY_MASK_TYPE_VISIBLE_TRIANGLE_MESH_KHR, &m));
        EXPECT(m.vertexCountOutput == 4 && m.indexCountOutput == 6 && CLOSE(v[0].x, tanf(-0.8f)) && CLOSE(v[2].y, tanf(0.8f)) && idx[5] == 3, "visible mesh");
        CHECK(xrPerfSettingsSetPerformanceLevelEXT(ses, XR_PERF_SETTINGS_DOMAIN_GPU_EXT, XR_PERF_SETTINGS_LEVEL_BOOST_EXT));
        XrSpaceLocationData ld[2]; XrSpaceVelocityData vd[2]; XrSpace sps[2] = {view, stage};
        XrSpaceVelocities sv = {XR_TYPE_SPACE_VELOCITIES, NULL, 2, vd}; XrSpaceLocations sl = {XR_TYPE_SPACE_LOCATIONS, &sv, 2, ld};
        XrSpacesLocateInfo sli = {XR_TYPE_SPACES_LOCATE_INFO, NULL, stage, 0, 2, sps};
        CHECK(xrLocateSpaces(ses, &sli, &sl));
        EXPECT(CLOSE(ld[0].pose.position.y, 1.6f) && (ld[1].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT), "xrLocateSpaces");
        PFN_xrVoidFunction fn; EXPECT(gipa(inst, "xrNotARealFunction", &fn) == XR_ERROR_FUNCTION_UNSUPPORTED && !fn, "unknown function");
        printf("extensions ok\n");
    }
    {   // waiting for a headset that sends nothing must not spin a core
        FILETIME c0, e0, k0, u0, k1, u1; GetProcessTimes(GetCurrentProcess(), &c0, &e0, &k0, &u0);
        LARGE_INTEGER q0, q1, qf; QueryPerformanceCounter(&q0); QueryPerformanceFrequency(&qf);
        for (int i = 0; i < 20; i++) frame(NULL, 0, 0);
        QueryPerformanceCounter(&q1); GetProcessTimes(GetCurrentProcess(), &c0, &e0, &k1, &u1);
        double wall = (double)(q1.QuadPart - q0.QuadPart) / qf.QuadPart;
        #define FT(t) ((double)(((uint64_t)(t).dwHighDateTime << 32) | (t).dwLowDateTime))
        double cpu = (FT(u1) - FT(u0) + FT(k1) - FT(k0)) * 1e-7;
        printf("xrWaitFrame idle: %.0f ms wall, %.0f ms cpu\n", wall * 1e3, cpu * 1e3);
        EXPECT(wall > 0.15 && cpu < wall * 0.5, "xrWaitFrame busy-waits");
    }

    // ---- a Vive-only game (new instance): trackpad = thumbstick, pad click = stick click / A, right menu = B
    CHECK(xrDestroySession(ses)); CHECK(xrDestroyInstance(inst));
    start("Vive-only game", &gb11);
    XrAction vmove = action("move", XR_ACTION_TYPE_VECTOR2F_INPUT), vmenu = action("menu", XR_ACTION_TYPE_BOOLEAN_INPUT), vtp = action("teleport", XR_ACTION_TYPE_BOOLEAN_INPUT);
    XrAction vgrip = action("grip", XR_ACTION_TYPE_BOOLEAN_INPUT);
    XrActionSuggestedBinding vb[] = {{vmove, path("/user/hand/right/input/trackpad")}, {vmenu, path("/user/hand/right/input/menu/click")},
        {vtp, path("/user/hand/right/input/trackpad/click")}, {vgrip, path("/user/hand/right/input/squeeze/click")}, {vmenu, path("/user/hand/left/input/menu/click")}};
    suggest("/interaction_profiles/htc/vive_controller", vb, sizeof vb / sizeof *vb);
    at.actionSets = &set; CHECK(xrAttachSessionActionSets(ses, &at));
    trk.hand[1].flags = VR4_HAND_ACTIVE | VR4_HAND_POSE_VALID; trk.hand[1].buttons = VR4_BTN_B | VR4_BTN_A; trk.hand[1].stick_x = -0.6f; trk.hand[1].squeeze = 0.9f;
    trk.hand[0].buttons = 0;
    trk.time_ns += 11000000; put_track(&trk, NULL);
    sync();
    EXPECT(profile_of("/user/hand/right") == path("/interaction_profiles/htc/vive_controller"), "vive profile");
    mv = get_vec2(vmove, "/user/hand/right");
    EXPECT(CLOSE(mv.x, -0.6f) && get_bool(vmenu, "/user/hand/right", NULL) && !get_bool(vmenu, "/user/hand/left", NULL) &&
           get_bool(vtp, "/user/hand/right", NULL) && get_bool(vgrip, "/user/hand/right", NULL), "vive remaps");
    printf("vive remaps ok\n");

    // ---- a D3D12 game (XR_KHR_D3D12_enable): frames are copied by the runtime's command lists on the game's queue
    CHECK(xrDestroySession(ses)); CHECK(xrDestroyInstance(inst));
    ID3D12Device *d12; ID3D12CommandQueue *q; ID3D12CommandAllocator *ca; ID3D12GraphicsCommandList *gl; ID3D12DescriptorHeap *heap;
    D3D12_COMMAND_QUEUE_DESC qd = {D3D12_COMMAND_LIST_TYPE_DIRECT}; D3D12_DESCRIPTOR_HEAP_DESC hd = {D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 1};
    EXPECT(SUCCEEDED(D3D12CreateDevice(NULL, D3D_FEATURE_LEVEL_11_0, &IID_ID3D12Device, (void **)&d12)) &&
           SUCCEEDED(ID3D12Device_CreateCommandQueue(d12, &qd, &IID_ID3D12CommandQueue, (void **)&q)) &&
           SUCCEEDED(ID3D12Device_CreateCommandAllocator(d12, D3D12_COMMAND_LIST_TYPE_DIRECT, &IID_ID3D12CommandAllocator, (void **)&ca)) &&
           SUCCEEDED(ID3D12Device_CreateCommandList(d12, 0, D3D12_COMMAND_LIST_TYPE_DIRECT, ca, NULL, &IID_ID3D12GraphicsCommandList, (void **)&gl)) &&
           SUCCEEDED(ID3D12Device_CreateDescriptorHeap(d12, &hd, &IID_ID3D12DescriptorHeap, (void **)&heap)), "D3D12 device");
    XrGraphicsBindingD3D12KHR gb12 = {XR_TYPE_GRAPHICS_BINDING_D3D12_KHR, NULL, d12, q};
    start("D3D12 game", &gb12);
    XrGraphicsRequirementsD3D12KHR r12 = {XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR}; CHECK(xrGetD3D12GraphicsRequirementsKHR(inst, sys, &r12));
    {
        XrSwapchainCreateInfo ci = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 1, 2 * w, h, 1, 1, 1};
        Chain c12 = {0}; CHECK(xrCreateSwapchain(ses, &ci, &c12.sc));
        XrSwapchainImageD3D12KHR im[3] = {{XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR}};
        uint32_t n, idx; CHECK(xrEnumerateSwapchainImages(c12.sc, 3, &n, (XrSwapchainImageBaseHeader *)im));
        CHECK(xrAcquireSwapchainImage(c12.sc, NULL, &idx));
        XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, NULL, XR_INFINITE_DURATION}; CHECK(xrWaitSwapchainImage(c12.sc, &wi));
        D3D12_RENDER_TARGET_VIEW_DESC rd = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, D3D12_RTV_DIMENSION_TEXTURE2D};
        D3D12_CPU_DESCRIPTOR_HANDLE rtv = ID3D12DescriptorHeap_GetCPUDescriptorHandleForHeapStart(heap);
        ID3D12Device_CreateRenderTargetView(d12, im[idx].texture, &rd, rtv);   // images arrive in RENDER_TARGET state
        ID3D12GraphicsCommandList_ClearRenderTargetView(gl, rtv, GREEN, 0, NULL);
        ID3D12GraphicsCommandList_Close(gl);
        ID3D12CommandQueue_ExecuteCommandLists(q, 1, (ID3D12CommandList *const *)&gl);
        CHECK(xrReleaseSwapchainImage(c12.sc, NULL));
        XrCompositionLayerProjection p12 = projection_into(&c12, stage, pviews12);
        const XrCompositionLayerBaseHeader *l12[] = {(XrCompositionLayerBaseHeader *)&p12};
        Probe pg[] = {{2, 2, PX_GREEN}, {cr, cy, PX_GREEN}};
        expect_frame("D3D12 projection", l12, 1, pg, 2);
        CHECK(xrDestroySession(ses));
    }
    printf("PASS\n");
    return 0;
}
