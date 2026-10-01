// VR4Mac OpenXR runtime: a Windows DLL loaded by games (directly, or via OpenComposite for OpenVR games)
// inside Wine. Poses/input come from the Mac app through shared memory (common/vr4mac.h); rendered eye
// images are copied back the same way and streamed to the Quest by the Mac app. D3D11 only.
#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include <openxr/openxr_reflection.h>
#include "../common/vr4mac.h"

#define EXPORT __declspec(dllexport)
#define MAXN 256
#define NSTAGE 4   // readback ring depth (GPU latency under D3DMetal is >1 frame)

static VR4Shm *shm;
static FILE *logfile;
static void logmsg(const char *fmt, ...) {
    if (!logfile) logfile = fopen("Z:\\tmp\\vr4mac\\runtime.log", "a");
    if (!logfile) return;
    va_list a; va_start(a, fmt); vfprintf(logfile, fmt, a); va_end(a); fputc('\n', logfile); fflush(logfile);
}

static float current_render_scale(void) {
    return (shm && shm->render_scale > 0.01f) ? shm->render_scale : 1.0f;
}
static float current_world_scale(void) {
    return (shm && shm->world_scale > 0.01f) ? shm->world_scale : 1.0f;
}

// ---------------------------------------------------------------- math
typedef XrQuaternionf Q; typedef XrVector3f V;
static Q qmul(Q a, Q b) {
    return (Q){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
               a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
static Q qconj(Q a) { return (Q){-a.x, -a.y, -a.z, a.w}; }
static V qrot(Q q, V v) { Q r = qmul(qmul(q, (Q){v.x, v.y, v.z, 0}), qconj(q)); return (V){r.x, r.y, r.z}; }
static XrPosef pmul(XrPosef a, XrPosef b) {   // b expressed in a's frame -> a's parent frame
    V t = qrot(a.orientation, b.position);
    return (XrPosef){qmul(a.orientation, b.orientation), {a.position.x + t.x, a.position.y + t.y, a.position.z + t.z}};
}
static XrPosef pinv(XrPosef a) {
    Q c = qconj(a.orientation); V t = qrot(c, a.position);
    return (XrPosef){c, {-t.x, -t.y, -t.z}};
}
static XrPosef xp(VR4Pose p) {
    float ws = current_world_scale();
    return (XrPosef){{p.qx, p.qy, p.qz, p.qw}, {p.px / ws, p.py / ws, p.pz / ws}};
}
static const XrPosef IDENT = {{0, 0, 0, 1}, {0, 0, 0}};

// ---------------------------------------------------------------- paths
static char *paths[4096]; static int npaths;
static XrPath intern(const char *s) {
    for (int i = 0; i < npaths; i++) if (!strcmp(paths[i], s)) return i + 1;
    if (npaths == 4096) return XR_NULL_PATH;
    paths[npaths] = _strdup(s);
    return ++npaths;
}
static const char *pstr(XrPath p) { return p >= 1 && p <= (XrPath)npaths ? paths[p - 1] : ""; }

// ---------------------------------------------------------------- objects
typedef struct { int hand; char comp[64]; } Binding;
typedef struct { uint32_t gen; float cur[2], prev[2]; } ActionHistory;
typedef struct Action { XrActionType type; char name[64]; XrPath sub[8]; int nsub; Binding b[16]; int nb; ActionHistory hist[3]; } Action;
typedef struct { char name[64]; } ActionSet;
typedef struct { XrPath profile; Action *action; XrPath binding; } Suggestion;
typedef struct {
    XrSession session; int ref; XrReferenceSpaceType type; Action *action; XrPath sub; XrPosef offset;
} Space;
typedef struct {
    ID3D11Texture2D *img[3]; int count, acquired, released; DXGI_FORMAT fmt; uint32_t w, h, array, mips;
} Swapchain;
typedef struct {
    ID3D11Device *dev; ID3D11DeviceContext *ctx; int running, focused, exitRequested;
    ID3D11Texture2D *staging[NSTAGE][2]; DXGI_FORMAT stagingFmt; uint32_t stagingW, stagingH; uint64_t stagingCounter;
    uint64_t stagingReady[NSTAGE]; VR4Pose stagingPose[NSTAGE][2]; uint64_t stagingDisplayTime[NSTAGE]; // stagingReady: submit order, 0 = free
    XrPosef localOrigin; XrPath profile;
} Session;


static Suggestion sugg[1024]; static int nsugg;
static VR4Tracking track;           // input snapshot (hands/buttons), refreshed by xrSyncActions
static VR4Tracking frameTrack;      // head/eyes of the latest xrWaitFrame
static VR4Tracking frameRing[4]; static int frameRingN;   // recent xrWaitFrame snapshots, for pipelined apps
static uint32_t lastSeq;
static uint32_t syncGen;   // bumped by xrSyncActions
static XrEventDataBuffer events[64]; static int evHead, evTail;
static Session *theSession;
static char appName[128];
static LARGE_INTEGER qpf;
static int64_t qpcOffsetNs;         // XrTime - QPC time

static int64_t qpc_ns(void) { LARGE_INTEGER c; QueryPerformanceCounter(&c); return (int64_t)((double)c.QuadPart * 1e9 / (double)qpf.QuadPart); }

static void push_state(XrSessionState st) {
    if (evTail - evHead >= 64) evHead = evTail - 63;
    XrEventDataSessionStateChanged *e = (XrEventDataSessionStateChanged *)&events[evTail % 64];
    memset(e, 0, sizeof(XrEventDataBuffer));
    e->type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED; e->session = (XrSession)theSession; e->state = st;
    e->time = qpc_ns() + qpcOffsetNs;
    evTail++;
}

/// Head/eye snapshot handed out by the xrWaitFrame that predicted `t` (falls back to the latest).
static const VR4Tracking *frame_for(XrTime t) {
    for (int i = 0; i < 4; i++) if (frameRing[i].time_ns == (uint64_t)t && t) return &frameRing[i];
    return &frameTrack;
}
static int read_tracking(void) {   // seqlock read of the Mac app's latest tracking sample
    for (int tries = 0; tries < 100; tries++) {
        uint32_t s1 = shm->track_seq; vr4_fence();
        if (s1 & 1) continue;
        VR4Tracking t = shm->track; vr4_fence();
        if (shm->track_seq == s1) { int fresh = s1 != lastSeq; track = t; lastSeq = s1; return fresh; }
    }
    return 0;
}

// ---------------------------------------------------------------- input mapping
static int btn(const VR4Hand *h, uint32_t bit) { return (h->buttons & bit) != 0; }
/// Scalar value of an input component path suffix like "trigger/value" or "a/click".
static float comp_value(const VR4Hand *h, const char *c) {
    #define IS(p) (!strncmp(c, p, strlen(p)))
    if (IS("trigger/touch")) return btn(h, VR4_BTN_TRIGGER_TOUCH);
    if (IS("trigger") || IS("select")) return h->trigger;
    if (IS("squeeze") || IS("grip/value") || IS("grip/click")) return h->squeeze;
    if (IS("thumbstick/x") || IS("trackpad/x")) return h->stick_x;
    if (IS("thumbstick/y") || IS("trackpad/y")) return h->stick_y;
    if (IS("thumbstick/click") || IS("trackpad/click")) return btn(h, VR4_BTN_STICK_CLICK);
    if (IS("thumbstick/touch") || IS("trackpad/touch")) return btn(h, VR4_BTN_STICK_TOUCH);
    if (IS("thumbrest/touch")) return btn(h, VR4_BTN_THUMB_TOUCH);
    if (IS("a/click")) return btn(h, VR4_BTN_A);
    if (IS("b/click")) return btn(h, VR4_BTN_B);
    if (IS("x/click")) return btn(h, VR4_BTN_X);
    if (IS("y/click")) return btn(h, VR4_BTN_Y);
    if (IS("a/touch") || IS("b/touch") || IS("x/touch") || IS("y/touch")) return btn(h, VR4_BTN_THUMB_TOUCH);
    if (IS("menu/click")) return btn(h, VR4_BTN_MENU);
    return 0;
    #undef IS
}
static int is_pose_comp(const char *c) { return !strcmp(c, "grip/pose") || !strcmp(c, "aim/pose") || !strcmp(c, "palm_ext/pose") || !strcmp(c, "grip_surface/pose"); }
static int parse_binding(const char *path, Binding *b) {   // "/user/hand/left/input/trigger/value"
    if (!strncmp(path, "/user/hand/left/", 16)) b->hand = 0;
    else if (!strncmp(path, "/user/hand/right/", 17)) b->hand = 1;
    else return 0;
    const char *c = strstr(path, "/input/");
    if (c) c += 7; else if ((c = strstr(path, "/output/"))) c += 8; else return 0;
    snprintf(b->comp, sizeof b->comp, "%s", c);
    return 1;
}
static int sub_matches(XrPath sub, int hand) {
    if (sub == XR_NULL_PATH) return 1;
    return !strcmp(pstr(sub), hand ? "/user/hand/right" : "/user/hand/left");
}
static XrPosef hand_pose(int hand, const char *comp, int *valid) {
    const VR4Hand *h = &track.hand[hand];
    *valid = (h->flags & VR4_HAND_POSE_VALID) != 0;
    return xp(!strcmp(comp, "aim/pose") ? h->aim : h->grip);
}

// ---------------------------------------------------------------- spaces
static XrPosef space_in_stage(Space *s, int *valid) {
    *valid = 1;
    if (s->ref) switch (s->type) {
        case XR_REFERENCE_SPACE_TYPE_VIEW: return pmul(xp(frameTrack.head), s->offset);
        case XR_REFERENCE_SPACE_TYPE_LOCAL: return pmul(((Session *)s->session)->localOrigin, s->offset);
        case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR: {
            XrPosef o = ((Session *)s->session)->localOrigin; o.position.y = 0; return pmul(o, s->offset);
        }
        default: return s->offset;
    }
    for (int i = 0; i < s->action->nb; i++) {
        Binding *b = &s->action->b[i];
        if (is_pose_comp(b->comp) && sub_matches(s->sub, b->hand)) return pmul(hand_pose(b->hand, b->comp, valid), s->offset);
    }
    *valid = 0;
    return IDENT;
}
static void set_local_origin(Session *s) {   // LOCAL = head position at start, yaw only
    float ws = current_world_scale();
    Q q = {track.head.qx, track.head.qy, track.head.qz, track.head.qw};
    V f = qrot(q, (V){0, 0, -1});
    float yaw = atan2f(-f.x, -f.z);
    s->localOrigin = (XrPosef){{0, sinf(yaw / 2), 0, cosf(yaw / 2)}, {track.head.px / ws, track.head.py / ws, track.head.pz / ws}};
    if (track.time_ns == 0) s->localOrigin.position.y = 1.6f / ws;
}

// ---------------------------------------------------------------- instance
#define FILL_ARRAY(cap, countOut, arr, n, ...) do { \
    if (countOut) *(countOut) = (n); \
    if ((cap) == 0) return XR_SUCCESS; \
    if ((cap) < (n)) return XR_ERROR_SIZE_INSUFFICIENT; \
    for (uint32_t i_ = 0; i_ < (n); i_++) { __VA_ARGS__; } } while (0)

// Compatibility: games refuse to start or wait forever without some of these (BONELAB blocks on XR_FB_display_refresh_rate).
// Depth / cylinder layers are accepted and ignored; the visibility mask is empty; LOCAL_FLOOR is the local origin on the floor.
static const char *exts[] = {XR_KHR_D3D11_ENABLE_EXTENSION_NAME, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME,
                             XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME,
                             XR_KHR_COMPOSITION_LAYER_CYLINDER_EXTENSION_NAME, XR_KHR_VISIBILITY_MASK_EXTENSION_NAME,
                             XR_EXT_LOCAL_FLOOR_EXTENSION_NAME, XR_FB_COLOR_SPACE_EXTENSION_NAME,
                             XR_KHR_COMPOSITION_LAYER_COLOR_SCALE_BIAS_EXTENSION_NAME};
static const uint32_t extVer[] = {XR_KHR_D3D11_enable_SPEC_VERSION, XR_KHR_win32_convert_performance_counter_time_SPEC_VERSION,
                                  XR_FB_display_refresh_rate_SPEC_VERSION, XR_KHR_composition_layer_depth_SPEC_VERSION,
                                  XR_KHR_composition_layer_cylinder_SPEC_VERSION, XR_KHR_visibility_mask_SPEC_VERSION,
                                  XR_EXT_local_floor_SPEC_VERSION, XR_FB_color_space_SPEC_VERSION,
                                  XR_KHR_composition_layer_color_scale_bias_SPEC_VERSION};
#define NEXTS (sizeof exts / sizeof *exts)

static XrResult XRAPI_CALL xrEnumerateApiLayerProperties_(uint32_t cap, uint32_t *n, XrApiLayerProperties *p) { (void)cap; (void)p; *n = 0; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrEnumerateInstanceExtensionProperties_(const char *layer, uint32_t cap, uint32_t *n, XrExtensionProperties *p) {
    (void)layer;
    FILL_ARRAY(cap, n, p, (uint32_t)NEXTS, { strcpy(p[i_].extensionName, exts[i_]); p[i_].extensionVersion = extVer[i_]; });
    return XR_SUCCESS;
}

static XrResult XRAPI_CALL xrCreateInstance_(const XrInstanceCreateInfo *ci, XrInstance *out) {
    for (uint32_t i = 0; i < ci->enabledExtensionCount; i++) {
        int ok = 0;
        for (size_t j = 0; j < NEXTS; j++) ok |= !strcmp(ci->enabledExtensionNames[i], exts[j]);
        if (!ok) { logmsg("unsupported extension %s", ci->enabledExtensionNames[i]); return XR_ERROR_EXTENSION_NOT_PRESENT; }
    }
    if (!shm) {
        HANDLE f = CreateFileA(VR4_SHM_PATH_WIN, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
        if (f == INVALID_HANDLE_VALUE) { logmsg("VR4Mac app not running (no %s)", VR4_SHM_PATH_WIN); return XR_ERROR_RUNTIME_UNAVAILABLE; }
        HANDLE m = CreateFileMappingA(f, NULL, PAGE_READWRITE, 0, VR4_SHM_SIZE, NULL);
        shm = m ? (VR4Shm *)MapViewOfFile(m, FILE_MAP_ALL_ACCESS, 0, 0, VR4_SHM_SIZE) : NULL;
        if (!shm || shm->magic != VR4_SHM_MAGIC || shm->version != VR4_SHM_VERSION) { logmsg("bad shared memory"); shm = NULL; return XR_ERROR_RUNTIME_UNAVAILABLE; }
    }
    QueryPerformanceFrequency(&qpf);
    snprintf(appName, sizeof appName, "%s", ci->applicationInfo.applicationName);
    snprintf(shm->app_name, sizeof shm->app_name, "%s", appName);
    logmsg("xrCreateInstance app=%s engine=%s", ci->applicationInfo.applicationName, ci->applicationInfo.engineName);
    *out = (XrInstance)(uintptr_t)1;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroyInstance_(XrInstance i) { (void)i; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrGetInstanceProperties_(XrInstance i, XrInstanceProperties *p) {
    (void)i; p->runtimeVersion = XR_MAKE_VERSION(0, 2, 0); strcpy(p->runtimeName, "VR4Mac"); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrPollEvent_(XrInstance i, XrEventDataBuffer *e) {
    (void)i;
    Session *s = theSession;
    if (s && s->running && !s->exitRequested) {   // dashboard open on the Mac side = app loses input focus
        int focused = !shm->input_blocked;
        if (focused != s->focused) { s->focused = focused; push_state(focused ? XR_SESSION_STATE_FOCUSED : XR_SESSION_STATE_VISIBLE); }
    }
    if (evHead == evTail) return XR_EVENT_UNAVAILABLE;
    memcpy(e, &events[evHead++ % 64], sizeof *e);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrResultToString_(XrInstance i, XrResult r, char buf[XR_MAX_RESULT_STRING_SIZE]) {
    (void)i;
    #define RS(name, val) case name: snprintf(buf, XR_MAX_RESULT_STRING_SIZE, "%s", #name); break;
    switch (r) { XR_LIST_ENUM_XrResult(RS) default: snprintf(buf, XR_MAX_RESULT_STRING_SIZE, "XR_UNKNOWN_%d", r); }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrStructureTypeToString_(XrInstance i, XrStructureType t, char buf[XR_MAX_STRUCTURE_NAME_SIZE]) {
    (void)i;
    #undef RS
    #define RS(name, val) case name: snprintf(buf, XR_MAX_STRUCTURE_NAME_SIZE, "%s", #name); break;
    switch (t) { XR_LIST_ENUM_XrStructureType(RS) default: snprintf(buf, XR_MAX_STRUCTURE_NAME_SIZE, "XR_UNKNOWN_STRUCTURE_TYPE_%d", t); }
    #undef RS
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrStringToPath_(XrInstance i, const char *s, XrPath *p) { (void)i; *p = intern(s); return *p ? XR_SUCCESS : XR_ERROR_PATH_COUNT_EXCEEDED; }
static XrResult XRAPI_CALL xrPathToString_(XrInstance i, XrPath p, uint32_t cap, uint32_t *n, char *buf) {
    (void)i;
    if (p < 1 || p > (XrPath)npaths) return XR_ERROR_PATH_INVALID;
    const char *s = pstr(p); uint32_t len = (uint32_t)strlen(s) + 1;
    *n = len;
    if (!cap) return XR_SUCCESS;
    if (cap < len) return XR_ERROR_SIZE_INSUFFICIENT;
    memcpy(buf, s, len);
    return XR_SUCCESS;
}

// ---------------------------------------------------------------- system
static XrResult XRAPI_CALL xrGetSystem_(XrInstance i, const XrSystemGetInfo *gi, XrSystemId *id) {
    (void)i;
    if (gi->formFactor != XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY) return XR_ERROR_FORM_FACTOR_UNSUPPORTED;
    *id = 1; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetSystemProperties_(XrInstance i, XrSystemId id, XrSystemProperties *p) {
    (void)i; (void)id;
    p->systemId = 1; p->vendorId = 0x2833;
    strcpy(p->systemName, "VR4Mac Quest");
    p->graphicsProperties.maxSwapchainImageWidth = 4096; p->graphicsProperties.maxSwapchainImageHeight = 4096;
    p->graphicsProperties.maxLayerCount = XR_MIN_COMPOSITION_LAYERS_SUPPORTED;
    p->trackingProperties.orientationTracking = XR_TRUE; p->trackingProperties.positionTracking = XR_TRUE;
    for (XrBaseOutStructure *next = (XrBaseOutStructure *)p->next; next; next = next->next) {
        if (next->type == XR_TYPE_SYSTEM_COLOR_SPACE_PROPERTIES_FB) {
            ((XrSystemColorSpacePropertiesFB *)next)->colorSpace = XR_COLOR_SPACE_QUEST_FB;
        }
    }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateEnvironmentBlendModes_(XrInstance i, XrSystemId id, XrViewConfigurationType v, uint32_t cap, uint32_t *n, XrEnvironmentBlendMode *m) {
    (void)i; (void)id; (void)v; FILL_ARRAY(cap, n, m, 1, m[i_] = XR_ENVIRONMENT_BLEND_MODE_OPAQUE); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateViewConfigurations_(XrInstance i, XrSystemId id, uint32_t cap, uint32_t *n, XrViewConfigurationType *t) {
    (void)i; (void)id; FILL_ARRAY(cap, n, t, 1, t[i_] = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetViewConfigurationProperties_(XrInstance i, XrSystemId id, XrViewConfigurationType t, XrViewConfigurationProperties *p) {
    (void)i; (void)id;
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    p->viewConfigurationType = t; p->fovMutable = XR_FALSE; return XR_SUCCESS;
}
static uint32_t eye_w(void) {
    uint32_t base = (shm && shm->eye_w) ? shm->eye_w : 1440;
    float s = current_render_scale();
    uint32_t w = (uint32_t)((float)base * s);
    uint32_t rounded = ((w + 16) / 32) * 32;
    return rounded < 128 ? 128 : (rounded > 4096 ? 4096 : rounded);
}
static uint32_t eye_h(void) {
    uint32_t base = (shm && shm->eye_h) ? shm->eye_h : 1584;
    float s = current_render_scale();
    uint32_t h = (uint32_t)((float)base * s);
    uint32_t rounded = ((h + 16) / 32) * 32;
    return rounded < 128 ? 128 : (rounded > 4096 ? 4096 : rounded);
}
static XrResult XRAPI_CALL xrEnumerateViewConfigurationViews_(XrInstance i, XrSystemId id, XrViewConfigurationType t, uint32_t cap, uint32_t *n, XrViewConfigurationView *v) {
    (void)i; (void)id;
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    FILL_ARRAY(cap, n, v, 2, {
        v[i_].recommendedImageRectWidth = eye_w(); v[i_].recommendedImageRectHeight = eye_h();
        v[i_].maxImageRectWidth = 4096; v[i_].maxImageRectHeight = 4096;
        v[i_].recommendedSwapchainSampleCount = 1; v[i_].maxSwapchainSampleCount = 1; });
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetD3D11GraphicsRequirementsKHR_(XrInstance i, XrSystemId id, XrGraphicsRequirementsD3D11KHR *r) {
    (void)i; (void)id;
    IDXGIFactory1 *f; IDXGIAdapter1 *a; DXGI_ADAPTER_DESC1 d;
    memset(&r->adapterLuid, 0, sizeof r->adapterLuid);
    if (SUCCEEDED(CreateDXGIFactory1(&IID_IDXGIFactory1, (void **)&f))) {
        if (SUCCEEDED(IDXGIFactory1_EnumAdapters1(f, 0, &a))) {
            if (SUCCEEDED(IDXGIAdapter1_GetDesc1(a, &d))) r->adapterLuid = d.AdapterLuid;
            IDXGIAdapter1_Release(a);
        }
        IDXGIFactory1_Release(f);
    }
    r->minFeatureLevel = D3D_FEATURE_LEVEL_11_0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrConvertWin32PerformanceCounterToTimeKHR_(XrInstance i, const LARGE_INTEGER *c, XrTime *t) {
    (void)i; *t = (XrTime)((double)c->QuadPart * 1e9 / (double)qpf.QuadPart) + qpcOffsetNs; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrConvertTimeToWin32PerformanceCounterKHR_(XrInstance i, XrTime t, LARGE_INTEGER *c) {
    (void)i; c->QuadPart = (LONGLONG)((double)(t - qpcOffsetNs) * (double)qpf.QuadPart / 1e9); return XR_SUCCESS;
}

// ---------------------------------------------------------------- session
static XrResult XRAPI_CALL xrCreateSession_(XrInstance i, const XrSessionCreateInfo *ci, XrSession *out) {
    (void)i;
    const XrGraphicsBindingD3D11KHR *gb = NULL;
    for (const XrBaseInStructure *b = ci->next; b; b = b->next)
        if (b->type == XR_TYPE_GRAPHICS_BINDING_D3D11_KHR) gb = (const XrGraphicsBindingD3D11KHR *)b;
    if (!gb || !gb->device) { logmsg("xrCreateSession: only D3D11 is supported"); return XR_ERROR_GRAPHICS_DEVICE_INVALID; }
    Session *s = calloc(1, sizeof *s);
    s->dev = gb->device; ID3D11Device_AddRef(s->dev);
    ID3D11Device_GetImmediateContext(s->dev, &s->ctx);
    read_tracking(); frameTrack = track; set_local_origin(s);
    theSession = s;
    *out = (XrSession)s;
    push_state(XR_SESSION_STATE_IDLE); push_state(XR_SESSION_STATE_READY);
    logmsg("xrCreateSession ok, eye %ux%u", eye_w(), eye_h());
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroySession_(XrSession h) {
    Session *s = (Session *)h;
    for (int k = 0; k < NSTAGE; k++)
        for (int i = 0; i < 2; i++)
            if (s->staging[k][i]) ID3D11Texture2D_Release(s->staging[k][i]);
    ID3D11DeviceContext_Release(s->ctx); ID3D11Device_Release(s->dev);
    if (theSession == s) theSession = NULL;
    free(s);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrBeginSession_(XrSession h, const XrSessionBeginInfo *bi) {
    Session *s = (Session *)h; (void)bi;
    s->running = 1; s->focused = 1;
    push_state(XR_SESSION_STATE_SYNCHRONIZED); push_state(XR_SESSION_STATE_VISIBLE); push_state(XR_SESSION_STATE_FOCUSED);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEndSession_(XrSession h) {
    Session *s = (Session *)h;
    s->running = 0;
    push_state(XR_SESSION_STATE_IDLE); push_state(XR_SESSION_STATE_EXITING);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrRequestExitSession_(XrSession h) {
    Session *s = (Session *)h;
    s->exitRequested = 1;
    push_state(XR_SESSION_STATE_VISIBLE); push_state(XR_SESSION_STATE_SYNCHRONIZED); push_state(XR_SESSION_STATE_STOPPING);
    return XR_SUCCESS;
}

// ---------------------------------------------------------------- spaces
static XrResult XRAPI_CALL xrEnumerateReferenceSpaces_(XrSession h, uint32_t cap, uint32_t *n, XrReferenceSpaceType *t) {
    (void)h;
    static const XrReferenceSpaceType all[] = {XR_REFERENCE_SPACE_TYPE_VIEW, XR_REFERENCE_SPACE_TYPE_LOCAL,
                                               XR_REFERENCE_SPACE_TYPE_STAGE, XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR};
    FILL_ARRAY(cap, n, t, 4, t[i_] = all[i_]);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrCreateReferenceSpace_(XrSession h, const XrReferenceSpaceCreateInfo *ci, XrSpace *out) {
    if (ci->referenceSpaceType != XR_REFERENCE_SPACE_TYPE_VIEW && ci->referenceSpaceType != XR_REFERENCE_SPACE_TYPE_LOCAL &&
        ci->referenceSpaceType != XR_REFERENCE_SPACE_TYPE_STAGE && ci->referenceSpaceType != XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR) {
        logmsg("reference space %d unsupported", ci->referenceSpaceType); return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
    }
    Space *s = calloc(1, sizeof *s);
    s->session = h; s->ref = 1; s->type = ci->referenceSpaceType; s->offset = ci->poseInReferenceSpace;
    *out = (XrSpace)s; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetReferenceSpaceBoundsRect_(XrSession h, XrReferenceSpaceType t, XrExtent2Df *b) {
    (void)h;
    if (t != XR_REFERENCE_SPACE_TYPE_STAGE) { b->width = b->height = 0; return XR_SPACE_BOUNDS_UNAVAILABLE; }
    b->width = 2; b->height = 2; return XR_SUCCESS;   // ponytail: fixed 2x2 m play area, send the Quest guardian later
}
static XrResult XRAPI_CALL xrCreateActionSpace_(XrSession h, const XrActionSpaceCreateInfo *ci, XrSpace *out) {
    Space *s = calloc(1, sizeof *s);
    s->session = h; s->action = (Action *)ci->action; s->sub = ci->subactionPath; s->offset = ci->poseInActionSpace;
    *out = (XrSpace)s; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrLocateSpace_(XrSpace sp, XrSpace base, XrTime t, XrSpaceLocation *loc) {
    (void)t;
    int v1, v2;
    XrPosef a = space_in_stage((Space *)sp, &v1), b = space_in_stage((Space *)base, &v2);
    loc->pose = pmul(pinv(b), a);
    loc->locationFlags = v1 && v2 ? XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT |
                                    XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT : 0;
    for (XrBaseOutStructure *n = loc->next; n; n = n->next)
        if (n->type == XR_TYPE_SPACE_VELOCITY) {
            XrSpaceVelocity *sv = (XrSpaceVelocity *)n;
            sv->velocityFlags = 0;
            sv->linearVelocity = (XrVector3f){0, 0, 0};
            sv->angularVelocity = (XrVector3f){0, 0, 0};
        }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroySpace_(XrSpace s) { free(s); return XR_SUCCESS; }

// ---------------------------------------------------------------- frames
static int64_t endSum, copySum;   // pacing log: time inside xrEndFrame / in the CPU readback copy
static XrResult XRAPI_CALL xrWaitFrame_(XrSession h, const XrFrameWaitInfo *wi, XrFrameState *fs) {
    (void)wi; (void)h;
    float fps = shm->fps > 0 ? shm->fps : 72;
    int64_t period = (int64_t)(1e9 / fps), start = qpc_ns();
    static int64_t lastExit, statStart, waitSum, workSum; static int statN;
    // Pace the game to the headset: block until the next tracking sample, but never past one period (+2 ms jitter)
    // after the previous frame was released. The deadline used to be 2 periods from *now*, so stale tracking (headset
    // asleep while linked) stacked the full timeout on top of the game's own frame time: 6 ms game -> 29 fps.
    int64_t deadline = (lastExit ? lastExit : start) + period + 2000000;
    while (!read_tracking()) {
        if (qpc_ns() > deadline) break;
        Sleep(0);
    }
    frameTrack = track;
    frameRing[frameRingN++ % 4] = track;
    int64_t now = qpc_ns();
    if (shm->client_connected && track.time_ns) qpcOffsetNs = (int64_t)track.time_ns - now - period;
    // Connected: the exact tracking sample time, since the Quest matches video to its pose history by it. Monotonic
    // synthesis only while disconnected; never latch across timelines (045fd16 regression: navy screen).
    static int64_t lastPredicted = 0;
    int64_t predicted;
    if (shm->client_connected && track.time_ns) predicted = (int64_t)track.time_ns;
    else { predicted = now + qpcOffsetNs + period; if (predicted <= lastPredicted) predicted = lastPredicted + period; }
    lastPredicted = predicted;
    fs->predictedDisplayTime = (XrTime)predicted;
    fs->predictedDisplayPeriod = period;
    fs->shouldRender = XR_TRUE;
    // pacing diagnostics every 5 s: time blocked here vs time the game spends between frames (render+submit+present)
    int64_t exitNs = qpc_ns();
    if (lastExit) { waitSum += exitNs - start; workSum += start - lastExit; statN++; }
    lastExit = exitNs;
    if (!statStart) statStart = exitNs;
    if (exitNs - statStart > 5000000000LL && statN) {
        logmsg("pacing: %.1f fps, wait %.2f ms, game %.2f ms/frame (endframe %.2f, copy %.2f), connected %u", statN * 1e9 / (exitNs - statStart),
               waitSum / 1e6 / statN, workSum / 1e6 / statN, endSum / 1e6 / statN, copySum / 1e6 / statN, shm->client_connected);
        statStart = exitNs; waitSum = workSum = endSum = copySum = 0; statN = 0;
    }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrBeginFrame_(XrSession h, const XrFrameBeginInfo *bi) { (void)h; (void)bi; return XR_SUCCESS; }

static XrResult XRAPI_CALL xrLocateViews_(XrSession h, const XrViewLocateInfo *li, XrViewState *vs, uint32_t cap, uint32_t *n, XrView *views) {
    (void)h;
    int valid;
    XrPosef base = space_in_stage((Space *)li->space, &valid);
    const VR4Tracking *ft = frame_for(li->displayTime);
    vs->viewStateFlags = valid ? (XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT |
                                  XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT) : 0;
    FILL_ARRAY(cap, n, views, 2, {
        VR4Eye d = {{i_ ? 0.032f : -0.032f, 1.6f, 0, 0, 0, 0, 1}, {-0.8f, 0.8f, 0.8f, -0.8f}};   // no headset yet
        const VR4Eye *e = ft->time_ns ? &ft->eye[i_] : &d;
        views[i_].pose = pmul(pinv(base), xp(e->pose));
        views[i_].fov = (XrFovf){e->fov.left, e->fov.right, e->fov.up, e->fov.down};
    });
    return XR_SUCCESS;
}

static Swapchain *find_sc(XrSwapchain s) { return (Swapchain *)s; }
static int is_rgba(DXGI_FORMAT f) { return f == DXGI_FORMAT_R8G8B8A8_UNORM || f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_R8G8B8A8_TYPELESS; }

static XrResult XRAPI_CALL xrEndFrameImpl(XrSession h, const XrFrameEndInfo *fi);
static XrResult XRAPI_CALL xrEndFrame_(XrSession h, const XrFrameEndInfo *fi) {
    int64_t endT0 = qpc_ns();
    XrResult r = xrEndFrameImpl(h, fi);
    endSum += qpc_ns() - endT0;
    return r;
}
static XrResult XRAPI_CALL xrEndFrameImpl(XrSession h, const XrFrameEndInfo *fi) {
    Session *s = (Session *)h;
    if (!s || !s->dev || !s->ctx || !fi) return XR_ERROR_HANDLE_INVALID;
    const XrCompositionLayerProjection *proj = NULL;
    for (uint32_t i = 0; i < fi->layerCount; i++)
        if (fi->layers[i] && fi->layers[i]->type == XR_TYPE_COMPOSITION_LAYER_PROJECTION) { proj = (const XrCompositionLayerProjection *)fi->layers[i]; break; }
    static int layersLogged;
    if (layersLogged < 3 && fi->layerCount) {
        layersLogged++;
        for (uint32_t i = 0; i < fi->layerCount; i++) if (fi->layers[i]) logmsg("endframe layer %u type %d flags 0x%llx", i, fi->layers[i]->type, (unsigned long long)fi->layers[i]->layerFlags);
        if (proj) for (int e = 0; e < 2 && e < (int)proj->viewCount; e++) {
            const XrSwapchainSubImage *si = &proj->views[e].subImage;
            logmsg("  view %d sc %p rect %d,%d %dx%d array %u", e, (void *)si->swapchain, si->imageRect.offset.x, si->imageRect.offset.y,
                   si->imageRect.extent.width, si->imageRect.extent.height, si->imageArrayIndex);
        }
    }
    if (!proj || proj->viewCount < 2) return XR_SUCCESS;   // ponytail: quad/overlay layers are ignored

    Swapchain *sc[2] = { find_sc(proj->views[0].subImage.swapchain), find_sc(proj->views[1].subImage.swapchain) };
    if (!sc[0] || !sc[1]) {
        static int warned_sc; if (!warned_sc++) logmsg("xrEndFrame: invalid swapchain handle");
        return XR_ERROR_HANDLE_INVALID;
    }
    if (sc[0]->released < 0 || sc[1]->released < 0) return XR_SUCCESS;

    uint64_t w = (uint64_t)proj->views[0].subImage.imageRect.extent.width, hgt = (uint64_t)proj->views[0].subImage.imageRect.extent.height;
    if (!w || !hgt || (uint64_t)proj->views[1].subImage.imageRect.extent.width != w || (uint64_t)proj->views[1].subImage.imageRect.extent.height != hgt ||
        2ULL * w * hgt * 4ULL > (uint64_t)VR4_FRAME_MAX) {
        static int warned; if (!warned++) logmsg("unsupported eye rects %llux%llu / %dx%d", (unsigned long long)w, (unsigned long long)hgt, proj->views[1].subImage.imageRect.extent.width, proj->views[1].subImage.imageRect.extent.height);
        return XR_SUCCESS;
    }

    DXGI_FORMAT fmt = sc[0]->fmt;
    if (sc[1]->fmt != fmt) {
        static int warned_fmt; if (!warned_fmt++) logmsg("mismatched swapchain formats between eyes: %d vs %d", sc[0]->fmt, sc[1]->fmt);
        return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
    }

    // Bounds checking for rects and array indices against swapchain metadata
    for (int e = 0; e < 2; e++) {
        const XrSwapchainSubImage *si = &proj->views[e].subImage;
        if (si->imageArrayIndex >= sc[e]->array) {
            static int warned_arr; if (!warned_arr++) logmsg("eye %d imageArrayIndex %u >= %u", e, si->imageArrayIndex, sc[e]->array);
            return XR_ERROR_RUNTIME_FAILURE;
        }
        if (si->imageRect.offset.x < 0 || si->imageRect.offset.y < 0 ||
            (uint64_t)si->imageRect.offset.x + w > (uint64_t)sc[e]->w ||
            (uint64_t)si->imageRect.offset.y + hgt > (uint64_t)sc[e]->h) {
            static int warned_rect; if (!warned_rect++) logmsg("eye %d imageRect bounds overflow", e);
            return XR_ERROR_RUNTIME_FAILURE;
        }
    }

    // Reallocate staging textures if dimensions or format changed
    if (!s->staging[0][0] || s->stagingFmt != fmt || s->stagingW != (uint32_t)w || s->stagingH != (uint32_t)hgt) {
        for (int k = 0; k < NSTAGE; k++) {
            s->stagingReady[k] = 0;
            for (int e = 0; e < 2; e++) {
                if (s->staging[k][e]) { ID3D11Texture2D_Release(s->staging[k][e]); s->staging[k][e] = NULL; }
            }
        }
        D3D11_TEXTURE2D_DESC d = {(UINT)w, (UINT)hgt, 1, 1, fmt, {1, 0}, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0};
        for (int k = 0; k < NSTAGE; k++) {
            for (int e = 0; e < 2; e++) {
                if (FAILED(ID3D11Device_CreateTexture2D(s->dev, &d, NULL, &s->staging[k][e]))) {
                    logmsg("staging alloc failed at [%d][%d], rolling back", k, e);
                    for (int rk = 0; rk < NSTAGE; rk++) {
                        s->stagingReady[rk] = 0;
                        for (int re = 0; re < 2; re++)
                            if (s->staging[rk][re]) { ID3D11Texture2D_Release(s->staging[rk][re]); s->staging[rk][re] = NULL; }
                    }
                    s->stagingFmt = 0; s->stagingW = s->stagingH = 0;
                    return XR_SUCCESS;
                }
            }
        }
        s->stagingFmt = fmt; s->stagingW = (uint32_t)w; s->stagingH = (uint32_t)hgt;
    }

    // Ring of NSTAGE readback slots: D3DMetal GPU latency is >1 frame, so a 2-slot flip overwrote unread copies and
    // published only ~every other frame (30 fps at 72). Copy into a free slot (or the oldest), publish the newest done.
    int curStage = 0;
    for (int k = 1; k < NSTAGE; k++)
        if (s->stagingReady[k] < s->stagingReady[curStage]) curStage = k;   // free (0) or oldest pending
    for (int e = 0; e < 2; e++) {
        const XrSwapchainSubImage *si = &proj->views[e].subImage;
        D3D11_BOX box = {(UINT)si->imageRect.offset.x, (UINT)si->imageRect.offset.y, 0, 0, 0, 1};
        box.right = box.left + (UINT)w; box.bottom = box.top + (UINT)hgt;
        ID3D11DeviceContext_CopySubresourceRegion(s->ctx, (ID3D11Resource *)s->staging[curStage][e], 0, 0, 0, 0,
            (ID3D11Resource *)sc[e]->img[sc[e]->released], si->imageArrayIndex * sc[e]->mips, &box);

        s->stagingPose[curStage][e] = (VR4Pose){proj->views[e].pose.position.x, proj->views[e].pose.position.y, proj->views[e].pose.position.z,
                                               proj->views[e].pose.orientation.x, proj->views[e].pose.orientation.y, proj->views[e].pose.orientation.z, proj->views[e].pose.orientation.w};
    }
    s->stagingDisplayTime[curStage] = (uint64_t)fi->displayTime;
    s->stagingReady[curStage] = ++s->stagingCounter;

    // GPU completes copies in submit order: try pending slots newest-first, publish the first one whose Map succeeds.
    int order[NSTAGE], n = 0;
    for (int k = 0; k < NSTAGE; k++) if (s->stagingReady[k]) order[n++] = k;
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && s->stagingReady[order[j]] > s->stagingReady[order[j - 1]]; j--) {
        int t = order[j]; order[j] = order[j - 1]; order[j - 1] = t;
    }
    int readStage = -1;
    D3D11_MAPPED_SUBRESOURCE m[2];
    for (int i = 0; i < n && readStage < 0; i++) {
        int k = order[i];
        if (FAILED(ID3D11DeviceContext_Map(s->ctx, (ID3D11Resource *)s->staging[k][0], 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m[0]))) continue;
        if (FAILED(ID3D11DeviceContext_Map(s->ctx, (ID3D11Resource *)s->staging[k][1], 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m[1]))) {
            ID3D11DeviceContext_Unmap(s->ctx, (ID3D11Resource *)s->staging[k][0], 0);
            continue;
        }
        readStage = k;
    }
    if (readStage < 0) return XR_SUCCESS;   // nothing finished yet; never block the game thread
    for (int k = 0; k < NSTAGE; k++)         // older pending copies are stale now
        if (s->stagingReady[k] && s->stagingReady[k] < s->stagingReady[readStage]) s->stagingReady[k] = 0;
    int isRgbaFmt = is_rgba(fmt);

    int64_t copyT0 = qpc_ns();
    uint32_t buf = (shm->frame_seq + 1) % 2;
    uint8_t *dst = vr4_frame(shm, buf);
    for (int e = 0; e < 2; e++) {
        for (uint32_t y = 0; y < (uint32_t)hgt; y++) {
            const uint32_t *src = (const uint32_t *)((const uint8_t *)m[e].pData + y * m[e].RowPitch);
            uint32_t *out = (uint32_t *)(dst + (y * 2 * (uint32_t)w + e * (uint32_t)w) * 4);
            memcpy(out, src, (size_t)w * 4);   // channel order fixed up on the Mac (frame_rgba)
        }
        ID3D11DeviceContext_Unmap(s->ctx, (ID3D11Resource *)s->staging[readStage][e], 0);
        shm->frame_eye_pose[buf][e] = s->stagingPose[readStage][e];
    }
    s->stagingReady[readStage] = 0;
    shm->frame_w[buf] = 2 * (uint32_t)w; shm->frame_h[buf] = (uint32_t)hgt; shm->frame_rgba[buf] = (uint32_t)isRgbaFmt;
    shm->frame_time_ns[buf] = s->stagingDisplayTime[readStage];
    copySum += qpc_ns() - copyT0;
    shm->runtime_heartbeat_ns = (uint64_t)qpc_ns();
    vr4_fence();
    shm->frame_seq++;
    return XR_SUCCESS;
}


// ---------------------------------------------------------------- swapchains
static const int64_t formats[] = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_B8G8R8A8_UNORM};
static XrResult XRAPI_CALL xrEnumerateSwapchainFormats_(XrSession h, uint32_t cap, uint32_t *n, int64_t *f) {
    (void)h; FILL_ARRAY(cap, n, f, 4, f[i_] = formats[i_]); return XR_SUCCESS;
}
static DXGI_FORMAT typeless(DXGI_FORMAT f) {
    return f == DXGI_FORMAT_R8G8B8A8_UNORM || f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ? DXGI_FORMAT_R8G8B8A8_TYPELESS : DXGI_FORMAT_B8G8R8A8_TYPELESS;
}
static XrResult XRAPI_CALL xrCreateSwapchain_(XrSession h, const XrSwapchainCreateInfo *ci, XrSwapchain *out) {
    Session *s = (Session *)h;
    int ok = 0;
    for (int i = 0; i < 4; i++) ok |= formats[i] == ci->format;
    if (!ok) { logmsg("swapchain format %lld unsupported", (long long)ci->format); return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED; }
    if (ci->sampleCount > 1) return XR_ERROR_FEATURE_UNSUPPORTED;
    logmsg("swapchain fmt %lld %ux%u array %u mips %u faces %u usage 0x%llx flags 0x%llx", (long long)ci->format, ci->width, ci->height,
           ci->arraySize, ci->mipCount, ci->faceCount, (unsigned long long)ci->usageFlags, (unsigned long long)ci->createFlags);
    Swapchain *sc = calloc(1, sizeof *sc);
    sc->fmt = typeless((DXGI_FORMAT)ci->format); sc->w = ci->width; sc->h = ci->height; sc->array = ci->arraySize; sc->mips = ci->mipCount ? ci->mipCount : 1; sc->released = -1;
    sc->count = ci->createFlags & XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT ? 1 : 3;
    UINT bind = D3D11_BIND_SHADER_RESOURCE;
    if (ci->usageFlags & XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT) bind |= D3D11_BIND_RENDER_TARGET;
    if (ci->usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT) bind |= D3D11_BIND_UNORDERED_ACCESS;
    D3D11_TEXTURE2D_DESC d = {ci->width, ci->height, ci->mipCount ? ci->mipCount : 1, ci->arraySize ? ci->arraySize : 1, sc->fmt, {1, 0},
                              D3D11_USAGE_DEFAULT, bind, 0, 0};
    for (int i = 0; i < sc->count; i++)
        if (FAILED(ID3D11Device_CreateTexture2D(s->dev, &d, NULL, &sc->img[i]))) { logmsg("CreateTexture2D failed"); free(sc); return XR_ERROR_RUNTIME_FAILURE; }
    *out = (XrSwapchain)sc;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroySwapchain_(XrSwapchain h) {
    Swapchain *sc = find_sc(h);
    for (int i = 0; i < sc->count; i++) ID3D11Texture2D_Release(sc->img[i]);
    free(sc); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateSwapchainImages_(XrSwapchain h, uint32_t cap, uint32_t *n, XrSwapchainImageBaseHeader *imgs) {
    Swapchain *sc = find_sc(h);
    XrSwapchainImageD3D11KHR *d = (XrSwapchainImageD3D11KHR *)imgs;
    FILL_ARRAY(cap, n, d, (uint32_t)sc->count, d[i_].texture = sc->img[i_]);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrAcquireSwapchainImage_(XrSwapchain h, const XrSwapchainImageAcquireInfo *ai, uint32_t *idx) {
    (void)ai; Swapchain *sc = find_sc(h);
    sc->acquired = (sc->acquired + 1) % sc->count; *idx = sc->acquired; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrWaitSwapchainImage_(XrSwapchain h, const XrSwapchainImageWaitInfo *wi) { (void)h; (void)wi; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrReleaseSwapchainImage_(XrSwapchain h, const XrSwapchainImageReleaseInfo *ri) {
    (void)ri; Swapchain *sc = find_sc(h); sc->released = sc->acquired; return XR_SUCCESS;
}

// ---------------------------------------------------------------- actions
static XrResult XRAPI_CALL xrCreateActionSet_(XrInstance i, const XrActionSetCreateInfo *ci, XrActionSet *out) {
    (void)i; ActionSet *s = calloc(1, sizeof *s); snprintf(s->name, 64, "%s", ci->actionSetName); *out = (XrActionSet)s; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroyActionSet_(XrActionSet s) { free(s); return XR_SUCCESS; }
static XrResult XRAPI_CALL xrCreateAction_(XrActionSet set, const XrActionCreateInfo *ci, XrAction *out) {
    (void)set;
    Action *a = calloc(1, sizeof *a);
    a->type = ci->actionType; snprintf(a->name, 64, "%s", ci->actionName);
    a->nsub = ci->countSubactionPaths < 8 ? (int)ci->countSubactionPaths : 8;
    for (int i = 0; i < a->nsub; i++) a->sub[i] = ci->subactionPaths[i];
    *out = (XrAction)a; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroyAction_(XrAction a) { (void)a; return XR_SUCCESS; }   // may still be referenced by suggestions
static XrResult XRAPI_CALL xrSuggestInteractionProfileBindings_(XrInstance i, const XrInteractionProfileSuggestedBinding *sb) {
    (void)i;
    for (uint32_t k = 0; k < sb->countSuggestedBindings && nsugg < 1024; k++)
        sugg[nsugg++] = (Suggestion){sb->interactionProfile, (Action *)sb->suggestedBindings[k].action, sb->suggestedBindings[k].binding};
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrAttachSessionActionSets_(XrSession h, const XrSessionActionSetsAttachInfo *ai) {
    Session *s = (Session *)h; (void)ai;
    XrPath touch = intern("/interaction_profiles/oculus/touch_controller");
    s->profile = nsugg ? sugg[0].profile : touch;
    for (int k = 0; k < nsugg; k++) if (sugg[k].profile == touch) s->profile = touch;
    for (int k = 0; k < nsugg; k++) {
        Action *a = sugg[k].action;
        if (sugg[k].profile != s->profile || a->nb >= 16) continue;
        if (parse_binding(pstr(sugg[k].binding), &a->b[a->nb])) a->nb++;
    }
    logmsg("attached %d bindings, profile %s", nsugg, pstr(s->profile));
    if (evTail - evHead >= 64) evHead = evTail - 63;
    XrEventDataInteractionProfileChanged *e = (XrEventDataInteractionProfileChanged *)&events[evTail++ % 64];
    memset(e, 0, sizeof(XrEventDataBuffer));
    e->type = XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED; e->session = h;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetCurrentInteractionProfile_(XrSession h, XrPath user, XrInteractionProfileState *st) {
    Session *s = (Session *)h;
    const char *u = pstr(user);
    st->interactionProfile = !strcmp(u, "/user/hand/left") || !strcmp(u, "/user/hand/right") ? s->profile : XR_NULL_PATH;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrSyncActions_(XrSession h, const XrActionsSyncInfo *si) {
    (void)si; Session *s = (Session *)h;
    read_tracking();
    syncGen++;
    return s->focused ? XR_SUCCESS : XR_SESSION_NOT_FOCUSED;
}
static int active(Session *s) { return s->focused && !shm->input_blocked; }
static float action_value(Session *s, Action *a, XrPath sub, XrVector2f *v2, int *bound) {
    float best = 0; *bound = 0;
    if (v2) *v2 = (XrVector2f){0, 0};
    for (int i = 0; i < a->nb; i++) {
        Binding *b = &a->b[i];
        if (!sub_matches(sub, b->hand)) continue;
        *bound = 1;
        if (!active(s)) continue;
        const VR4Hand *hh = &track.hand[b->hand];
        if (v2 && (!strcmp(b->comp, "thumbstick") || !strcmp(b->comp, "trackpad"))) {
            if (fabsf(hh->stick_x) + fabsf(hh->stick_y) > fabsf(v2->x) + fabsf(v2->y)) *v2 = (XrVector2f){hh->stick_x, hh->stick_y};
            continue;
        }
        float v = comp_value(hh, b->comp);
        if (fabsf(v) > fabsf(best)) best = v;
    }
    return best;
}
static int slot(XrPath sub) { const char *p = pstr(sub); return !strcmp(p, "/user/hand/left") ? 0 : !strcmp(p, "/user/hand/right") ? 1 : 2; }
/// changedSinceLastSync: compares this sync's value with the previous sync's, stable across repeated reads.
static int changed(Action *a, XrPath sub, float x, float y) {
    ActionHistory *h = &a->hist[slot(sub)];
    if (h->gen != syncGen) { h->prev[0] = h->cur[0]; h->prev[1] = h->cur[1]; h->cur[0] = x; h->cur[1] = y; h->gen = syncGen; }
    return h->cur[0] != h->prev[0] || h->cur[1] != h->prev[1];
}
static XrResult XRAPI_CALL xrGetActionStateBoolean_(XrSession h, const XrActionStateGetInfo *gi, XrActionStateBoolean *st) {
    int bound; float v = action_value((Session *)h, (Action *)gi->action, gi->subactionPath, NULL, &bound);
    XrBool32 now = v > 0.5f;
    st->changedSinceLastSync = changed((Action *)gi->action, gi->subactionPath, (float)now, 0);
    st->currentState = now; st->isActive = bound; st->lastChangeTime = 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetActionStateFloat_(XrSession h, const XrActionStateGetInfo *gi, XrActionStateFloat *st) {
    int bound; float v = action_value((Session *)h, (Action *)gi->action, gi->subactionPath, NULL, &bound);
    st->changedSinceLastSync = changed((Action *)gi->action, gi->subactionPath, v, 0); st->currentState = v; st->isActive = bound; st->lastChangeTime = 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetActionStateVector2f_(XrSession h, const XrActionStateGetInfo *gi, XrActionStateVector2f *st) {
    int bound; XrVector2f v; action_value((Session *)h, (Action *)gi->action, gi->subactionPath, &v, &bound);
    st->changedSinceLastSync = changed((Action *)gi->action, gi->subactionPath, v.x, v.y);
    st->currentState = v; st->isActive = bound; st->lastChangeTime = 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetActionStatePose_(XrSession h, const XrActionStateGetInfo *gi, XrActionStatePose *st) {
    (void)h; Action *a = (Action *)gi->action;
    st->isActive = XR_FALSE;
    for (int i = 0; i < a->nb; i++)
        if (is_pose_comp(a->b[i].comp) && sub_matches(gi->subactionPath, a->b[i].hand))
            st->isActive = (track.hand[a->b[i].hand].flags & VR4_HAND_ACTIVE) != 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateBoundSourcesForAction_(XrSession h, const XrBoundSourcesForActionEnumerateInfo *ei, uint32_t cap, uint32_t *n, XrPath *out) {
    (void)h; Action *a = (Action *)ei->action;
    char buf[160];
    FILL_ARRAY(cap, n, out, (uint32_t)a->nb, {
        snprintf(buf, sizeof buf, "/user/hand/%s/input/%s", a->b[i_].hand ? "right" : "left", a->b[i_].comp);
        out[i_] = intern(buf); });
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetInputSourceLocalizedName_(XrSession h, const XrInputSourceLocalizedNameGetInfo *gi, uint32_t cap, uint32_t *n, char *buf) {
    (void)h;
    const char *s = pstr(gi->sourcePath); uint32_t len = (uint32_t)strlen(s) + 1;
    *n = len;
    if (!cap) return XR_SUCCESS;
    if (cap < len) return XR_ERROR_SIZE_INSUFFICIENT;
    memcpy(buf, s, len); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrApplyHapticFeedback_(XrSession h, const XrHapticActionInfo *hi, const XrHapticBaseHeader *hb) {
    (void)h;
    if (hb->type != XR_TYPE_HAPTIC_VIBRATION) return XR_SUCCESS;
    const XrHapticVibration *v = (const XrHapticVibration *)hb;
    Action *a = (Action *)hi->action;
    for (int i = 0; i < a->nb; i++) if (sub_matches(hi->subactionPath, a->b[i].hand)) {
        float dur = v->duration <= 0 ? 0.02f : (float)v->duration / 1e9f;
        shm->haptic = (VR4Haptics){(uint8_t)a->b[i].hand, v->amplitude, dur, v->frequency > 0 ? v->frequency : 0};
        vr4_fence(); shm->haptic_seq++;
    }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrStopHapticFeedback_(XrSession h, const XrHapticActionInfo *hi) {
    (void)h;
    Action *a = (Action *)hi->action;
    for (int i = 0; i < a->nb; i++) if (sub_matches(hi->subactionPath, a->b[i].hand)) {
        shm->haptic = (VR4Haptics){(uint8_t)a->b[i].hand, 0.0f, 0.0f, 0.0f};
        vr4_fence(); shm->haptic_seq++;
    }
    return XR_SUCCESS;
}

// ---------------------------------------------------------------- compatibility extensions
static float current_hz(void) { return shm && shm->fps > 0 ? shm->fps : 72; }
static XrResult XRAPI_CALL xrEnumerateDisplayRefreshRatesFB_(XrSession h, uint32_t cap, uint32_t *n, float *rates) {
    (void)h; FILL_ARRAY(cap, n, rates, 1, rates[i_] = current_hz()); return XR_SUCCESS;   // the rate MacVR negotiated with the headset
}
static XrResult XRAPI_CALL xrGetDisplayRefreshRateFB_(XrSession h, float *rate) { (void)h; *rate = current_hz(); return XR_SUCCESS; }
static XrResult XRAPI_CALL xrRequestDisplayRefreshRateFB_(XrSession h, float rate) {
    (void)h; return rate == 0 || fabsf(rate - current_hz()) < 0.5f ? XR_SUCCESS : XR_ERROR_DISPLAY_REFRESH_RATE_UNSUPPORTED_FB;
}
static XrResult XRAPI_CALL xrGetVisibilityMaskKHR_(XrSession h, XrViewConfigurationType t, uint32_t view, XrVisibilityMaskTypeKHR mt, XrVisibilityMaskKHR *m) {
    (void)h; (void)t; (void)view; (void)mt;   // no hidden-area mask: the whole eye buffer is visible
    m->vertexCountOutput = 0; m->indexCountOutput = 0; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateColorSpacesFB_(XrSession h, uint32_t cap, uint32_t *n, XrColorSpaceFB *cs) {
    (void)h;
    static const XrColorSpaceFB supported[] = {XR_COLOR_SPACE_QUEST_FB, XR_COLOR_SPACE_REC709_FB, XR_COLOR_SPACE_UNMANAGED_FB};
    FILL_ARRAY(cap, n, cs, 3, cs[i_] = supported[i_]);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrSetColorSpaceFB_(XrSession h, const XrColorSpaceFB cs) {
    (void)h;
    if (cs < XR_COLOR_SPACE_UNMANAGED_FB || cs > XR_COLOR_SPACE_ADOBE_RGB_FB) return XR_ERROR_COLOR_SPACE_UNSUPPORTED_FB;
    return XR_SUCCESS;
}

// ---------------------------------------------------------------- dispatch
static XrResult XRAPI_CALL xrGetInstanceProcAddr_(XrInstance inst, const char *name, PFN_xrVoidFunction *fn);
static const struct { const char *name; PFN_xrVoidFunction fn; } table[] = {
#define F(n) {#n, (PFN_xrVoidFunction)n##_},
    F(xrGetInstanceProcAddr) F(xrEnumerateApiLayerProperties) F(xrEnumerateInstanceExtensionProperties) F(xrCreateInstance)
    F(xrDestroyInstance) F(xrGetInstanceProperties) F(xrPollEvent) F(xrResultToString) F(xrStructureTypeToString)
    F(xrStringToPath) F(xrPathToString) F(xrGetSystem) F(xrGetSystemProperties) F(xrEnumerateEnvironmentBlendModes)
    F(xrEnumerateViewConfigurations) F(xrGetViewConfigurationProperties) F(xrEnumerateViewConfigurationViews)
    F(xrGetD3D11GraphicsRequirementsKHR) F(xrConvertWin32PerformanceCounterToTimeKHR) F(xrConvertTimeToWin32PerformanceCounterKHR)
    F(xrCreateSession) F(xrDestroySession) F(xrBeginSession) F(xrEndSession) F(xrRequestExitSession)
    F(xrEnumerateReferenceSpaces) F(xrCreateReferenceSpace) F(xrGetReferenceSpaceBoundsRect) F(xrCreateActionSpace)
    F(xrLocateSpace) F(xrDestroySpace) F(xrWaitFrame) F(xrBeginFrame) F(xrEndFrame) F(xrLocateViews)
    F(xrEnumerateSwapchainFormats) F(xrCreateSwapchain) F(xrDestroySwapchain) F(xrEnumerateSwapchainImages)
    F(xrAcquireSwapchainImage) F(xrWaitSwapchainImage) F(xrReleaseSwapchainImage)
    F(xrCreateActionSet) F(xrDestroyActionSet) F(xrCreateAction) F(xrDestroyAction) F(xrSuggestInteractionProfileBindings)
    F(xrAttachSessionActionSets) F(xrGetCurrentInteractionProfile) F(xrSyncActions) F(xrGetActionStateBoolean)
    F(xrGetActionStateFloat) F(xrGetActionStateVector2f) F(xrGetActionStatePose) F(xrEnumerateBoundSourcesForAction)
    F(xrGetInputSourceLocalizedName) F(xrApplyHapticFeedback) F(xrStopHapticFeedback)
    F(xrEnumerateDisplayRefreshRatesFB) F(xrGetDisplayRefreshRateFB) F(xrRequestDisplayRefreshRateFB) F(xrGetVisibilityMaskKHR)
    F(xrEnumerateColorSpacesFB) F(xrSetColorSpaceFB)
#undef F
};
static XrResult XRAPI_CALL xrGetInstanceProcAddr_(XrInstance inst, const char *name, PFN_xrVoidFunction *fn) {
    (void)inst;
    for (size_t i = 0; i < sizeof table / sizeof table[0]; i++)
        if (!strcmp(name, table[i].name)) { *fn = table[i].fn; return XR_SUCCESS; }
    *fn = NULL;
    logmsg("unsupported function %s", name);
    return XR_ERROR_FUNCTION_UNSUPPORTED;
}

EXPORT XrResult XRAPI_CALL xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo *li, XrNegotiateRuntimeRequest *rr) {
    if (!li || !rr || li->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO || rr->structType != XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST ||
        li->minInterfaceVersion > XR_CURRENT_LOADER_RUNTIME_VERSION || li->maxInterfaceVersion < XR_CURRENT_LOADER_RUNTIME_VERSION)
        return XR_ERROR_INITIALIZATION_FAILED;
    rr->runtimeInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
    rr->runtimeApiVersion = XR_MAKE_VERSION(1, 0, 0);
    rr->getInstanceProcAddr = xrGetInstanceProcAddr_;
    return XR_SUCCESS;
}
