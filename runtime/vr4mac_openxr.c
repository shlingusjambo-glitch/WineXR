// VR4Mac OpenXR runtime (WineXR): a Windows DLL loaded by games (directly, or via OpenComposite for OpenVR games)
// inside Wine. Poses/input come from the Mac app through shared memory (common/vr4mac.h); rendered eye
// images (plus quad/cylinder layers, composited on the GPU) are copied back the same way and streamed to the
// Quest by the Mac app. D3D11, or D3D12 (projection layers only).
#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <d3d12.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#define XR_USE_GRAPHICS_API_D3D12
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include <openxr/openxr_reflection.h>
#include "../common/vr4mac.h"

#define EXPORT __declspec(dllexport)
#define NSTAGE 4   // readback ring depth (GPU latency under D3DMetal is >1 frame)

static VR4Shm *shm;
static FILE *logfile;
static char shmPath[MAX_PATH], logPath[MAX_PATH + 8];
static void init_paths(void) {   // VR4_SHM=<windows path> points tests at their own block (log goes next to it)
    if (*shmPath) return;
    if (GetEnvironmentVariableA("VR4_SHM", shmPath, MAX_PATH) - 1u < MAX_PATH - 1u) snprintf(logPath, sizeof logPath, "%s.log", shmPath);
    else { strcpy(shmPath, VR4_SHM_PATH_WIN); strcpy(logPath, "Z:\\tmp\\vr4mac\\runtime.log"); }
}
static void logmsg(const char *fmt, ...) {
    init_paths();
    if (!logfile) logfile = fopen(logPath, "a");
    if (!logfile) return;
    va_list a; va_start(a, fmt); vfprintf(logfile, fmt, a); va_end(a); fputc('\n', logfile); fflush(logfile);
}
static void logonce(const char *fmt, ...) {   // each distinct message once per process (unsupported calls, layers, bindings)
    static char *seen[256]; static int n;
    char m[512]; va_list a; va_start(a, fmt); vsnprintf(m, sizeof m, fmt, a); va_end(a);
    for (int i = 0; i < n; i++) if (!strcmp(seen[i], m)) return;
    if (n == 256) return;
    seen[n++] = _strdup(m);
    logmsg("%s", m);
}
static void rel(void *com) { if (com) IUnknown_Release((IUnknown *)com); }

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
static inline V vcross(V a, V b) { return (V){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline V vadd(V a, V b) { return (V){a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V vsub(V a, V b) { return (V){a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V vscale(V a, float s) { return (V){a.x * s, a.y * s, a.z * s}; }
static inline float vlen(V a) { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }
static inline float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
static XrPosef pinv(XrPosef a) {
    Q c = qconj(a.orientation); V t = qrot(c, a.position);
    return (XrPosef){c, {-t.x, -t.y, -t.z}};
}
static XrPosef xp(VR4Pose p) {   // shared-memory (raw stage metres) -> app units
    float ws = current_world_scale();
    return (XrPosef){{p.qx, p.qy, p.qz, p.qw}, {p.px / ws, p.py / ws, p.pz / ws}};
}
static VR4Pose raw_pose(XrPosef p) {   // app units -> raw stage metres
    float ws = current_world_scale();
    return (VR4Pose){p.position.x * ws, p.position.y * ws, p.position.z * ws, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
}
static const XrPosef IDENT = {{0, 0, 0, 1}, {0, 0, 0}};
/// Finite-difference velocity a -> b over dt seconds (clamped against tracking glitches).
static void pose_velocity(XrPosef a, XrPosef b, float dt, V *lin, V *ang) {
    *lin = vscale(vsub(b.position, a.position), 1 / dt);
    Q r = qmul(b.orientation, qconj(a.orientation));
    if (r.w < 0) r = (Q){-r.x, -r.y, -r.z, -r.w};
    float sinHalf = sqrtf(fmaxf(0, 1 - r.w * r.w)), angle = 2 * acosf(fminf(r.w, 1));
    *ang = sinHalf > 1e-4f && angle > 1e-4f ? vscale((V){r.x, r.y, r.z}, angle / dt / sinHalf) : (V){0, 0, 0};
    float l = vlen(*lin), w = vlen(*ang);
    if (l > 40) *lin = vscale(*lin, 40 / l);
    if (w > 100) *ang = vscale(*ang, 100 / w);
}

// ---------------------------------------------------------------- paths
static char *paths[4096]; static int npaths;
static XrPath intern(const char *s) {
    for (int i = 0; i < npaths; i++) if (!strcmp(paths[i], s)) return i + 1;
    if (npaths == 4096) return XR_NULL_PATH;
    paths[npaths] = _strdup(s);
    return ++npaths;
}
static const char *pstr(XrPath p) { return p >= 1 && p <= (XrPath)npaths ? paths[p - 1] : ""; }
static XrPath handPath[2];   // /user/hand/left, /user/hand/right

// ---------------------------------------------------------------- objects
typedef struct { XrPath path; int8_t hand; uint8_t src, hi; } Binding;   // hi: from the hand-interaction profile
typedef struct { uint32_t gen; float cur[2], prev[2]; } ActionHistory;
typedef struct Action { XrActionType type; char name[64]; XrPath sub[8]; int nsub; Binding b[32]; int nb; ActionHistory hist[3]; } Action;
typedef struct { char name[64]; } ActionSet;
typedef struct { XrPath profile; Action *action; XrPath binding; } Suggestion;
typedef struct {
    XrSession session; int ref; XrReferenceSpaceType type; Action *action; XrPath sub; XrPosef offset;
} Space;
typedef struct {
    ID3D11Texture2D *img[3]; ID3D12Resource *img12[3]; int count, acquired, released; DXGI_FORMAT fmt; uint32_t w, h, array, mips;
} Swapchain;
typedef struct { int hand, unobstructed; } HandTracker;
typedef struct {
    ID3D11Device *dev; ID3D11DeviceContext *ctx; int running, focused, exitRequested;
    ID3D11Texture2D *staging[NSTAGE]; DXGI_FORMAT stagingFmt; uint32_t stagingW, stagingH; uint64_t stagingCounter;   // side-by-side readback ring
    uint64_t stagingReady[NSTAGE]; VR4Pose stagingPose[NSTAGE][2]; uint64_t stagingDisplayTime[NSTAGE]; // stagingReady: submit order, 0 = free
    XrPosef localOrigin; V localOriginRaw;
    XrPath profile, handProfile; int handHi[2];   // controller / hand-interaction profile; handHi[h]: hand h uses the latter now
    uint64_t lastPublished;   // displayTime of the last actually-published frame (duplicate-submit skip)
    // layer compositor, created with the first quad/cylinder layer (compOk -1: shaders unavailable)
    int compOk; ID3D11VertexShader *vs; ID3D11PixelShader *ps; ID3D11InputLayout *il; ID3D11Buffer *vb, *cb;
    ID3D11SamplerState *smp; ID3D11BlendState *blend[2]; ID3D11RasterizerState *rs;
    ID3D11Texture2D *target; ID3D11RenderTargetView *rtv; DXGI_FORMAT targetFmt; uint32_t targetW, targetH;
    // D3D12 sessions: the readback ring is buffers filled by our own command lists on the game's queue
    ID3D12Device *dev12; ID3D12CommandQueue *queue; ID3D12Fence *fence; HANDLE fenceEvent; uint64_t fenceValue;
    ID3D12CommandAllocator *alloc[NSTAGE]; ID3D12GraphicsCommandList *cl[NSTAGE]; ID3D12Resource *rb[NSTAGE]; uint64_t rbFence[NSTAGE]; uint32_t rbPitch;
} Session;


static Suggestion sugg[1024]; static int nsugg;
static VR4Tracking track;           // input snapshot (hands/buttons), refreshed by xrSyncActions
static VR4HandJoints joints[2], prevJoints[2]; static uint64_t jointsNs, prevJointsNs;   // hand-tracking snapshot (+ previous, for velocities)
static VR4Tracking frameTrack;      // head/eyes of the latest xrWaitFrame
static VR4Tracking frameRing[4]; static int frameRingN;   // recent xrWaitFrame snapshots, for pipelined apps
static const VR4Tracking noHeadset = {0, {0, 1.6f, 0, 0, 0, 0, 1},   // until the first tracking sample
    {{{-0.032f, 1.6f, 0, 0, 0, 0, 1}, {-0.8f, 0.8f, 0.8f, -0.8f}}, {{0.032f, 1.6f, 0, 0, 0, 0, 1}, {-0.8f, 0.8f, 0.8f, -0.8f}}}};
static uint32_t lastSeq;
static int64_t lastSampleNs;        // when the latest fresh tracking sample was read (QPC ns)
static uint32_t syncGen;   // bumped by xrSyncActions
static XrEventDataBuffer events[64]; static int evHead, evTail;
static Session *theSession;
static char appName[128];
static LARGE_INTEGER qpf;
static int64_t qpcOffsetNs;         // XrTime - QPC time

static int64_t qpc_ns(void) { LARGE_INTEGER c; QueryPerformanceCounter(&c); return (int64_t)((double)c.QuadPart * 1e9 / (double)qpf.QuadPart); }

static XrEventDataBuffer *new_event(XrStructureType type) {
    if (evTail - evHead >= 64) evHead = evTail - 63;
    XrEventDataBuffer *e = &events[evTail++ % 64];
    memset(e, 0, sizeof *e); e->type = type;
    return e;
}
static void push_state(XrSessionState st) {
    XrEventDataSessionStateChanged *e = (XrEventDataSessionStateChanged *)new_event(XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED);
    e->session = (XrSession)theSession; e->state = st; e->time = qpc_ns() + qpcOffsetNs;
}

/// Head/eye snapshot handed out by the xrWaitFrame that predicted `t` (falls back to the latest).
static const VR4Tracking *frame_for(XrTime t) {
    for (int i = 0; i < 4; i++) if (frameRing[i].time_ns == (uint64_t)t && t) return &frameRing[i];
    return frameTrack.time_ns ? &frameTrack : &noHeadset;
}
static void update_velocities(const VR4Tracking *t);
static int read_tracking(void) {   // seqlock read of the Mac app's latest tracking sample and hand joints
    for (int tries = 0; tries < 100; tries++) {
        uint32_t s1 = shm->track_seq; vr4_fence();
        if (s1 & 1) continue;
        if (s1 == lastSeq) return 0;   // nothing new (xrWaitFrame polls this): skip the copy
        VR4Tracking t = shm->track; VR4HandJoints j[2]; memcpy(j, shm->hand_joints, sizeof j); vr4_fence();
        if (shm->track_seq != s1) continue;
        track = t; lastSeq = s1; lastSampleNs = qpc_ns();
        memcpy(prevJoints, joints, sizeof joints); prevJointsNs = jointsNs;
        memcpy(joints, j, sizeof joints); jointsNs = t.time_ns;
        update_velocities(&track);
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------- input mapping
// Every suggested binding resolves once (at attach) to one of these sources on the Quest's Touch controllers (or
// tracked hands). Other controllers' profiles are remapped: an Index/Vive "a"/"b" on the left hand is X/Y, a Vive
// trackpad is the thumbstick, a missing right-hand menu button is B.
enum { S_NONE, S_TRIGGER, S_TRIGGER_CLICK, S_TRIGGER_TOUCH, S_SQUEEZE, S_SQUEEZE_CLICK, S_STICK, S_STICK_X, S_STICK_Y,
       S_STICK_CLICK, S_STICK_TOUCH, S_THUMB_TOUCH, S_THUMB_NEAR, S_LOWER, S_UPPER, S_MENU, S_SYSTEM, S_PAD_CLICK,
       S_PINCH_READY, S_GRASP, S_GRASP_READY, S_HAPTIC, S_AIM, S_GRIP, S_PINCH_POSE, S_POKE_POSE };
#define IS_POSE(s) ((s) >= S_AIM)
enum { P_STICK = 1, P_FACE = 2, P_HAND = 4 };   // profile has a thumbstick / face buttons / is the hand-interaction profile
static const struct { const char *path; int flags; } profiles[] = {   // controller profiles in preference order
    {"/interaction_profiles/oculus/touch_controller", P_STICK | P_FACE},
    {"/interaction_profiles/meta/touch_controller_plus", P_STICK | P_FACE},
    {"/interaction_profiles/meta/touch_plus_controller", P_STICK | P_FACE},
    {"/interaction_profiles/facebook/touch_controller_pro", P_STICK | P_FACE},
    {"/interaction_profiles/meta/touch_pro_controller", P_STICK | P_FACE},
    {"/interaction_profiles/valve/index_controller", P_STICK | P_FACE},
    {"/interaction_profiles/hp/mixed_reality_controller", P_STICK | P_FACE},
    {"/interaction_profiles/htc/vive_cosmos_controller", P_STICK | P_FACE},
    {"/interaction_profiles/microsoft/motion_controller", P_STICK},
    {"/interaction_profiles/htc/vive_controller", 0},
    {"/interaction_profiles/khr/simple_controller", 0},
    {"/interaction_profiles/ext/hand_interaction_ext", P_HAND},
};
#define NPROF (int)(sizeof profiles / sizeof *profiles)
static int profile_index(XrPath p) { for (int i = 0; i < NPROF; i++) if (!strcmp(pstr(p), profiles[i].path)) return i; return -1; }
static int profile_flags(XrPath p) { int i = profile_index(p); return i < 0 ? P_STICK | P_FACE : profiles[i].flags; }

/// Component path after /input/ ("trigger/value", "a/click", "thumbstick") -> source, for `hand` of a profile with `flags`.
static int resolve(const char *c, int flags, int hand, int output) {
    if (output) return S_HAPTIC;
    const char *slash = strchr(c, '/'), *comp = slash ? slash + 1 : "";
    size_t n = slash ? (size_t)(slash - c) : strlen(c);
    #define ID(s) (n == sizeof(s) - 1 && !strncmp(c, s, n))
    #define C(s) (!strcmp(comp, s))
    if (ID("trigger")) return C("touch") || C("proximity_fb") || C("curl_meta") || C("curl_fb") ? S_TRIGGER_TOUCH :
                              C("click") ? S_TRIGGER_CLICK : C("slide_meta") || C("slide_fb") ? S_NONE : S_TRIGGER;
    if (ID("trigger_meta")) return S_TRIGGER_TOUCH;   // proximity_meta
    if (ID("select")) return S_TRIGGER_CLICK;
    if (ID("squeeze")) return C("click") ? S_SQUEEZE_CLICK : S_SQUEEZE;
    if (ID("thumbstick") || ID("joystick"))
        return C("x") ? S_STICK_X : C("y") ? S_STICK_Y : C("click") ? S_STICK_CLICK : C("touch") ? S_STICK_TOUCH : S_STICK;
    if (ID("trackpad")) {   // a Vive's only thumb input is its pad; a pad next to a real stick (Index, WMR) stays idle
        if (flags & P_STICK) return C("click") ? S_LOWER : S_NONE;
        return C("x") ? S_STICK_X : C("y") ? S_STICK_Y : C("click") ? S_PAD_CLICK : C("touch") ? S_STICK_TOUCH : C("force") ? S_NONE : S_STICK;
    }
    if (ID("a") || ID("x")) return C("touch") ? S_THUMB_TOUCH : S_LOWER;
    if (ID("b") || ID("y")) return C("touch") ? S_THUMB_TOUCH : S_UPPER;
    if (ID("thumbrest")) return C("touch") ? S_THUMB_TOUCH : S_NONE;
    if (ID("thumb_meta") || ID("thumb_fb")) return S_THUMB_NEAR;
    if (ID("menu")) return hand && !(flags & P_FACE) ? S_UPPER : S_MENU;   // Touch has no right menu button: B
    if (ID("system")) return S_SYSTEM;
    if (ID("pinch_ext") || ID("aim_activate_ext")) return C("pose") ? S_PINCH_POSE : C("ready_ext") ? S_PINCH_READY : S_TRIGGER;
    if (ID("grasp_ext")) return C("ready_ext") ? S_GRASP_READY : S_GRASP;
    if (ID("poke_ext")) return S_POKE_POSE;
    if (ID("aim")) return S_AIM;
    if (ID("grip") || ID("palm_ext") || ID("grip_surface")) return C("pose") ? S_GRIP : S_SQUEEZE;
    return S_NONE;
    #undef ID
    #undef C
}
static int parse_binding(const char *path, int *hand, const char **comp, int *output) {   // "/user/hand/left/input/trigger/value"
    if (!strncmp(path, "/user/hand/left/", 16)) *hand = 0;
    else if (!strncmp(path, "/user/hand/right/", 17)) *hand = 1;
    else return 0;
    const char *c = path + (*hand ? 17 : 16);
    if (!strncmp(c, "input/", 6)) { *comp = c + 6; *output = 0; return 1; }
    if (!strncmp(c, "output/", 7)) { *comp = c + 7; *output = 1; return 1; }
    return 0;
}
static int sub_matches(XrPath sub, int hand) { return sub == XR_NULL_PATH || sub == handPath[hand]; }
static int in_use(const Session *s, const Binding *b) { return b->hi == s->handHi[b->hand]; }   // the hand's current profile

// tracked hands: raw joint geometry (metres, independent of world scale)
static float jdist(int h, int a, int b) {
    const VR4Pose *p = &joints[h].joint[a], *q = &joints[h].joint[b];
    return vlen((V){p->px - q->px, p->py - q->py, p->pz - q->pz});
}
static float pinch_strength(int h, int tip) { return clamp01((0.07f - jdist(h, XR_HAND_JOINT_THUMB_TIP_EXT, tip)) / 0.05f); }   // open 7 cm .. closed 2 cm
static float grasp(int h) {   // middle/ring/little fingertips curled onto the palm
    float d = (jdist(h, 15, 0) + jdist(h, 20, 0) + jdist(h, 25, 0)) / 3;
    return clamp01((0.085f - d) / 0.04f);
}
static int hand_tracked(int h) { return joints[h].tracked && (track.hand[h].flags & VR4_HAND_TRACKED); }

static float src_value(int hand, int s) {
    const VR4Hand *h = &track.hand[hand];
    uint32_t b = h->buttons, lower = hand ? VR4_BTN_A : VR4_BTN_X, upper = hand ? VR4_BTN_B : VR4_BTN_Y;
    switch (s) {
        case S_TRIGGER: return h->trigger;
        case S_TRIGGER_CLICK: return h->trigger > 0.5f;
        case S_TRIGGER_TOUCH: return (b & VR4_BTN_TRIGGER_TOUCH) || h->trigger > 0.05f;
        case S_SQUEEZE: return h->squeeze;
        case S_SQUEEZE_CLICK: return h->squeeze > 0.5f;
        case S_STICK_X: return h->stick_x;
        case S_STICK_Y: return h->stick_y;
        case S_STICK_CLICK: return (b & VR4_BTN_STICK_CLICK) != 0;
        case S_STICK_TOUCH: return (b & (VR4_BTN_STICK_TOUCH | VR4_BTN_STICK_CLICK)) != 0;
        case S_THUMB_TOUCH: return (b & VR4_BTN_THUMB_TOUCH) != 0;
        case S_THUMB_NEAR: return (b & (VR4_BTN_THUMB_TOUCH | VR4_BTN_STICK_TOUCH)) != 0;
        case S_LOWER: return (b & lower) != 0;
        case S_UPPER: return (b & upper) != 0;
        case S_MENU: return (b & VR4_BTN_MENU) != 0;
        case S_SYSTEM: return !hand && (b & VR4_BTN_MENU);
        case S_PAD_CLICK: return (b & (VR4_BTN_STICK_CLICK | lower)) != 0;
        case S_PINCH_READY: return hand_tracked(hand) ? (h->flags & VR4_HAND_PINCH_READY) != 0 : (h->flags & VR4_HAND_ACTIVE) != 0;
        case S_GRASP: return hand_tracked(hand) ? grasp(hand) : h->squeeze;
        case S_GRASP_READY: return (h->flags & VR4_HAND_ACTIVE) != 0;
    }
    return 0;
}
static XrPosef hand_pose(int hand, int src, int *valid) {
    const VR4Hand *h = &track.hand[hand];
    *valid = (h->flags & VR4_HAND_POSE_VALID) != 0;
    if (hand_tracked(hand) && src == S_POKE_POSE) return xp(joints[hand].joint[XR_HAND_JOINT_INDEX_TIP_EXT]);
    if (hand_tracked(hand) && src == S_PINCH_POSE) {   // between thumb and index tips, pointing along the aim ray
        XrPosef p = xp(h->aim), a = xp(joints[hand].joint[XR_HAND_JOINT_THUMB_TIP_EXT]), b = xp(joints[hand].joint[XR_HAND_JOINT_INDEX_TIP_EXT]);
        p.position = vscale(vadd(a.position, b.position), 0.5f);
        return p;
    }
    return xp(src == S_GRIP ? h->grip : h->aim);
}

// ---------------------------------------------------------------- spaces
typedef enum {
    TRACK_SLOT_HEAD = 0,
    TRACK_SLOT_HAND_LEFT_AIM,
    TRACK_SLOT_HAND_LEFT_GRIP,
    TRACK_SLOT_HAND_RIGHT_AIM,
    TRACK_SLOT_HAND_RIGHT_GRIP,
    TRACK_SLOT_COUNT
} TrackSlot;

typedef struct {
    XrPosef lastPose;
    uint64_t lastTimeNs;
    V linearVel;
    V angularVel;
    int valid;
} VelocityTracker;

static VelocityTracker velTrackers[TRACK_SLOT_COUNT];

static void update_velocities(const VR4Tracking *t) {
    if (!t || t->time_ns == 0) return;
    for (int slot = 0; slot < TRACK_SLOT_COUNT; slot++) {
        VelocityTracker *vt = &velTrackers[slot];
        int h = (slot - 1) / 2, slotValid = slot == TRACK_SLOT_HEAD || (t->hand[h].flags & VR4_HAND_POSE_VALID);
        VR4Pose rawPose = slot == TRACK_SLOT_HEAD ? t->head : slot % 2 ? t->hand[h].aim : t->hand[h].grip;
        if (!slotValid) {
            vt->valid = 0;
            vt->lastTimeNs = 0;
            vt->linearVel = (V){0, 0, 0};
            vt->angularVel = (V){0, 0, 0};
            continue;
        }
        XrPosef p = xp(rawPose);
        if (vt->valid && vt->lastTimeNs > 0 && t->time_ns > vt->lastTimeNs) {
            double dt = (double)(t->time_ns - vt->lastTimeNs) * 1e-9;
            if (dt >= 0.002 && dt <= 0.15) {
                V instLinear, instAngular;
                pose_velocity(vt->lastPose, p, (float)dt, &instLinear, &instAngular);
                if (vt->linearVel.x != 0 || vt->linearVel.y != 0 || vt->linearVel.z != 0) {
                    vt->linearVel = vadd(vscale(instLinear, 0.75f), vscale(vt->linearVel, 0.25f));
                    vt->angularVel = vadd(vscale(instAngular, 0.75f), vscale(vt->angularVel, 0.25f));
                } else {
                    vt->linearVel = instLinear;
                    vt->angularVel = instAngular;
                }
            }
        }
        vt->lastPose = p;
        vt->lastTimeNs = t->time_ns;
        vt->valid = 1;
    }
}

static XrPosef current_local_origin(const Session *s) {
    if (!s) return IDENT;
    float ws = current_world_scale();
    XrPosef o = s->localOrigin;
    o.position = (V){s->localOriginRaw.x / ws, s->localOriginRaw.y / ws, s->localOriginRaw.z / ws};
    return o;
}

/// Pose of a space in stage space; VIEW follows the head of `ft` (the frame being located or composited).
static XrPosef space_in_stage_at(Space *s, const VR4Tracking *ft, int *valid) {
    *valid = 1;
    if (s->ref) switch (s->type) {
        case XR_REFERENCE_SPACE_TYPE_VIEW: return pmul(xp(ft->head), s->offset);
        case XR_REFERENCE_SPACE_TYPE_LOCAL: return pmul(current_local_origin((Session *)s->session), s->offset);
        case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR: {
            XrPosef o = current_local_origin((Session *)s->session); o.position.y = 0; return pmul(o, s->offset);
        }
        default: return s->offset;
    }
    for (int i = 0; i < s->action->nb; i++) {
        Binding *b = &s->action->b[i];
        if (IS_POSE(b->src) && in_use((Session *)s->session, b) && sub_matches(s->sub, b->hand))
            return pmul(hand_pose(b->hand, b->src, valid), s->offset);
    }
    *valid = 0;
    return IDENT;
}
static XrPosef space_in_stage(Space *s, int *valid) { return space_in_stage_at(s, frame_for(0), valid); }

static void space_velocity_in_stage(Space *s, V *outLinear, V *outAngular, int *valid) {
    *valid = 1;
    *outLinear = (V){0, 0, 0};
    *outAngular = (V){0, 0, 0};
    if (!s) { *valid = 0; return; }
    if (s->ref) {
        switch (s->type) {
            case XR_REFERENCE_SPACE_TYPE_STAGE:
            case XR_REFERENCE_SPACE_TYPE_LOCAL:
            case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR:
                *valid = 1;
                return;
            case XR_REFERENCE_SPACE_TYPE_VIEW: {
                VelocityTracker *vt = &velTrackers[TRACK_SLOT_HEAD];
                *valid = vt->valid;
                if (!vt->valid) return;
                XrPosef headPose = xp(frame_for(0)->head);
                V r = qrot(headPose.orientation, s->offset.position);
                *outLinear = vadd(vt->linearVel, vcross(vt->angularVel, r));
                *outAngular = vt->angularVel;
                return;
            }
            default:
                *valid = 0;
                return;
        }
    }
    for (int i = 0; i < s->action->nb; i++) {
        Binding *b = &s->action->b[i];
        if (IS_POSE(b->src) && in_use((Session *)s->session, b) && sub_matches(s->sub, b->hand)) {
            VelocityTracker *vt = &velTrackers[1 + 2 * b->hand + (b->src == S_GRIP)];
            *valid = vt->valid;
            if (!vt->valid) return;
            int hvalid;
            XrPosef hp = hand_pose(b->hand, b->src, &hvalid);
            if (!hvalid) { *valid = 0; return; }
            V r = qrot(hp.orientation, s->offset.position);
            *outLinear = vadd(vt->linearVel, vcross(vt->angularVel, r));
            *outAngular = vt->angularVel;
            return;
        }
    }
    *valid = 0;
}

static void set_local_origin(Session *s) {   // LOCAL = head position at start, yaw only
    Q q = {track.head.qx, track.head.qy, track.head.qz, track.head.qw};
    V f = qrot(q, (V){0, 0, -1});
    float yaw = atan2f(-f.x, -f.z);
    s->localOrigin.orientation = (Q){0, sinf(yaw / 2), 0, cosf(yaw / 2)};
    s->localOriginRaw = (V){track.head.px, track.head.py, track.head.pz};
    if (track.time_ns == 0) s->localOriginRaw.y = 1.6f;
    float ws = current_world_scale();
    s->localOrigin.position = (V){s->localOriginRaw.x / ws, s->localOriginRaw.y / ws, s->localOriginRaw.z / ws};
}

// ---------------------------------------------------------------- instance
#define FILL_ARRAY(cap, countOut, arr, n, ...) do { \
    if (countOut) *(countOut) = (n); \
    if ((cap) == 0) return XR_SUCCESS; \
    if ((cap) < (n)) return XR_ERROR_SIZE_INSUFFICIENT; \
    for (uint32_t i_ = 0; i_ < (n); i_++) { __VA_ARGS__; } } while (0)

// Compatibility: games refuse to start or wait forever without some of these (BONELAB blocks on XR_FB_display_refresh_rate).
// Depth layers are accepted and ignored; LOCAL_FLOOR is the local origin on the floor. The controller-profile
// extensions only unlock their interaction profiles (fed from Touch); the performance settings are hints.
#define EXTS(_) \
    _(XR_KHR_D3D11_enable, XR_KHR_D3D11_ENABLE_EXTENSION_NAME) \
    _(XR_KHR_D3D12_enable, XR_KHR_D3D12_ENABLE_EXTENSION_NAME) \
    _(XR_KHR_win32_convert_performance_counter_time, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME) \
    _(XR_FB_display_refresh_rate, XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME) \
    _(XR_KHR_composition_layer_depth, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME) \
    _(XR_KHR_composition_layer_cylinder, XR_KHR_COMPOSITION_LAYER_CYLINDER_EXTENSION_NAME) \
    _(XR_KHR_visibility_mask, XR_KHR_VISIBILITY_MASK_EXTENSION_NAME) \
    _(XR_EXT_local_floor, XR_EXT_LOCAL_FLOOR_EXTENSION_NAME) \
    _(XR_FB_color_space, XR_FB_COLOR_SPACE_EXTENSION_NAME) \
    _(XR_KHR_composition_layer_color_scale_bias, XR_KHR_COMPOSITION_LAYER_COLOR_SCALE_BIAS_EXTENSION_NAME) \
    _(XR_EXT_debug_utils, XR_EXT_DEBUG_UTILS_EXTENSION_NAME) \
    _(XR_EXT_hand_tracking, XR_EXT_HAND_TRACKING_EXTENSION_NAME) \
    _(XR_EXT_hand_joints_motion_range, XR_EXT_HAND_JOINTS_MOTION_RANGE_EXTENSION_NAME) \
    _(XR_EXT_hand_tracking_data_source, XR_EXT_HAND_TRACKING_DATA_SOURCE_EXTENSION_NAME) \
    _(XR_FB_hand_tracking_aim, XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME) \
    _(XR_EXT_hand_interaction, XR_EXT_HAND_INTERACTION_EXTENSION_NAME) \
    _(XR_META_touch_controller_plus, XR_META_TOUCH_CONTROLLER_PLUS_EXTENSION_NAME) \
    _(XR_FB_touch_controller_pro, XR_FB_TOUCH_CONTROLLER_PRO_EXTENSION_NAME) \
    _(XR_FB_touch_controller_proximity, XR_FB_TOUCH_CONTROLLER_PROXIMITY_EXTENSION_NAME) \
    _(XR_EXT_hp_mixed_reality_controller, XR_EXT_HP_MIXED_REALITY_CONTROLLER_EXTENSION_NAME) \
    _(XR_HTC_vive_cosmos_controller_interaction, XR_HTC_VIVE_COSMOS_CONTROLLER_INTERACTION_EXTENSION_NAME) \
    _(XR_EXT_performance_settings, XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME) \
    _(XR_KHR_locate_spaces, XR_KHR_LOCATE_SPACES_EXTENSION_NAME)
// no XR_EXT_palm_pose: we only have grip/aim, and a fake palm (= grip) made OpenComposite rotate/shift the hands
#define EXT_NAME(id, name) name,
#define EXT_VER(id, name) id##_SPEC_VERSION,
static const char *exts[] = {EXTS(EXT_NAME)};
static const uint32_t extVer[] = {EXTS(EXT_VER)};
#define NEXTS (sizeof exts / sizeof *exts)

static XrResult XRAPI_CALL xrEnumerateApiLayerProperties_(uint32_t cap, uint32_t *n, XrApiLayerProperties *p) { (void)cap; (void)p; *n = 0; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrEnumerateInstanceExtensionProperties_(const char *layer, uint32_t cap, uint32_t *n, XrExtensionProperties *p) {
    (void)layer;
    FILL_ARRAY(cap, n, p, (uint32_t)NEXTS, { strcpy(p[i_].extensionName, exts[i_]); p[i_].extensionVersion = extVer[i_]; });
    return XR_SUCCESS;
}

static XrResult XRAPI_CALL xrCreateInstance_(const XrInstanceCreateInfo *ci, XrInstance *out) {
    char req[1024] = "";
    for (uint32_t i = 0; i < ci->enabledExtensionCount; i++) {
        int ok = 0;
        for (size_t j = 0; j < NEXTS; j++) ok |= !strcmp(ci->enabledExtensionNames[i], exts[j]);
        if (!ok) { logmsg("game requires %s, which WineXR does not implement", ci->enabledExtensionNames[i]); return XR_ERROR_EXTENSION_NOT_PRESENT; }
        if (strlen(req) + strlen(ci->enabledExtensionNames[i]) + 2 < sizeof req) strcat(strcat(req, " "), ci->enabledExtensionNames[i]);
    }
    if (!shm) {
        init_paths();
        HANDLE f = CreateFileA(shmPath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
        if (f == INVALID_HANDLE_VALUE) { logmsg("VR4Mac app not running (no %s)", shmPath); return XR_ERROR_RUNTIME_UNAVAILABLE; }
        HANDLE m = CreateFileMappingA(f, NULL, PAGE_READWRITE, 0, VR4_SHM_SIZE, NULL);
        shm = m ? (VR4Shm *)MapViewOfFile(m, FILE_MAP_ALL_ACCESS, 0, 0, VR4_SHM_SIZE) : NULL;
        if (!shm || shm->magic != VR4_SHM_MAGIC || shm->version != VR4_SHM_VERSION) { logmsg("bad shared memory"); shm = NULL; return XR_ERROR_RUNTIME_UNAVAILABLE; }
    }
    QueryPerformanceFrequency(&qpf);
    handPath[0] = intern("/user/hand/left"); handPath[1] = intern("/user/hand/right");
    snprintf(appName, sizeof appName, "%s", ci->applicationInfo.applicationName);
    snprintf(shm->app_name, sizeof shm->app_name, "%s", appName);
    logmsg("xrCreateInstance app=%s engine=%s api %u.%u, extensions:%s", ci->applicationInfo.applicationName, ci->applicationInfo.engineName,
           XR_VERSION_MAJOR(ci->applicationInfo.apiVersion), XR_VERSION_MINOR(ci->applicationInfo.apiVersion), req);
    *out = (XrInstance)(uintptr_t)1;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroyInstance_(XrInstance i) { (void)i; nsugg = 0; return XR_SUCCESS; }   // suggestions are per instance
static XrResult XRAPI_CALL xrGetInstanceProperties_(XrInstance i, XrInstanceProperties *p) {
    (void)i; p->runtimeVersion = XR_MAKE_VERSION(1, 1, 0); strcpy(p->runtimeName, "VR4Mac"); return XR_SUCCESS;
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
        if (next->type == XR_TYPE_SYSTEM_COLOR_SPACE_PROPERTIES_FB) ((XrSystemColorSpacePropertiesFB *)next)->colorSpace = XR_COLOR_SPACE_QUEST_FB;
        if (next->type == XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT) ((XrSystemHandTrackingPropertiesEXT *)next)->supportsHandTracking = XR_TRUE;
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
static LUID adapter_luid(void) {   // the first (only) GPU
    IDXGIFactory1 *f; IDXGIAdapter1 *a; DXGI_ADAPTER_DESC1 d; LUID l = {0, 0};
    if (SUCCEEDED(CreateDXGIFactory1(&IID_IDXGIFactory1, (void **)&f))) {
        if (SUCCEEDED(IDXGIFactory1_EnumAdapters1(f, 0, &a))) {
            if (SUCCEEDED(IDXGIAdapter1_GetDesc1(a, &d))) l = d.AdapterLuid;
            IDXGIAdapter1_Release(a);
        }
        IDXGIFactory1_Release(f);
    }
    return l;
}
static XrResult XRAPI_CALL xrGetD3D11GraphicsRequirementsKHR_(XrInstance i, XrSystemId id, XrGraphicsRequirementsD3D11KHR *r) {
    (void)i; (void)id; r->adapterLuid = adapter_luid(); r->minFeatureLevel = D3D_FEATURE_LEVEL_11_0; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetD3D12GraphicsRequirementsKHR_(XrInstance i, XrSystemId id, XrGraphicsRequirementsD3D12KHR *r) {
    (void)i; (void)id; r->adapterLuid = adapter_luid(); r->minFeatureLevel = D3D_FEATURE_LEVEL_11_0; return XR_SUCCESS;
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
    const XrGraphicsBindingD3D11KHR *gb = NULL; const XrGraphicsBindingD3D12KHR *gb12 = NULL;
    for (const XrBaseInStructure *b = ci->next; b; b = b->next) {
        if (b->type == XR_TYPE_GRAPHICS_BINDING_D3D11_KHR) gb = (const XrGraphicsBindingD3D11KHR *)b;
        if (b->type == XR_TYPE_GRAPHICS_BINDING_D3D12_KHR) gb12 = (const XrGraphicsBindingD3D12KHR *)b;
    }
    if ((!gb || !gb->device) && (!gb12 || !gb12->device || !gb12->queue)) {
        char name[XR_MAX_STRUCTURE_NAME_SIZE] = "no graphics binding";
        if (ci->next) xrStructureTypeToString_(XR_NULL_HANDLE, ((const XrBaseInStructure *)ci->next)->type, name);
        logmsg("xrCreateSession: the game renders with %s; WineXR supports D3D11 and D3D12", name);
        return XR_ERROR_GRAPHICS_DEVICE_INVALID;
    }
    Session *s = calloc(1, sizeof *s);
    if (gb && gb->device) { s->dev = gb->device; ID3D11Device_AddRef(s->dev); ID3D11Device_GetImmediateContext(s->dev, &s->ctx); }
    else {
        s->dev12 = gb12->device; ID3D12Device_AddRef(s->dev12); s->queue = gb12->queue; ID3D12CommandQueue_AddRef(s->queue);
        int ok = SUCCEEDED(ID3D12Device_CreateFence(s->dev12, 0, D3D12_FENCE_FLAG_NONE, &IID_ID3D12Fence, (void **)&s->fence));
        for (int k = 0; ok && k < NSTAGE; k++)
            ok = SUCCEEDED(ID3D12Device_CreateCommandAllocator(s->dev12, D3D12_COMMAND_LIST_TYPE_DIRECT, &IID_ID3D12CommandAllocator, (void **)&s->alloc[k])) &&
                 SUCCEEDED(ID3D12Device_CreateCommandList(s->dev12, 0, D3D12_COMMAND_LIST_TYPE_DIRECT, s->alloc[k], NULL, &IID_ID3D12GraphicsCommandList, (void **)&s->cl[k])) &&
                 SUCCEEDED(ID3D12GraphicsCommandList_Close(s->cl[k]));
        s->fenceEvent = CreateEventA(NULL, FALSE, FALSE, NULL);
        if (!ok) { logmsg("xrCreateSession: D3D12 readback setup failed"); rel(s->fence); s->fence = NULL; }
    }
    read_tracking(); frameTrack = track; set_local_origin(s);
    theSession = s;
    *out = (XrSession)s;
    push_state(XR_SESSION_STATE_IDLE); push_state(XR_SESSION_STATE_READY);
    logmsg("xrCreateSession ok, eye %ux%u", eye_w(), eye_h());
    return XR_SUCCESS;
}
static void wait_gpu12(Session *s, uint64_t v) {   // until the game's queue has passed our signal `v`
    if (s->fence && ID3D12Fence_GetCompletedValue(s->fence) < v && SUCCEEDED(ID3D12Fence_SetEventOnCompletion(s->fence, v, s->fenceEvent)))
        WaitForSingleObject(s->fenceEvent, 1000);
}
static XrResult XRAPI_CALL xrDestroySession_(XrSession h) {
    Session *s = (Session *)h;
    wait_gpu12(s, s->fenceValue);
    for (int k = 0; k < NSTAGE; k++) { rel(s->alloc[k]); rel(s->cl[k]); rel(s->rb[k]); }
    rel(s->fence); rel(s->queue); rel(s->dev12);
    if (s->fenceEvent) CloseHandle(s->fenceEvent);
    for (int k = 0; k < NSTAGE; k++) rel(s->staging[k]);
    rel(s->vs); rel(s->ps); rel(s->il); rel(s->vb); rel(s->cb); rel(s->smp); rel(s->blend[0]); rel(s->blend[1]); rel(s->rs);
    rel(s->rtv); rel(s->target); rel(s->ctx); rel(s->dev);
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
        logonce("reference space %d unsupported", ci->referenceSpaceType); return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
    }
    Space *s = calloc(1, sizeof *s);
    s->session = h; s->ref = 1; s->type = ci->referenceSpaceType; s->offset = ci->poseInReferenceSpace;
    *out = (XrSpace)s; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetReferenceSpaceBoundsRect_(XrSession h, XrReferenceSpaceType t, XrExtent2Df *b) {
    (void)h;
    if (t != XR_REFERENCE_SPACE_TYPE_STAGE) { b->width = b->height = 0; return XR_SPACE_BOUNDS_UNAVAILABLE; }
    float ws = current_world_scale();
    b->width = 2.0f / ws; b->height = 2.0f / ws; return XR_SUCCESS;   // fixed 2x2 m play area scaled to stage
}
static XrResult XRAPI_CALL xrCreateActionSpace_(XrSession h, const XrActionSpaceCreateInfo *ci, XrSpace *out) {
    Space *s = calloc(1, sizeof *s);
    s->session = h; s->action = (Action *)ci->action; s->sub = ci->subactionPath; s->offset = ci->poseInActionSpace;
    *out = (XrSpace)s; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrLocateSpace_(XrSpace sp, XrSpace base, XrTime t, XrSpaceLocation *loc) {
    (void)t;
    if (!loc) return XR_ERROR_VALIDATION_FAILURE;
    if (!sp || !base) return XR_ERROR_HANDLE_INVALID;
    int v1, v2;
    XrPosef a = space_in_stage((Space *)sp, &v1), b = space_in_stage((Space *)base, &v2);
    loc->pose = pmul(pinv(b), a);
    loc->locationFlags = v1 && v2 ? XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT |
                                    XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT : 0;
    for (XrBaseOutStructure *n = loc->next; n; n = n->next)
        if (n->type == XR_TYPE_SPACE_VELOCITY) {
            XrSpaceVelocity *sv = (XrSpaceVelocity *)n;
            V va, wa, vb, wb;
            int vvel1 = 0, vvel2 = 0;
            space_velocity_in_stage((Space *)sp, &va, &wa, &vvel1);
            space_velocity_in_stage((Space *)base, &vb, &wb, &vvel2);
            if (v1 && v2 && vvel1 && vvel2) {
                V relLin = vsub(va, vb);
                V relAng = vsub(wa, wb);
                sv->linearVelocity = qrot(qconj(b.orientation), relLin);
                sv->angularVelocity = qrot(qconj(b.orientation), relAng);
                sv->velocityFlags = XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
            } else {
                sv->velocityFlags = 0;
                sv->linearVelocity = (XrVector3f){0, 0, 0};
                sv->angularVelocity = (XrVector3f){0, 0, 0};
            }
        }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrLocateSpaces_(XrSession h, const XrSpacesLocateInfo *li, XrSpaceLocations *out) {   // XR_KHR_locate_spaces / 1.1
    (void)h;
    if (!li || !out || !out->locations || li->spaceCount != out->locationCount) return XR_ERROR_VALIDATION_FAILURE;
    XrSpaceVelocities *vel = NULL;
    for (XrBaseOutStructure *n = out->next; n; n = n->next) if (n->type == XR_TYPE_SPACE_VELOCITIES) vel = (XrSpaceVelocities *)n;
    if (vel && (vel->velocityCount != li->spaceCount || !vel->velocities)) return XR_ERROR_VALIDATION_FAILURE;
    for (uint32_t i = 0; i < li->spaceCount; i++) {
        XrSpaceVelocity sv = {XR_TYPE_SPACE_VELOCITY}; XrSpaceLocation l = {XR_TYPE_SPACE_LOCATION, vel ? &sv : NULL};
        XrResult r = xrLocateSpace_(li->spaces[i], li->baseSpace, li->time, &l);
        if (XR_FAILED(r)) return r;
        out->locations[i] = (XrSpaceLocationData){l.locationFlags, l.pose};
        if (vel) vel->velocities[i] = (XrSpaceVelocityData){sv.velocityFlags, sv.linearVelocity, sv.angularVelocity};
    }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroySpace_(XrSpace s) { free(s); return XR_SUCCESS; }

// ---------------------------------------------------------------- frames
static int64_t endSum, copySum;   // pacing log: time inside xrEndFrame / in the CPU readback copy
static XrResult XRAPI_CALL xrWaitFrame_(XrSession h, const XrFrameWaitInfo *wi, XrFrameState *fs) {
    (void)wi; (void)h;
    if (!fs) return XR_ERROR_VALIDATION_FAILURE;
    float fps = shm->fps > 0 ? shm->fps : 72;
    int64_t period = (int64_t)(1e9 / fps), start = qpc_ns();
    static int64_t lastExit, statStart, waitSum, workSum; static int statN;
    // Pace the game to the headset: block until the next tracking sample, but never past one period (+2 ms jitter)
    // after the previous frame was released. The deadline used to be 2 periods from *now*, so stale tracking (headset
    // asleep while linked) stacked the full timeout on top of the game's own frame time: 6 ms game -> 29 fps.
    // Spins only within 3 ms of when the next sample is due (one period after the last); sleeps otherwise (Sleep(1) is
    // ~1 ms under Wine), so a quiet headset doesn't cost a busy core.
    int64_t deadline = (lastExit ? lastExit : start) + period + 2000000;
    while (!read_tracking()) {
        int64_t now = qpc_ns(), due = lastSampleNs + period;
        if (now > deadline) break;
        Sleep(lastSampleNs && now > due - 3000000 && now < due + 3000000 ? 0 : 1);
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
    if (!li || !vs) return XR_ERROR_VALIDATION_FAILURE;
    if (!li->space) return XR_ERROR_HANDLE_INVALID;
    int valid;
    const VR4Tracking *ft = frame_for(li->displayTime);
    XrPosef base = space_in_stage_at((Space *)li->space, ft, &valid);
    vs->viewStateFlags = valid ? (XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT |
                                  XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT) : 0;
    FILL_ARRAY(cap, n, views, 2, {
        const VR4Eye *e = &ft->eye[i_];
        views[i_].pose = pmul(pinv(base), xp(e->pose));
        views[i_].fov = (XrFovf){e->fov.left, e->fov.right, e->fov.up, e->fov.down};
    });
    return XR_SUCCESS;
}

static Swapchain *find_sc(XrSwapchain s) { return (Swapchain *)s; }
// Swapchain textures are typeless; 8-bit colour families are what the compositor reads (raw bytes, no sRGB conversion).
static int is_rgba(DXGI_FORMAT f) { return f == DXGI_FORMAT_R8G8B8A8_TYPELESS; }
static int is_color(DXGI_FORMAT f) { return f == DXGI_FORMAT_R8G8B8A8_TYPELESS || f == DXGI_FORMAT_B8G8R8A8_TYPELESS; }
static DXGI_FORMAT unorm(DXGI_FORMAT f) { return is_rgba(f) ? DXGI_FORMAT_R8G8B8A8_UNORM : DXGI_FORMAT_B8G8R8A8_UNORM; }

// ---------------------------------------------------------------- compositor
// The first stereo projection is copied into a side-by-side target; quad and cylinder layers submitted after it are
// drawn on top in order (anything before an opaque projection is hidden by it). Without a projection, the layers go
// on black at their real place in the world. Clip-space vertices are built on the CPU, so the shaders stay trivial.
static const char hlsl[] =
    "Texture2D t : register(t0); SamplerState s : register(s0); cbuffer c : register(b0) { float4 scale, bias; };\n"
    "void vs(float4 p : POSITION, float2 uv : TEXCOORD, out float4 o : SV_Position, out float2 ouv : TEXCOORD) { o = p; ouv = uv; }\n"
    "float4 ps(float4 p : SV_Position, float2 uv : TEXCOORD) : SV_Target { return t.Sample(s, uv) * scale + bias; }\n";
typedef struct { float x, y, z, w, u, v; } Vtx;
#define CYL_SEGS 32
#define MAXV (2 * (CYL_SEGS + 1))

static int comp_init(Session *s) {
    if (s->compOk) return s->compOk > 0;
    s->compOk = -1;
    HMODULE dc = LoadLibraryA("d3dcompiler_47.dll");
    pD3DCompile compile = dc ? (pD3DCompile)(void *)GetProcAddress(dc, "D3DCompile") : NULL;
    ID3DBlob *vsb = NULL, *psb = NULL, *err = NULL;
    if (!compile || FAILED(compile(hlsl, sizeof hlsl - 1, "winexr", NULL, NULL, "vs", "vs_4_0", 0, 0, &vsb, &err)) ||
        FAILED(compile(hlsl, sizeof hlsl - 1, "winexr", NULL, NULL, "ps", "ps_4_0", 0, 0, &psb, &err))) {
        logmsg("layer compositor unavailable (%s): quad/cylinder layers are not shown", err ? (const char *)ID3D10Blob_GetBufferPointer(err) : "no d3dcompiler_47");
        rel(vsb); rel(psb); rel(err);
        return 0;
    }
    D3D11_INPUT_ELEMENT_DESC ie[] = {{"POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
                                     {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 16, D3D11_INPUT_PER_VERTEX_DATA, 0}};
    D3D11_BUFFER_DESC vbd = {MAXV * sizeof(Vtx), D3D11_USAGE_DYNAMIC, D3D11_BIND_VERTEX_BUFFER, D3D11_CPU_ACCESS_WRITE, 0, 0};
    D3D11_BUFFER_DESC cbd = {32, D3D11_USAGE_DYNAMIC, D3D11_BIND_CONSTANT_BUFFER, D3D11_CPU_ACCESS_WRITE, 0, 0};
    D3D11_SAMPLER_DESC sd = {D3D11_FILTER_MIN_MAG_MIP_LINEAR, D3D11_TEXTURE_ADDRESS_CLAMP, D3D11_TEXTURE_ADDRESS_CLAMP, D3D11_TEXTURE_ADDRESS_CLAMP,
                             0, 1, D3D11_COMPARISON_NEVER, {0, 0, 0, 0}, 0, D3D11_FLOAT32_MAX};
    D3D11_RASTERIZER_DESC rd = {D3D11_FILL_SOLID, D3D11_CULL_NONE, FALSE, 0, 0, 0, TRUE, FALSE, FALSE, FALSE};
    D3D11_BLEND_DESC bd = {FALSE, FALSE, {{TRUE, D3D11_BLEND_ONE, D3D11_BLEND_INV_SRC_ALPHA, D3D11_BLEND_OP_ADD,   // premultiplied alpha
                                           D3D11_BLEND_ONE, D3D11_BLEND_INV_SRC_ALPHA, D3D11_BLEND_OP_ADD, D3D11_COLOR_WRITE_ENABLE_ALL}}};
    ID3D11Device *d = s->dev;
    const void *vsc = ID3D10Blob_GetBufferPointer(vsb); SIZE_T vsn = ID3D10Blob_GetBufferSize(vsb);
    int ok = SUCCEEDED(ID3D11Device_CreateVertexShader(d, vsc, vsn, NULL, &s->vs)) &&
             SUCCEEDED(ID3D11Device_CreatePixelShader(d, ID3D10Blob_GetBufferPointer(psb), ID3D10Blob_GetBufferSize(psb), NULL, &s->ps)) &&
             SUCCEEDED(ID3D11Device_CreateInputLayout(d, ie, 2, vsc, vsn, &s->il)) &&
             SUCCEEDED(ID3D11Device_CreateBuffer(d, &vbd, NULL, &s->vb)) && SUCCEEDED(ID3D11Device_CreateBuffer(d, &cbd, NULL, &s->cb)) &&
             SUCCEEDED(ID3D11Device_CreateSamplerState(d, &sd, &s->smp)) && SUCCEEDED(ID3D11Device_CreateRasterizerState(d, &rd, &s->rs)) &&
             SUCCEEDED(ID3D11Device_CreateBlendState(d, &bd, &s->blend[0]));
    bd.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;   // straight alpha (XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT)
    ok = ok && SUCCEEDED(ID3D11Device_CreateBlendState(d, &bd, &s->blend[1]));
    rel(vsb); rel(psb); rel(err);
    if (!ok) { logmsg("layer compositor: D3D11 setup failed, quad/cylinder layers are not shown"); return 0; }
    s->compOk = 1;
    return 1;
}

typedef struct {   // the game's pipeline state the compositor touches, restored afterwards
    ID3D11InputLayout *il; D3D11_PRIMITIVE_TOPOLOGY topo; ID3D11Buffer *vb, *pcb; UINT stride, off;
    ID3D11VertexShader *vs; ID3D11HullShader *hs; ID3D11DomainShader *ds; ID3D11GeometryShader *gs; ID3D11PixelShader *ps;
    ID3D11ShaderResourceView *srv; ID3D11SamplerState *smp; ID3D11RasterizerState *rs;
    D3D11_VIEWPORT vp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]; UINT nvp;
    ID3D11RenderTargetView *rtv[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT]; ID3D11DepthStencilView *dsv;
    ID3D11BlendState *bs; FLOAT bf[4]; UINT mask; ID3D11DepthStencilState *dss; UINT sref;
} SavedState;
static void save_state(ID3D11DeviceContext *c, SavedState *z) {
    ID3D11DeviceContext_IAGetInputLayout(c, &z->il); ID3D11DeviceContext_IAGetPrimitiveTopology(c, &z->topo);
    ID3D11DeviceContext_IAGetVertexBuffers(c, 0, 1, &z->vb, &z->stride, &z->off);
    ID3D11DeviceContext_VSGetShader(c, &z->vs, NULL, NULL); ID3D11DeviceContext_HSGetShader(c, &z->hs, NULL, NULL);
    ID3D11DeviceContext_DSGetShader(c, &z->ds, NULL, NULL); ID3D11DeviceContext_GSGetShader(c, &z->gs, NULL, NULL);
    ID3D11DeviceContext_PSGetShader(c, &z->ps, NULL, NULL); ID3D11DeviceContext_PSGetConstantBuffers(c, 0, 1, &z->pcb);
    ID3D11DeviceContext_PSGetShaderResources(c, 0, 1, &z->srv); ID3D11DeviceContext_PSGetSamplers(c, 0, 1, &z->smp);
    ID3D11DeviceContext_RSGetState(c, &z->rs);
    z->nvp = 0; ID3D11DeviceContext_RSGetViewports(c, &z->nvp, NULL); ID3D11DeviceContext_RSGetViewports(c, &z->nvp, z->vp);
    ID3D11DeviceContext_OMGetRenderTargets(c, D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, z->rtv, &z->dsv);
    ID3D11DeviceContext_OMGetBlendState(c, &z->bs, z->bf, &z->mask); ID3D11DeviceContext_OMGetDepthStencilState(c, &z->dss, &z->sref);
}
static void restore_state(ID3D11DeviceContext *c, SavedState *z) {
    ID3D11DeviceContext_IASetInputLayout(c, z->il); ID3D11DeviceContext_IASetPrimitiveTopology(c, z->topo);
    ID3D11DeviceContext_IASetVertexBuffers(c, 0, 1, &z->vb, &z->stride, &z->off);
    ID3D11DeviceContext_VSSetShader(c, z->vs, NULL, 0); ID3D11DeviceContext_HSSetShader(c, z->hs, NULL, 0);
    ID3D11DeviceContext_DSSetShader(c, z->ds, NULL, 0); ID3D11DeviceContext_GSSetShader(c, z->gs, NULL, 0);
    ID3D11DeviceContext_PSSetShader(c, z->ps, NULL, 0); ID3D11DeviceContext_PSSetConstantBuffers(c, 0, 1, &z->pcb);
    ID3D11DeviceContext_PSSetShaderResources(c, 0, 1, &z->srv); ID3D11DeviceContext_PSSetSamplers(c, 0, 1, &z->smp);
    ID3D11DeviceContext_RSSetState(c, z->rs); ID3D11DeviceContext_RSSetViewports(c, z->nvp, z->vp);
    ID3D11DeviceContext_OMSetRenderTargets(c, D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, z->rtv, z->dsv);
    ID3D11DeviceContext_OMSetBlendState(c, z->bs, z->bf, z->mask); ID3D11DeviceContext_OMSetDepthStencilState(c, z->dss, z->sref);
    rel(z->il); rel(z->vb); rel(z->pcb); rel(z->vs); rel(z->hs); rel(z->ds); rel(z->gs); rel(z->ps); rel(z->srv); rel(z->smp);
    rel(z->rs); rel(z->dsv); rel(z->bs); rel(z->dss);
    for (int i = 0; i < D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT; i++) rel(z->rtv[i]);
}

static int is_overlay(const XrCompositionLayerBaseHeader *l) {
    return l && (l->type == XR_TYPE_COMPOSITION_LAYER_QUAD || l->type == XR_TYPE_COMPOSITION_LAYER_CYLINDER_KHR);
}
/// Draws one quad or cylinder layer into both eyes of the bound side-by-side target (each eye `w` x `h`).
static void draw_layer(Session *s, const XrCompositionLayerBaseHeader *L, const VR4Tracking *ft, const XrPosef eye[2], const XrFovf fov[2], uint32_t w, uint32_t h) {
    const XrSwapchainSubImage *si; XrSpace space; XrPosef pose; XrEyeVisibility vis; Vtx loc[MAXV]; int nv = 0;
    if (L->type == XR_TYPE_COMPOSITION_LAYER_QUAD) {   // in the pose's XY plane, facing +Z
        const XrCompositionLayerQuad *q = (const XrCompositionLayerQuad *)L;
        si = &q->subImage; space = q->space; pose = q->pose; vis = q->eyeVisibility;
        float x = q->size.width / 2, y = q->size.height / 2;
        loc[0] = (Vtx){-x, y, 0, 1, 0, 0}; loc[1] = (Vtx){x, y, 0, 1, 1, 0}; loc[2] = (Vtx){-x, -y, 0, 1, 0, 1}; loc[3] = (Vtx){x, -y, 0, 1, 1, 1};
        nv = 4;
    } else {   // cylinder: an arc around the pose's Y axis, centred on -Z, seen from inside
        const XrCompositionLayerCylinderKHR *c = (const XrCompositionLayerCylinderKHR *)L;
        si = &c->subImage; space = c->space; pose = c->pose; vis = c->eyeVisibility;
        if (!(c->radius > 0) || isinf(c->radius) || !(c->centralAngle > 0) || !(c->aspectRatio > 0)) {
            logonce("cylinder layer radius %g angle %g aspect %g not drawn", c->radius, c->centralAngle, c->aspectRatio); return;
        }
        float half = c->radius * c->centralAngle / c->aspectRatio / 2;
        for (int i = 0; i <= CYL_SEGS; i++) {
            float f = (float)i / CYL_SEGS, a = (f - 0.5f) * c->centralAngle, x = c->radius * sinf(a), z = -c->radius * cosf(a);
            loc[nv++] = (Vtx){x, half, z, 1, f, 0}; loc[nv++] = (Vtx){x, -half, z, 1, f, 1};
        }
    }
    Swapchain *sc = find_sc(si->swapchain);
    if (!sc || sc->released < 0 || si->imageArrayIndex >= sc->array || !is_color(sc->fmt)) return;
    float u0 = (float)si->imageRect.offset.x / sc->w, v0 = (float)si->imageRect.offset.y / sc->h;
    float du = (float)si->imageRect.extent.width / sc->w, dv = (float)si->imageRect.extent.height / sc->h;
    if (!space) return;
    int valid; XrPosef world = pmul(space_in_stage_at((Space *)space, ft, &valid), pose);
    if (!valid) return;
    float cb[8] = {1, 1, 1, 1, 0, 0, 0, 0};   // XR_KHR_composition_layer_color_scale_bias
    for (const XrBaseInStructure *n = L->next; n; n = n->next)
        if (n->type == XR_TYPE_COMPOSITION_LAYER_COLOR_SCALE_BIAS_KHR) {
            const XrCompositionLayerColorScaleBiasKHR *c = (const XrCompositionLayerColorScaleBiasKHR *)n;
            memcpy(cb, &c->colorScale, 16); memcpy(cb + 4, &c->colorBias, 16);
        }
    D3D11_SHADER_RESOURCE_VIEW_DESC sd = {unorm(sc->fmt), D3D11_SRV_DIMENSION_TEXTURE2DARRAY};
    sd.Texture2DArray.MipLevels = 1; sd.Texture2DArray.FirstArraySlice = si->imageArrayIndex; sd.Texture2DArray.ArraySize = 1;
    ID3D11ShaderResourceView *srv;
    if (FAILED(ID3D11Device_CreateShaderResourceView(s->dev, (ID3D11Resource *)sc->img[sc->released], &sd, &srv))) return;
    ID3D11DeviceContext *c = s->ctx; D3D11_MAPPED_SUBRESOURCE m;
    if (SUCCEEDED(ID3D11DeviceContext_Map(c, (ID3D11Resource *)s->cb, 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) { memcpy(m.pData, cb, sizeof cb); ID3D11DeviceContext_Unmap(c, (ID3D11Resource *)s->cb, 0); }
    ID3D11DeviceContext_PSSetShaderResources(c, 0, 1, &srv);
    XrCompositionLayerFlags lf = L->layerFlags;
    ID3D11DeviceContext_OMSetBlendState(c, lf & XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT ?
        s->blend[(lf & XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT) != 0] : NULL, NULL, 0xffffffff);
    for (int e = 0; e < 2; e++) {
        if ((vis == XR_EYE_VISIBILITY_LEFT && e) || (vis == XR_EYE_VISIBILITY_RIGHT && !e)) continue;
        XrPosef mv = pmul(pinv(eye[e]), world);   // layer -> eye
        float l = tanf(fov[e].angleLeft), r = tanf(fov[e].angleRight), u = tanf(fov[e].angleUp), d = tanf(fov[e].angleDown);
        if (FAILED(ID3D11DeviceContext_Map(c, (ID3D11Resource *)s->vb, 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) continue;
        Vtx *out = m.pData;
        for (int i = 0; i < nv; i++) {   // asymmetric-frustum projection; z keeps 1 cm near clip, no depth test
            V p = vadd(qrot(mv.orientation, (V){loc[i].x, loc[i].y, loc[i].z}), mv.position);
            out[i] = (Vtx){(2 * p.x + (r + l) * p.z) / (r - l), (2 * p.y + (u + d) * p.z) / (u - d), -p.z - 0.01f, -p.z,
                           u0 + loc[i].u * du, v0 + loc[i].v * dv};
        }
        ID3D11DeviceContext_Unmap(c, (ID3D11Resource *)s->vb, 0);
        D3D11_VIEWPORT vp = {(FLOAT)(e * w), 0, (FLOAT)w, (FLOAT)h, 0, 1};
        ID3D11DeviceContext_RSSetViewports(c, 1, &vp);
        ID3D11DeviceContext_Draw(c, (UINT)nv, 0);
    }
    rel(srv);
}

/// Side-by-side render target the layers are drawn into (same colour family as the projection).
static int ensure_target(Session *s, DXGI_FORMAT fmt, uint32_t w, uint32_t h) {
    if (s->target && s->targetFmt == fmt && s->targetW == w && s->targetH == h) return 1;
    rel(s->rtv); rel(s->target); s->rtv = NULL; s->target = NULL;
    D3D11_TEXTURE2D_DESC d = {w, h, 1, 1, fmt, {1, 0}, D3D11_USAGE_DEFAULT, D3D11_BIND_RENDER_TARGET, 0, 0};
    D3D11_RENDER_TARGET_VIEW_DESC rd = {unorm(fmt), D3D11_RTV_DIMENSION_TEXTURE2D};
    if (FAILED(ID3D11Device_CreateTexture2D(s->dev, &d, NULL, &s->target)) ||
        FAILED(ID3D11Device_CreateRenderTargetView(s->dev, (ID3D11Resource *)s->target, &rd, &s->rtv))) {
        logonce("compositor target %ux%u alloc failed", w, h); rel(s->target); s->target = NULL; return 0;
    }
    s->targetFmt = fmt; s->targetW = w; s->targetH = h;
    return 1;
}
static int ensure_staging(Session *s, DXGI_FORMAT fmt, uint32_t w, uint32_t h) {
    if (s->staging[0] && s->stagingFmt == fmt && s->stagingW == w && s->stagingH == h) return 1;
    for (int k = 0; k < NSTAGE; k++) { rel(s->staging[k]); s->staging[k] = NULL; s->stagingReady[k] = 0; }
    s->stagingFmt = 0; s->stagingW = s->stagingH = 0;
    D3D11_TEXTURE2D_DESC d = {w, h, 1, 1, fmt, {1, 0}, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0};
    for (int k = 0; k < NSTAGE; k++)
        if (FAILED(ID3D11Device_CreateTexture2D(s->dev, &d, NULL, &s->staging[k]))) {
            logmsg("staging alloc %ux%u failed at %d", w, h, k);
            for (int r = 0; r < NSTAGE; r++) { rel(s->staging[r]); s->staging[r] = NULL; }
            return 0;
        }
    s->stagingFmt = fmt; s->stagingW = w; s->stagingH = h;
    return 1;
}

// D3D12: readback buffers (rows padded to 256 bytes) filled by a command list per slot on the game's queue.
static int ensure_readback12(Session *s, DXGI_FORMAT fmt, uint32_t w, uint32_t h) {
    if (s->rb[0] && s->stagingFmt == fmt && s->stagingW == w && s->stagingH == h) return 1;
    wait_gpu12(s, s->fenceValue);   // no copy may still be landing in the buffers we drop
    for (int k = 0; k < NSTAGE; k++) { rel(s->rb[k]); s->rb[k] = NULL; s->stagingReady[k] = 0; }
    s->stagingFmt = 0; s->stagingW = s->stagingH = 0;
    s->rbPitch = (w * 4 + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);
    D3D12_HEAP_PROPERTIES hp = {D3D12_HEAP_TYPE_READBACK, D3D12_CPU_PAGE_PROPERTY_UNKNOWN, D3D12_MEMORY_POOL_UNKNOWN, 0, 0};
    D3D12_RESOURCE_DESC d = {D3D12_RESOURCE_DIMENSION_BUFFER, 0, (UINT64)s->rbPitch * h, 1, 1, 1, DXGI_FORMAT_UNKNOWN, {1, 0}, D3D12_TEXTURE_LAYOUT_ROW_MAJOR, D3D12_RESOURCE_FLAG_NONE};
    for (int k = 0; k < NSTAGE; k++)
        if (FAILED(ID3D12Device_CreateCommittedResource(s->dev12, &hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COPY_DEST, NULL, &IID_ID3D12Resource, (void **)&s->rb[k]))) {
            logmsg("readback alloc %ux%u failed at %d", w, h, k);
            for (int r = 0; r < NSTAGE; r++) { rel(s->rb[r]); s->rb[r] = NULL; }
            return 0;
        }
    s->stagingFmt = fmt; s->stagingW = w; s->stagingH = h;
    return 1;
}
static void copy12(Session *s, int k, const XrCompositionLayerProjection *proj, Swapchain *sc[2], uint32_t w, uint32_t hgt) {
    wait_gpu12(s, s->rbFence[k]);   // the slot's previous copy must be done before its allocator is reused (ring: almost never waits)
    ID3D12GraphicsCommandList *cl = s->cl[k];
    ID3D12CommandAllocator_Reset(s->alloc[k]); ID3D12GraphicsCommandList_Reset(cl, s->alloc[k], NULL);
    for (int e = 0; e < 2; e++) {   // images are handed back in RENDER_TARGET state (XR_KHR_D3D12_enable)
        const XrSwapchainSubImage *si = &proj->views[e].subImage;
        ID3D12Resource *img = sc[e]->img12[sc[e]->released]; UINT sub = si->imageArrayIndex * sc[e]->mips;
        D3D12_RESOURCE_BARRIER b = {D3D12_RESOURCE_BARRIER_TYPE_TRANSITION, D3D12_RESOURCE_BARRIER_FLAG_NONE};
        b.Transition = (D3D12_RESOURCE_TRANSITION_BARRIER){img, sub, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE};
        ID3D12GraphicsCommandList_ResourceBarrier(cl, 1, &b);
        D3D12_TEXTURE_COPY_LOCATION to = {s->rb[k], D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT}, from = {img, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX};
        to.PlacedFootprint.Footprint = (D3D12_SUBRESOURCE_FOOTPRINT){sc[e]->fmt, 2 * w, hgt, 1, s->rbPitch};
        from.SubresourceIndex = sub;
        D3D12_BOX box = {(UINT)si->imageRect.offset.x, (UINT)si->imageRect.offset.y, 0, (UINT)si->imageRect.offset.x + w, (UINT)si->imageRect.offset.y + hgt, 1};
        ID3D12GraphicsCommandList_CopyTextureRegion(cl, &to, e * w, 0, 0, &from, &box);
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE; b.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        ID3D12GraphicsCommandList_ResourceBarrier(cl, 1, &b);
    }
    ID3D12GraphicsCommandList_Close(cl);
    ID3D12CommandQueue_ExecuteCommandLists(s->queue, 1, (ID3D12CommandList *const *)&cl);
    ID3D12CommandQueue_Signal(s->queue, s->fence, ++s->fenceValue);
    s->rbFence[k] = s->fenceValue;
}

/// Publishes the newest finished readback slot to shared memory; never waits on the GPU.
static void publish(Session *s, const XrFrameEndInfo *fi) {
    // GPU completes copies in submit order: try pending slots newest-first, publish the first one whose Map succeeds.
    int order[NSTAGE], n = 0;
    for (int k = 0; k < NSTAGE; k++) if (s->stagingReady[k]) order[n++] = k;
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && s->stagingReady[order[j]] > s->stagingReady[order[j - 1]]; j--) {
        int t = order[j]; order[j] = order[j - 1]; order[j - 1] = t;
    }
    int k = -1; uint32_t pitch = 0; void *src = NULL;
    uint64_t done = s->fence ? ID3D12Fence_GetCompletedValue(s->fence) : 0;
    D3D12_RANGE all = {0, (SIZE_T)s->rbPitch * s->stagingH}, none = {0, 0};
    for (int i = 0; i < n && k < 0; i++) {
        int c = order[i]; D3D11_MAPPED_SUBRESOURCE m;
        if (s->dev12) { if (done >= s->rbFence[c] && SUCCEEDED(ID3D12Resource_Map(s->rb[c], 0, &all, &src))) { k = c; pitch = s->rbPitch; } }
        else if (SUCCEEDED(ID3D11DeviceContext_Map(s->ctx, (ID3D11Resource *)s->staging[c], 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m))) { k = c; src = m.pData; pitch = m.RowPitch; }
    }
    if (k < 0) return;   // nothing finished yet; never block the game thread
    for (int j = 0; j < NSTAGE; j++)         // older pending copies are stale now
        if (s->stagingReady[j] && s->stagingReady[j] < s->stagingReady[k]) s->stagingReady[j] = 0;
    int64_t copyT0 = qpc_ns();
    uint32_t buf = (shm->frame_seq + 1) % 2, row = s->stagingW * 4;
    uint8_t *dst = vr4_frame(shm, buf);
    if (pitch == row) memcpy(dst, src, (size_t)row * s->stagingH);   // channel order fixed up on the Mac (frame_rgba)
    else for (uint32_t y = 0; y < s->stagingH; y++) memcpy(dst + (size_t)y * row, (const uint8_t *)src + (size_t)y * pitch, row);
    if (s->dev12) ID3D12Resource_Unmap(s->rb[k], 0, &none);
    else ID3D11DeviceContext_Unmap(s->ctx, (ID3D11Resource *)s->staging[k], 0);
    s->stagingReady[k] = 0;
    shm->frame_eye_pose[buf][0] = s->stagingPose[k][0]; shm->frame_eye_pose[buf][1] = s->stagingPose[k][1];
    shm->frame_w[buf] = s->stagingW; shm->frame_h[buf] = s->stagingH; shm->frame_rgba[buf] = (uint32_t)is_rgba(s->stagingFmt);
    shm->frame_time_ns[buf] = s->stagingDisplayTime[k];
    s->lastPublished = (uint64_t)fi->displayTime;
    copySum += qpc_ns() - copyT0;
    shm->runtime_heartbeat_ns = (uint64_t)qpc_ns();
    vr4_fence();
    shm->frame_seq++;
}

static XrResult XRAPI_CALL xrEndFrameImpl(XrSession h, const XrFrameEndInfo *fi);
static XrResult XRAPI_CALL xrEndFrame_(XrSession h, const XrFrameEndInfo *fi) {
    int64_t endT0 = qpc_ns();
    XrResult r = xrEndFrameImpl(h, fi);
    endSum += qpc_ns() - endT0;
    return r;
}
static XrResult XRAPI_CALL xrEndFrameImpl(XrSession h, const XrFrameEndInfo *fi) {
    Session *s = (Session *)h;
    if (!s || !fi) return XR_ERROR_HANDLE_INVALID;
    if (fi->displayTime && (uint64_t)fi->displayTime == s->lastPublished) {
        shm->runtime_heartbeat_ns = (uint64_t)qpc_ns();   // same frame re-submitted: no new pixels, stay alive
        return XR_SUCCESS;
    }
    int pi = -1, nover = 0;
    for (uint32_t i = 0; i < fi->layerCount && pi < 0; i++)
        if (fi->layers[i] && fi->layers[i]->type == XR_TYPE_COMPOSITION_LAYER_PROJECTION) pi = (int)i;
    for (uint32_t i = 0; i < fi->layerCount; i++) {
        const XrCompositionLayerBaseHeader *l = fi->layers[i];
        if (!l || (int)i == pi) continue;
        if (is_overlay(l)) nover += (int)i > pi;
        else {
            char name[XR_MAX_STRUCTURE_NAME_SIZE]; xrStructureTypeToString_(XR_NULL_HANDLE, l->type, name);
            logonce("%s layers are not composited", name);
        }
    }
    const XrCompositionLayerProjection *proj = pi >= 0 ? (const XrCompositionLayerProjection *)fi->layers[pi] : NULL;
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
    const VR4Tracking *ft = frame_for(fi->displayTime);
    Swapchain *sc[2] = {0}; uint32_t w, hgt; DXGI_FORMAT fmt; XrPosef eye[2]; XrFovf fov[2];
    if (proj) {
        if (proj->viewCount < 2) { logonce("projection layer with %u views", proj->viewCount); return XR_SUCCESS; }
        sc[0] = find_sc(proj->views[0].subImage.swapchain); sc[1] = find_sc(proj->views[1].subImage.swapchain);
        if (!sc[0] || !sc[1]) { logonce("xrEndFrame: invalid swapchain handle"); return XR_ERROR_HANDLE_INVALID; }
        if (sc[0]->released < 0 || sc[1]->released < 0) return XR_SUCCESS;
        w = (uint32_t)proj->views[0].subImage.imageRect.extent.width; hgt = (uint32_t)proj->views[0].subImage.imageRect.extent.height;
        if (!w || !hgt || proj->views[1].subImage.imageRect.extent.width != (int32_t)w || proj->views[1].subImage.imageRect.extent.height != (int32_t)hgt) {
            logonce("unsupported eye rects %ux%u / %dx%d", w, hgt, proj->views[1].subImage.imageRect.extent.width, proj->views[1].subImage.imageRect.extent.height);
            return XR_SUCCESS;
        }
        fmt = sc[0]->fmt;
        if (sc[1]->fmt != fmt) { logonce("mismatched swapchain formats between eyes: %d vs %d", sc[0]->fmt, sc[1]->fmt); return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED; }
        if (!is_color(fmt)) { logonce("projection swapchain format %d is not 8-bit colour", fmt); return XR_SUCCESS; }
        for (int e = 0; e < 2; e++) {   // bounds checking for rects and array indices against swapchain metadata
            const XrSwapchainSubImage *si = &proj->views[e].subImage;
            if (si->imageArrayIndex >= sc[e]->array) { logonce("eye %d imageArrayIndex %u >= %u", e, si->imageArrayIndex, sc[e]->array); return XR_ERROR_RUNTIME_FAILURE; }
            if (si->imageRect.offset.x < 0 || si->imageRect.offset.y < 0 || (uint64_t)si->imageRect.offset.x + w > sc[e]->w ||
                (uint64_t)si->imageRect.offset.y + hgt > sc[e]->h) { logonce("eye %d imageRect bounds overflow", e); return XR_ERROR_RUNTIME_FAILURE; }
        }
        int valid; XrPosef base = proj->space ? space_in_stage_at((Space *)proj->space, ft, &valid) : IDENT;
        for (int e = 0; e < 2; e++) { eye[e] = pmul(base, proj->views[e].pose); fov[e] = proj->views[e].fov; }
    } else {   // layers only (menus, players, loading screens)
        if (!nover) return XR_SUCCESS;
        w = eye_w(); hgt = eye_h(); fmt = DXGI_FORMAT_R8G8B8A8_TYPELESS;
        for (int e = 0; e < 2; e++) {
            const VR4Fov *f = &ft->eye[e].fov;
            eye[e] = xp(ft->eye[e].pose); fov[e] = (XrFovf){f->left, f->right, f->up, f->down};
        }
    }
    if (2ULL * w * hgt * 4ULL > (uint64_t)VR4_FRAME_MAX) { logonce("eye %ux%u too large for the frame buffer", w, hgt); return XR_SUCCESS; }
    if (s->dev12) {
        if (nover) logonce("quad/cylinder layers are not composited for D3D12 games");
        if (!proj || !s->fence || !ensure_readback12(s, fmt, 2 * w, hgt)) return XR_SUCCESS;
    } else {
        if (nover && !comp_init(s)) { if (!proj) return XR_SUCCESS; nover = 0; }
        if (!ensure_staging(s, fmt, 2 * w, hgt) || (nover && !ensure_target(s, fmt, 2 * w, hgt))) return XR_SUCCESS;
    }

    // Ring of NSTAGE readback slots: D3DMetal GPU latency is >1 frame, so a 2-slot flip overwrote unread copies and
    // published only ~every other frame (30 fps at 72). Copy into a free slot (or the oldest), publish the newest done.
    int k = 0;
    for (int j = 1; j < NSTAGE; j++)
        if (s->stagingReady[j] < s->stagingReady[k]) k = j;   // free (0) or oldest pending
    if (s->dev12) copy12(s, k, proj, sc, w, hgt);
    else {
        ID3D11DeviceContext *c = s->ctx;
        ID3D11Resource *dst = nover ? (ID3D11Resource *)s->target : (ID3D11Resource *)s->staging[k];
        if (proj) for (int e = 0; e < 2; e++) {   // both eyes side by side
            const XrSwapchainSubImage *si = &proj->views[e].subImage;
            D3D11_BOX box = {(UINT)si->imageRect.offset.x, (UINT)si->imageRect.offset.y, 0, (UINT)si->imageRect.offset.x + w, (UINT)si->imageRect.offset.y + hgt, 1};
            ID3D11DeviceContext_CopySubresourceRegion(c, dst, 0, e * w, 0, 0, (ID3D11Resource *)sc[e]->img[sc[e]->released], si->imageArrayIndex * sc[e]->mips, &box);
        } else {
            static const FLOAT black[4] = {0, 0, 0, 1};
            ID3D11DeviceContext_ClearRenderTargetView(c, s->rtv, black);
        }
        if (nover) {
            SavedState z = {0}; save_state(c, &z);
            UINT stride = sizeof(Vtx), off = 0;
            ID3D11DeviceContext_IASetInputLayout(c, s->il); ID3D11DeviceContext_IASetPrimitiveTopology(c, D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
            ID3D11DeviceContext_IASetVertexBuffers(c, 0, 1, &s->vb, &stride, &off);
            ID3D11DeviceContext_VSSetShader(c, s->vs, NULL, 0); ID3D11DeviceContext_HSSetShader(c, NULL, NULL, 0);
            ID3D11DeviceContext_DSSetShader(c, NULL, NULL, 0); ID3D11DeviceContext_GSSetShader(c, NULL, NULL, 0);
            ID3D11DeviceContext_PSSetShader(c, s->ps, NULL, 0); ID3D11DeviceContext_PSSetConstantBuffers(c, 0, 1, &s->cb);
            ID3D11DeviceContext_PSSetSamplers(c, 0, 1, &s->smp); ID3D11DeviceContext_RSSetState(c, s->rs);
            ID3D11DeviceContext_OMSetRenderTargets(c, 1, &s->rtv, NULL); ID3D11DeviceContext_OMSetDepthStencilState(c, NULL, 0);
            for (uint32_t i = (uint32_t)(pi + 1); i < fi->layerCount; i++)
                if (is_overlay(fi->layers[i])) draw_layer(s, fi->layers[i], ft, eye, fov, w, hgt);
            restore_state(c, &z);
            ID3D11DeviceContext_CopyResource(c, (ID3D11Resource *)s->staging[k], (ID3D11Resource *)s->target);
        }
        ID3D11DeviceContext_Flush(c);   // start the copy now, not at the game's next flush
    }
    for (int e = 0; e < 2; e++) s->stagingPose[k][e] = raw_pose(eye[e]);   // stage space: the Mac reprojects the backdrop with these
    s->stagingDisplayTime[k] = (uint64_t)fi->displayTime;
    s->stagingReady[k] = ++s->stagingCounter;
    publish(s, fi);
    return XR_SUCCESS;
}


// ---------------------------------------------------------------- swapchains
// Colour formats first (8-bit, raw bytes streamed as sRGB); depth formats serve XR_KHR_composition_layer_depth.
static const int64_t formats[] = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_B8G8R8A8_UNORM,
                                  DXGI_FORMAT_D32_FLOAT, DXGI_FORMAT_D24_UNORM_S8_UINT, DXGI_FORMAT_D16_UNORM, DXGI_FORMAT_D32_FLOAT_S8X24_UINT};
#define NFORMATS (uint32_t)(sizeof formats / sizeof *formats)
static XrResult XRAPI_CALL xrEnumerateSwapchainFormats_(XrSession h, uint32_t cap, uint32_t *n, int64_t *f) {
    (void)h; FILL_ARRAY(cap, n, f, NFORMATS, f[i_] = formats[i_]); return XR_SUCCESS;
}
static DXGI_FORMAT typeless(DXGI_FORMAT f) {   // so games can make sRGB or linear views (and depth views) of the same image
    switch (f) {
        case DXGI_FORMAT_R8G8B8A8_UNORM: case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: return DXGI_FORMAT_R8G8B8A8_TYPELESS;
        case DXGI_FORMAT_B8G8R8A8_UNORM: case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB: return DXGI_FORMAT_B8G8R8A8_TYPELESS;
        case DXGI_FORMAT_D32_FLOAT: return DXGI_FORMAT_R32_TYPELESS;
        case DXGI_FORMAT_D24_UNORM_S8_UINT: return DXGI_FORMAT_R24G8_TYPELESS;
        case DXGI_FORMAT_D16_UNORM: return DXGI_FORMAT_R16_TYPELESS;
        default: return DXGI_FORMAT_R32G8X24_TYPELESS;
    }
}
static XrResult XRAPI_CALL xrCreateSwapchain_(XrSession h, const XrSwapchainCreateInfo *ci, XrSwapchain *out) {
    Session *s = (Session *)h;
    int ok = 0;
    for (uint32_t i = 0; i < NFORMATS; i++) ok |= formats[i] == ci->format;
    if (!ok) { logonce("swapchain format %lld unsupported", (long long)ci->format); return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED; }
    if (ci->sampleCount > 1) logonce("swapchain asks %u samples: accepted without MSAA (game renders unaliased)", ci->sampleCount);
    logmsg("swapchain fmt %lld %ux%u array %u mips %u faces %u usage 0x%llx flags 0x%llx", (long long)ci->format, ci->width, ci->height,
           ci->arraySize, ci->mipCount, ci->faceCount, (unsigned long long)ci->usageFlags, (unsigned long long)ci->createFlags);
    Swapchain *sc = calloc(1, sizeof *sc);
    sc->fmt = typeless((DXGI_FORMAT)ci->format); sc->w = ci->width; sc->h = ci->height; sc->array = ci->arraySize ? ci->arraySize : 1;
    sc->mips = ci->mipCount ? ci->mipCount : 1; sc->released = -1;
    sc->count = ci->createFlags & XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT ? 1 : 3;
    if (s->dev12) {   // handed out in the states XR_KHR_D3D12_enable promises: RENDER_TARGET / DEPTH_WRITE
        int color = is_color(sc->fmt);
        D3D12_HEAP_PROPERTIES hp = {D3D12_HEAP_TYPE_DEFAULT, D3D12_CPU_PAGE_PROPERTY_UNKNOWN, D3D12_MEMORY_POOL_UNKNOWN, 0, 0};
        D3D12_RESOURCE_DESC rd = {D3D12_RESOURCE_DIMENSION_TEXTURE2D, 0, ci->width, ci->height, (UINT16)sc->array, (UINT16)sc->mips, sc->fmt, {1, 0},
            D3D12_TEXTURE_LAYOUT_UNKNOWN, !color ? D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL : D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET |
            (ci->usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT ? D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS : D3D12_RESOURCE_FLAG_NONE)};
        for (int i = 0; i < sc->count; i++)
            if (FAILED(ID3D12Device_CreateCommittedResource(s->dev12, &hp, D3D12_HEAP_FLAG_NONE, &rd, color ? D3D12_RESOURCE_STATE_RENDER_TARGET : D3D12_RESOURCE_STATE_DEPTH_WRITE,
                                                            NULL, &IID_ID3D12Resource, (void **)&sc->img12[i]))) {
                logmsg("CreateCommittedResource failed for swapchain format %lld", (long long)ci->format);
                for (int j = 0; j < i; j++) rel(sc->img12[j]);
                free(sc); return XR_ERROR_RUNTIME_FAILURE;
            }
        *out = (XrSwapchain)sc;
        return XR_SUCCESS;
    }
    UINT bind = D3D11_BIND_SHADER_RESOURCE;
    if (!is_color(sc->fmt)) bind |= D3D11_BIND_DEPTH_STENCIL;
    else if (ci->usageFlags & XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT) bind |= D3D11_BIND_RENDER_TARGET;
    if (is_color(sc->fmt) && (ci->usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT)) bind |= D3D11_BIND_UNORDERED_ACCESS;
    D3D11_TEXTURE2D_DESC d = {ci->width, ci->height, sc->mips, sc->array, sc->fmt, {1, 0}, D3D11_USAGE_DEFAULT, bind, 0, 0};
    for (int i = 0; i < sc->count; i++)
        if (FAILED(ID3D11Device_CreateTexture2D(s->dev, &d, NULL, &sc->img[i]))) {
            logmsg("CreateTexture2D failed for swapchain format %lld", (long long)ci->format);
            for (int j = 0; j < i; j++) rel(sc->img[j]);
            free(sc); return XR_ERROR_RUNTIME_FAILURE;
        }
    *out = (XrSwapchain)sc;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroySwapchain_(XrSwapchain h) {
    Swapchain *sc = find_sc(h);
    for (int i = 0; i < sc->count; i++) { rel(sc->img[i]); rel(sc->img12[i]); }
    free(sc); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateSwapchainImages_(XrSwapchain h, uint32_t cap, uint32_t *n, XrSwapchainImageBaseHeader *imgs) {
    Swapchain *sc = find_sc(h);
    if (sc->img12[0]) {
        XrSwapchainImageD3D12KHR *d = (XrSwapchainImageD3D12KHR *)imgs;
        FILL_ARRAY(cap, n, d, (uint32_t)sc->count, d[i_].texture = sc->img12[i_]);
        return XR_SUCCESS;
    }
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
    int j = 0;   // a later call for the same profile replaces its earlier bindings
    for (int k = 0; k < nsugg; k++) if (sugg[k].profile != sb->interactionProfile) sugg[j++] = sugg[k];
    nsugg = j;
    for (uint32_t k = 0; k < sb->countSuggestedBindings && nsugg < 1024; k++)
        sugg[nsugg++] = (Suggestion){sb->interactionProfile, (Action *)sb->suggestedBindings[k].action, sb->suggestedBindings[k].binding};
    logmsg("suggested %u bindings for %s%s", sb->countSuggestedBindings, pstr(sb->interactionProfile),
           profile_index(sb->interactionProfile) < 0 ? " (unknown profile: used like Touch if nothing better is suggested)" : "");
    return XR_SUCCESS;
}
/// XR_EXT_hand_interaction: a hand that put its controller down switches to the hand-interaction profile (if suggested).
static void update_hand_profiles(Session *s, int notify) {
    int changed = 0;
    for (int h = 0; h < 2; h++) {
        int hi = s->handProfile && (!s->profile || hand_tracked(h));
        changed |= hi != s->handHi[h]; s->handHi[h] = hi;
    }
    if (changed && notify) ((XrEventDataInteractionProfileChanged *)new_event(XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED))->session = (XrSession)s;
}
static XrResult XRAPI_CALL xrAttachSessionActionSets_(XrSession h, const XrSessionActionSetsAttachInfo *ai) {
    Session *s = (Session *)h; (void)ai;
    int best = NPROF + 1;   // profile table order; unknown profiles after all known controllers
    s->profile = s->handProfile = XR_NULL_PATH;
    for (int k = 0; k < nsugg; k++) {
        int i = profile_index(sugg[k].profile), rank = i < 0 ? NPROF : i;
        if (i >= 0 && (profiles[i].flags & P_HAND)) s->handProfile = sugg[k].profile;
        else if (rank < best) { best = rank; s->profile = sugg[k].profile; }
    }
    for (int k = 0; k < nsugg; k++) sugg[k].action->nb = 0;
    int nb = 0;
    for (int k = 0; k < nsugg; k++) {
        Action *a = sugg[k].action; XrPath p = sugg[k].profile;
        int hand, out; const char *comp;
        if ((p != s->profile && p != s->handProfile) || a->nb >= 32 || !parse_binding(pstr(sugg[k].binding), &hand, &comp, &out)) continue;
        int src = resolve(comp, profile_flags(p), hand, out);
        if (src == S_NONE) { logonce("%s: nothing on the Quest feeds %s (stays idle)", pstr(p), pstr(sugg[k].binding)); continue; }
        a->b[a->nb++] = (Binding){sugg[k].binding, (int8_t)hand, (uint8_t)src, p == s->handProfile};
        nb++;
    }
    update_hand_profiles(s, 0);
    logmsg("attached %d bindings, profile %s, hands %s", nb, s->profile ? pstr(s->profile) : "none", s->handProfile ? pstr(s->handProfile) : "none");
    ((XrEventDataInteractionProfileChanged *)new_event(XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED))->session = h;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetCurrentInteractionProfile_(XrSession h, XrPath user, XrInteractionProfileState *st) {
    Session *s = (Session *)h;
    if (!st) return XR_ERROR_VALIDATION_FAILURE;
    int hand = user == handPath[0] ? 0 : user == handPath[1] ? 1 : -1;
    st->interactionProfile = hand < 0 ? XR_NULL_PATH : s->handHi[hand] ? s->handProfile : s->profile;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrSyncActions_(XrSession h, const XrActionsSyncInfo *si) {
    (void)si; Session *s = (Session *)h;
    read_tracking();
    syncGen++;
    update_hand_profiles(s, 1);
    return s->focused ? XR_SUCCESS : XR_SESSION_NOT_FOCUSED;
}
static int active(Session *s) { return s->focused && !shm->input_blocked; }
static float action_value(Session *s, Action *a, XrPath sub, XrVector2f *v2, int *bound) {
    float best = 0; *bound = 0;
    if (v2) *v2 = (XrVector2f){0, 0};
    for (int i = 0; i < a->nb; i++) {
        Binding *b = &a->b[i];
        if (!in_use(s, b) || !sub_matches(sub, b->hand)) continue;
        *bound = 1;
        if (!active(s)) continue;
        const VR4Hand *hh = &track.hand[b->hand];
        if (v2) {
            if (b->src == S_STICK && fabsf(hh->stick_x) + fabsf(hh->stick_y) > fabsf(v2->x) + fabsf(v2->y)) *v2 = (XrVector2f){hh->stick_x, hh->stick_y};
            continue;
        }
        float v = src_value(b->hand, b->src);
        if (fabsf(v) > fabsf(best)) best = v;
    }
    return best;
}
static int slot(XrPath sub) { return sub == handPath[0] ? 0 : sub == handPath[1] ? 1 : 2; }
/// changedSinceLastSync: compares this sync's value with the previous sync's, stable across repeated reads.
static int changed(Action *a, XrPath sub, float x, float y) {
    ActionHistory *h = &a->hist[slot(sub)];
    if (h->gen != syncGen) { h->prev[0] = h->cur[0]; h->prev[1] = h->cur[1]; h->cur[0] = x; h->cur[1] = y; h->gen = syncGen; }
    return h->cur[0] != h->prev[0] || h->cur[1] != h->prev[1];
}
static XrResult XRAPI_CALL xrGetActionStateBoolean_(XrSession h, const XrActionStateGetInfo *gi, XrActionStateBoolean *st) {
    if (!gi || !st) return XR_ERROR_VALIDATION_FAILURE;
    if (!gi->action) return XR_ERROR_HANDLE_INVALID;
    int bound; float v = action_value((Session *)h, (Action *)gi->action, gi->subactionPath, NULL, &bound);
    XrBool32 now = v > 0.5f;
    st->changedSinceLastSync = changed((Action *)gi->action, gi->subactionPath, (float)now, 0);
    st->currentState = now; st->isActive = bound; st->lastChangeTime = 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetActionStateFloat_(XrSession h, const XrActionStateGetInfo *gi, XrActionStateFloat *st) {
    if (!gi || !st) return XR_ERROR_VALIDATION_FAILURE;
    if (!gi->action) return XR_ERROR_HANDLE_INVALID;
    int bound; float v = action_value((Session *)h, (Action *)gi->action, gi->subactionPath, NULL, &bound);
    st->changedSinceLastSync = changed((Action *)gi->action, gi->subactionPath, v, 0); st->currentState = v; st->isActive = bound; st->lastChangeTime = 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetActionStateVector2f_(XrSession h, const XrActionStateGetInfo *gi, XrActionStateVector2f *st) {
    if (!gi || !st) return XR_ERROR_VALIDATION_FAILURE;
    if (!gi->action) return XR_ERROR_HANDLE_INVALID;
    int bound; XrVector2f v; action_value((Session *)h, (Action *)gi->action, gi->subactionPath, &v, &bound);
    st->changedSinceLastSync = changed((Action *)gi->action, gi->subactionPath, v.x, v.y);
    st->currentState = v; st->isActive = bound; st->lastChangeTime = 0;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrGetActionStatePose_(XrSession h, const XrActionStateGetInfo *gi, XrActionStatePose *st) {
    if (!gi || !st) return XR_ERROR_VALIDATION_FAILURE;
    if (!gi->action) return XR_ERROR_HANDLE_INVALID;
    Action *a = (Action *)gi->action;
    st->isActive = XR_FALSE;
    for (int i = 0; i < a->nb; i++) {
        Binding *b = &a->b[i];
        if (IS_POSE(b->src) && in_use((Session *)h, b) && sub_matches(gi->subactionPath, b->hand) && (track.hand[b->hand].flags & VR4_HAND_ACTIVE))
            st->isActive = XR_TRUE;
    }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrEnumerateBoundSourcesForAction_(XrSession h, const XrBoundSourcesForActionEnumerateInfo *ei, uint32_t cap, uint32_t *n, XrPath *out) {
    if (!ei || (!out && cap > 0)) return XR_ERROR_VALIDATION_FAILURE;
    if (!ei->action) return XR_ERROR_HANDLE_INVALID;
    Action *a = (Action *)ei->action;
    XrPath src[32]; uint32_t ns = 0;
    for (int i = 0; i < a->nb; i++) if (in_use((Session *)h, &a->b[i])) src[ns++] = a->b[i].path;
    FILL_ARRAY(cap, n, out, ns, out[i_] = src[i_]);
    return XR_SUCCESS;
}
/// Names the physical Quest control a source is fed from ("Left Hand Touch Controller X Button" for an Index left "a").
static XrResult XRAPI_CALL xrGetInputSourceLocalizedName_(XrSession h, const XrInputSourceLocalizedNameGetInfo *gi, uint32_t cap, uint32_t *n, char *buf) {
    Session *s = (Session *)h;
    if (!gi || !n || !gi->whichComponents) return XR_ERROR_VALIDATION_FAILURE;
    int hand, out; const char *comp;
    if (!parse_binding(pstr(gi->sourcePath), &hand, &comp, &out)) return XR_ERROR_PATH_UNSUPPORTED;
    static const char *const names[] = {[S_TRIGGER] = "Trigger", [S_TRIGGER_CLICK] = "Trigger", [S_TRIGGER_TOUCH] = "Trigger Touch",
        [S_SQUEEZE] = "Grip", [S_SQUEEZE_CLICK] = "Grip", [S_STICK] = "Thumbstick", [S_STICK_X] = "Thumbstick", [S_STICK_Y] = "Thumbstick",
        [S_STICK_CLICK] = "Thumbstick Click", [S_STICK_TOUCH] = "Thumbstick Touch", [S_THUMB_TOUCH] = "Thumb Rest", [S_THUMB_NEAR] = "Thumb Proximity",
        [S_MENU] = "Menu Button", [S_SYSTEM] = "Menu Button", [S_PAD_CLICK] = "Thumbstick Click", [S_PINCH_READY] = "Pinch Ready",
        [S_GRASP] = "Grasp", [S_GRASP_READY] = "Grasp Ready", [S_HAPTIC] = "Haptics", [S_AIM] = "Aim", [S_GRIP] = "Grip",
        [S_PINCH_POSE] = "Pinch", [S_POKE_POSE] = "Index Fingertip", [S_POKE_POSE + 1] = NULL};
    int hi = s->handHi[hand], src = resolve(comp, profile_flags(hi ? s->handProfile : s->profile), hand, out);
    const char *part[3] = {gi->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_USER_PATH_BIT ? (hand ? "Right Hand" : "Left Hand") : NULL,
                           gi->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_INTERACTION_PROFILE_BIT ? (hi ? "Hand Tracking" : "Touch Controller") : NULL,
                           gi->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_COMPONENT_BIT ?
                               (src == S_LOWER ? (hand ? "A Button" : "X Button") : src == S_UPPER ? (hand ? "B Button" : "Y Button") : names[src] ? names[src] : comp) : NULL};
    char name[256] = "";
    for (int i = 0; i < 3; i++) if (part[i]) snprintf(name + strlen(name), sizeof name - strlen(name), "%s%s", *name ? " " : "", part[i]);
    uint32_t len = (uint32_t)strlen(name) + 1;
    *n = len;
    if (!cap) return XR_SUCCESS;
    if (cap < len) return XR_ERROR_SIZE_INSUFFICIENT;
    memcpy(buf, name, len); return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrApplyHapticFeedback_(XrSession h, const XrHapticActionInfo *hi, const XrHapticBaseHeader *hb) {
    if (hb->type != XR_TYPE_HAPTIC_VIBRATION) return XR_SUCCESS;
    const XrHapticVibration *v = (const XrHapticVibration *)hb;
    Action *a = (Action *)hi->action;
    for (int i = 0; i < a->nb; i++) if (a->b[i].src == S_HAPTIC && in_use((Session *)h, &a->b[i]) && sub_matches(hi->subactionPath, a->b[i].hand)) {
        float dur = v->duration <= 0 ? 0.02f : (float)v->duration / 1e9f;
        shm->haptic = (VR4Haptics){(uint8_t)a->b[i].hand, v->amplitude, dur, v->frequency > 0 ? v->frequency : 0};
        vr4_fence(); shm->haptic_seq++;
    }
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrStopHapticFeedback_(XrSession h, const XrHapticActionInfo *hi) {
    Action *a = (Action *)hi->action;
    for (int i = 0; i < a->nb; i++) if (a->b[i].src == S_HAPTIC && in_use((Session *)h, &a->b[i]) && sub_matches(hi->subactionPath, a->b[i].hand)) {
        shm->haptic = (VR4Haptics){(uint8_t)a->b[i].hand, 0.0f, 0.0f, 0.0f};
        vr4_fence(); shm->haptic_seq++;
    }
    return XR_SUCCESS;
}

// ---------------------------------------------------------------- hand tracking
// XR_EXT_hand_tracking from the Quest's own 26 joints (shm hand_joints), with velocities from consecutive samples,
// typical joint radii, XR_FB_hand_tracking_aim (the Mac's shoulder-to-pinch ray, pinch strengths) and
// XR_EXT_hand_tracking_data_source (always unobstructed: a hand holding a controller reports inactive).
static const float jointRadius[XR_HAND_JOINT_COUNT_EXT] = {0.022f, 0.020f,   // palm, wrist
    0.019f, 0.012f, 0.010f, 0.009f, 0.021f, 0.011f, 0.009f, 0.008f, 0.007f, 0.021f, 0.011f, 0.009f, 0.008f, 0.007f,   // thumb, index, middle
    0.019f, 0.010f, 0.008f, 0.007f, 0.006f, 0.017f, 0.009f, 0.007f, 0.006f, 0.005f};                                  // ring, little
static XrResult XRAPI_CALL xrCreateHandTrackerEXT_(XrSession h, const XrHandTrackerCreateInfoEXT *ci, XrHandTrackerEXT *out) {
    (void)h;
    if (!ci || !out || (ci->hand != XR_HAND_LEFT_EXT && ci->hand != XR_HAND_RIGHT_EXT)) return XR_ERROR_VALIDATION_FAILURE;
    if (ci->handJointSet != XR_HAND_JOINT_SET_DEFAULT_EXT) { logonce("hand joint set %d unsupported", ci->handJointSet); return XR_ERROR_VALIDATION_FAILURE; }
    int unobstructed = 1;
    for (const XrBaseInStructure *n = ci->next; n; n = n->next)
        if (n->type == XR_TYPE_HAND_TRACKING_DATA_SOURCE_INFO_EXT) {
            const XrHandTrackingDataSourceInfoEXT *d = (const XrHandTrackingDataSourceInfoEXT *)n;
            if (!d->requestedDataSourceCount) return XR_ERROR_VALIDATION_FAILURE;
            unobstructed = 0;
            for (uint32_t i = 0; i < d->requestedDataSourceCount; i++) unobstructed |= d->requestedDataSources[i] == XR_HAND_TRACKING_DATA_SOURCE_UNOBSTRUCTED_EXT;
        }
    HandTracker *t = calloc(1, sizeof *t);
    t->hand = ci->hand == XR_HAND_RIGHT_EXT; t->unobstructed = unobstructed;
    *out = (XrHandTrackerEXT)t;
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroyHandTrackerEXT_(XrHandTrackerEXT t) { free(t); return XR_SUCCESS; }
static XrResult XRAPI_CALL xrLocateHandJointsEXT_(XrHandTrackerEXT ht, const XrHandJointsLocateInfoEXT *li, XrHandJointLocationsEXT *locs) {
    HandTracker *t = (HandTracker *)ht;
    if (!t || !li || !li->baseSpace) return XR_ERROR_HANDLE_INVALID;
    if (!locs || locs->jointCount != XR_HAND_JOINT_COUNT_EXT || !locs->jointLocations) return XR_ERROR_VALIDATION_FAILURE;
    XrHandJointVelocitiesEXT *vel = NULL; XrHandTrackingAimStateFB *aim = NULL; XrHandTrackingDataSourceStateEXT *src = NULL;
    for (XrBaseOutStructure *n = locs->next; n; n = n->next) {
        if (n->type == XR_TYPE_HAND_JOINT_VELOCITIES_EXT) vel = (XrHandJointVelocitiesEXT *)n;
        if (n->type == XR_TYPE_HAND_TRACKING_AIM_STATE_FB) aim = (XrHandTrackingAimStateFB *)n;
        if (n->type == XR_TYPE_HAND_TRACKING_DATA_SOURCE_STATE_EXT) src = (XrHandTrackingDataSourceStateEXT *)n;
    }
    if (vel && (vel->jointCount != XR_HAND_JOINT_COUNT_EXT || !vel->jointVelocities)) return XR_ERROR_VALIDATION_FAILURE;
    int h = t->hand, bv;
    XrPosef base = space_in_stage((Space *)li->baseSpace, &bv), inv = pinv(base);
    int on = t->unobstructed && hand_tracked(h) && bv && theSession && active(theSession);
    float ws = current_world_scale(), dt = (float)(jointsNs - prevJointsNs) * 1e-9f;
    int velOk = on && prevJoints[h].tracked && dt > 0.002f && dt < 0.15f;
    locs->isActive = on;
    for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
        XrHandJointLocationEXT *L = &locs->jointLocations[i];
        XrPosef p = xp(joints[h].joint[i]);
        L->locationFlags = on ? XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT |
                                XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT : 0;
        L->pose = on ? pmul(inv, p) : IDENT;
        L->radius = on ? jointRadius[i] / ws : 0;
        if (!vel) continue;
        XrHandJointVelocityEXT *jv = &vel->jointVelocities[i];
        *jv = (XrHandJointVelocityEXT){0};
        if (!velOk) continue;
        V lin, ang; pose_velocity(xp(prevJoints[h].joint[i]), p, dt, &lin, &ang);
        jv->linearVelocity = qrot(inv.orientation, lin); jv->angularVelocity = qrot(inv.orientation, ang);
        jv->velocityFlags = XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
    }
    if (src) { src->isActive = on; src->dataSource = XR_HAND_TRACKING_DATA_SOURCE_UNOBSTRUCTED_EXT; }
    if (aim) {
        const VR4Hand *vh = &track.hand[h];
        aim->status = 0; aim->aimPose = IDENT;
        aim->pinchStrengthIndex = aim->pinchStrengthMiddle = aim->pinchStrengthRing = aim->pinchStrengthLittle = 0;
        if (on) {
            const VR4Pose *palm = &joints[h].joint[XR_HAND_JOINT_PALM_EXT];
            V toHead = {track.head.px - palm->px, track.head.py - palm->py, track.head.pz - palm->pz};
            V normal = qrot((Q){palm->qx, palm->qy, palm->qz, palm->qw}, (V){0, -1, 0});   // palm normal is -Y
            float facing = vlen(toHead) > 0 ? (normal.x * toHead.x + normal.y * toHead.y + normal.z * toHead.z) / vlen(toHead) : 0;
            int pinched = vh->trigger > 0.5f;   // the Mac's pinch (with hysteresis) is the trigger
            aim->pinchStrengthIndex = pinched ? 1 : pinch_strength(h, XR_HAND_JOINT_INDEX_TIP_EXT);
            aim->pinchStrengthMiddle = pinch_strength(h, XR_HAND_JOINT_MIDDLE_TIP_EXT);
            aim->pinchStrengthRing = pinch_strength(h, XR_HAND_JOINT_RING_TIP_EXT);
            aim->pinchStrengthLittle = pinch_strength(h, XR_HAND_JOINT_LITTLE_TIP_EXT);
            aim->status = XR_HAND_TRACKING_AIM_COMPUTED_BIT_FB | (facing > 0.6f ? XR_HAND_TRACKING_AIM_SYSTEM_GESTURE_BIT_FB : XR_HAND_TRACKING_AIM_VALID_BIT_FB) |
                          (pinched ? XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB : 0) |
                          (aim->pinchStrengthMiddle >= 1 ? XR_HAND_TRACKING_AIM_MIDDLE_PINCHING_BIT_FB : 0) |
                          (aim->pinchStrengthRing >= 1 ? XR_HAND_TRACKING_AIM_RING_PINCHING_BIT_FB : 0) |
                          (aim->pinchStrengthLittle >= 1 ? XR_HAND_TRACKING_AIM_LITTLE_PINCHING_BIT_FB : 0) |
                          (h ? XR_HAND_TRACKING_AIM_DOMINANT_HAND_BIT_FB : 0) | (vh->buttons & VR4_BTN_MENU ? XR_HAND_TRACKING_AIM_MENU_PRESSED_BIT_FB : 0);
            aim->aimPose = pmul(inv, xp(vh->aim));
        }
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
/// Nothing is hidden: the hidden mesh is empty and the visible mesh / outline cover the whole view (tangent space).
static XrResult XRAPI_CALL xrGetVisibilityMaskKHR_(XrSession h, XrViewConfigurationType t, uint32_t view, XrVisibilityMaskTypeKHR mt, XrVisibilityMaskKHR *m) {
    (void)h;
    if (t != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    if (view > 1 || !m) return XR_ERROR_VALIDATION_FAILURE;
    const VR4Fov *f = &frame_for(0)->eye[view].fov;
    XrVector2f v[4] = {{tanf(f->left), tanf(f->down)}, {tanf(f->right), tanf(f->down)}, {tanf(f->right), tanf(f->up)}, {tanf(f->left), tanf(f->up)}};
    static const uint32_t tri[6] = {0, 1, 2, 0, 2, 3}, loop[4] = {0, 1, 2, 3};   // counter-clockwise
    int hidden = mt == XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR, line = mt == XR_VISIBILITY_MASK_TYPE_LINE_LOOP_KHR;
    uint32_t nv = hidden ? 0 : 4, ni = hidden ? 0 : line ? 4 : 6;
    m->vertexCountOutput = nv; m->indexCountOutput = ni;
    if (!m->vertexCapacityInput && !m->indexCapacityInput) return XR_SUCCESS;
    if (m->vertexCapacityInput < nv || m->indexCapacityInput < ni) return XR_ERROR_SIZE_INSUFFICIENT;
    memcpy(m->vertices, v, nv * sizeof *v); memcpy(m->indices, line ? loop : tri, ni * sizeof *tri);
    return XR_SUCCESS;
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
static XrResult XRAPI_CALL xrPerfSettingsSetPerformanceLevelEXT_(XrSession h, XrPerfSettingsDomainEXT d, XrPerfSettingsLevelEXT l) {
    (void)h;   // a hint: the Mac's GPU and the headset's clocks aren't ours to set
    if (d != XR_PERF_SETTINGS_DOMAIN_CPU_EXT && d != XR_PERF_SETTINGS_DOMAIN_GPU_EXT) return XR_ERROR_VALIDATION_FAILURE;
    logonce("performance level %d requested for %s (hint only)", l, d == XR_PERF_SETTINGS_DOMAIN_CPU_EXT ? "CPU" : "GPU");
    return XR_SUCCESS;
}

static XrResult XRAPI_CALL xrSetDebugUtilsObjectNameEXT_(XrInstance i, const XrDebugUtilsObjectNameInfoEXT *n) { (void)i; (void)n; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrCreateDebugUtilsMessengerEXT_(XrInstance i, const XrDebugUtilsMessengerCreateInfoEXT *ci, XrDebugUtilsMessengerEXT *out) {
    (void)i; (void)ci; *out = (XrDebugUtilsMessengerEXT)(uintptr_t)1; return XR_SUCCESS;
}
static XrResult XRAPI_CALL xrDestroyDebugUtilsMessengerEXT_(XrDebugUtilsMessengerEXT m) { (void)m; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrSessionBeginDebugUtilsLabelRegionEXT_(XrSession s, const XrDebugUtilsLabelEXT *l) { (void)s; (void)l; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrSessionEndDebugUtilsLabelRegionEXT_(XrSession s) { (void)s; return XR_SUCCESS; }
static XrResult XRAPI_CALL xrSessionInsertDebugUtilsLabelEXT_(XrSession s, const XrDebugUtilsLabelEXT *l) { (void)s; (void)l; return XR_SUCCESS; }

// ---------------------------------------------------------------- dispatch
static XrResult XRAPI_CALL xrGetInstanceProcAddr_(XrInstance inst, const char *name, PFN_xrVoidFunction *fn);
static const struct { const char *name; PFN_xrVoidFunction fn; } table[] = {
#define F(n) {#n, (PFN_xrVoidFunction)n##_},
    F(xrGetInstanceProcAddr) F(xrEnumerateApiLayerProperties) F(xrEnumerateInstanceExtensionProperties) F(xrCreateInstance)
    F(xrDestroyInstance) F(xrGetInstanceProperties) F(xrPollEvent) F(xrResultToString) F(xrStructureTypeToString)
    F(xrStringToPath) F(xrPathToString) F(xrGetSystem) F(xrGetSystemProperties) F(xrEnumerateEnvironmentBlendModes)
    F(xrEnumerateViewConfigurations) F(xrGetViewConfigurationProperties) F(xrEnumerateViewConfigurationViews)
    F(xrGetD3D11GraphicsRequirementsKHR) F(xrGetD3D12GraphicsRequirementsKHR) F(xrConvertWin32PerformanceCounterToTimeKHR) F(xrConvertTimeToWin32PerformanceCounterKHR)
    F(xrCreateSession) F(xrDestroySession) F(xrBeginSession) F(xrEndSession) F(xrRequestExitSession)
    F(xrEnumerateReferenceSpaces) F(xrCreateReferenceSpace) F(xrGetReferenceSpaceBoundsRect) F(xrCreateActionSpace)
    F(xrLocateSpace) F(xrLocateSpaces) F(xrDestroySpace) F(xrWaitFrame) F(xrBeginFrame) F(xrEndFrame) F(xrLocateViews)
    F(xrEnumerateSwapchainFormats) F(xrCreateSwapchain) F(xrDestroySwapchain) F(xrEnumerateSwapchainImages)
    F(xrAcquireSwapchainImage) F(xrWaitSwapchainImage) F(xrReleaseSwapchainImage)
    F(xrCreateActionSet) F(xrDestroyActionSet) F(xrCreateAction) F(xrDestroyAction) F(xrSuggestInteractionProfileBindings)
    F(xrAttachSessionActionSets) F(xrGetCurrentInteractionProfile) F(xrSyncActions) F(xrGetActionStateBoolean)
    F(xrGetActionStateFloat) F(xrGetActionStateVector2f) F(xrGetActionStatePose) F(xrEnumerateBoundSourcesForAction)
    F(xrGetInputSourceLocalizedName) F(xrApplyHapticFeedback) F(xrStopHapticFeedback)
    F(xrCreateHandTrackerEXT) F(xrDestroyHandTrackerEXT) F(xrLocateHandJointsEXT)
    F(xrEnumerateDisplayRefreshRatesFB) F(xrGetDisplayRefreshRateFB) F(xrRequestDisplayRefreshRateFB) F(xrGetVisibilityMaskKHR)
    F(xrEnumerateColorSpacesFB) F(xrSetColorSpaceFB) F(xrPerfSettingsSetPerformanceLevelEXT)
    F(xrSetDebugUtilsObjectNameEXT) F(xrCreateDebugUtilsMessengerEXT) F(xrDestroyDebugUtilsMessengerEXT)
    F(xrSessionBeginDebugUtilsLabelRegionEXT) F(xrSessionEndDebugUtilsLabelRegionEXT) F(xrSessionInsertDebugUtilsLabelEXT)
#undef F
    {"xrLocateSpacesKHR", (PFN_xrVoidFunction)xrLocateSpaces_},
};
static XrResult XRAPI_CALL xrGetInstanceProcAddr_(XrInstance inst, const char *name, PFN_xrVoidFunction *fn) {
    (void)inst;
    if (!name || !fn) return XR_ERROR_VALIDATION_FAILURE;
    for (size_t i = 0; i < sizeof table / sizeof table[0]; i++)
        if (!strcmp(name, table[i].name)) { *fn = table[i].fn; return XR_SUCCESS; }
    *fn = NULL;
    logonce("unsupported function %s (the game may lose that feature)", name);
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
