// Smoke test: behaves like a minimal D3D11 OpenXR game against vr4mac_openxr.dll, then checks the
// pixels landed in shared memory. Run under Wine: wine64 test_xr.exe  (exit code 0 = pass)
#define COBJMACROS
#include <windows.h>
#include <d3d11.h>
#include <stdio.h>
#include <stdlib.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include "../common/vr4mac.h"

#define CHECK(x) do { XrResult r_ = (x); if (XR_FAILED(r_)) { printf("FAIL %s = %d\n", #x, r_); return 1; } } while (0)
static PFN_xrGetInstanceProcAddr gipa;
static XrInstance inst;
#define FN(name) PFN_##name name; gipa(inst, #name, (PFN_xrVoidFunction *)&name)

int main(int argc, char **argv) {
    // Sikarugir engine wine disconnects console stdout: VR4_TEST_LOG=C:\path\file redirects it.
    const char *logf = getenv("VR4_TEST_LOG");
    if (logf) freopen(logf, "w", stdout);
    int frames = argc > 1 ? atoi(argv[1]) : 10;
    HMODULE dll = LoadLibraryA("vr4mac_openxr.dll");
    if (!dll) { printf("FAIL load dll\n"); return 1; }
    PFN_xrNegotiateLoaderRuntimeInterface neg = (PFN_xrNegotiateLoaderRuntimeInterface)GetProcAddress(dll, "xrNegotiateLoaderRuntimeInterface");
    XrNegotiateLoaderInfo li = {XR_LOADER_INTERFACE_STRUCT_LOADER_INFO, XR_LOADER_INFO_STRUCT_VERSION, sizeof li, 1, 1, XR_MAKE_VERSION(1, 0, 0), XR_MAKE_VERSION(1, 1, 0)};
    XrNegotiateRuntimeRequest rr = {XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST, XR_RUNTIME_INFO_STRUCT_VERSION, sizeof rr};
    CHECK(neg(&li, &rr));
    gipa = rr.getInstanceProcAddr;
    FN(xrCreateInstance);
    const char *ext[] = {XR_KHR_D3D11_ENABLE_EXTENSION_NAME};
    XrInstanceCreateInfo ci = {XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(ci.applicationInfo.applicationName, "VR4Mac smoke test"); ci.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ci.enabledExtensionCount = 1; ci.enabledExtensionNames = ext;
    CHECK(xrCreateInstance(&ci, &inst));
    FN(xrGetSystem); FN(xrEnumerateViewConfigurationViews); FN(xrGetD3D11GraphicsRequirementsKHR); FN(xrCreateSession);
    FN(xrBeginSession); FN(xrCreateSwapchain); FN(xrDestroySwapchain); FN(xrEnumerateSwapchainImages); FN(xrAcquireSwapchainImage);
    FN(xrWaitSwapchainImage); FN(xrReleaseSwapchainImage); FN(xrWaitFrame); FN(xrBeginFrame); FN(xrEndFrame);
    FN(xrLocateViews); FN(xrCreateReferenceSpace); FN(xrPollEvent);
    XrSystemGetInfo sg = {XR_TYPE_SYSTEM_GET_INFO, NULL, XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    XrSystemId sys; CHECK(xrGetSystem(inst, &sg, &sys));
    XrViewConfigurationView vcv[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}}; uint32_t n;
    CHECK(xrEnumerateViewConfigurationViews(inst, sys, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &n, vcv));
    XrGraphicsRequirementsD3D11KHR req = {XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    CHECK(xrGetD3D11GraphicsRequirementsKHR(inst, sys, &req));
    ID3D11Device *dev; ID3D11DeviceContext *ctx;
    if (FAILED(D3D11CreateDevice(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, NULL, 0, D3D11_SDK_VERSION, &dev, NULL, &ctx))) { printf("FAIL D3D11CreateDevice\n"); return 1; }
    XrGraphicsBindingD3D11KHR gb = {XR_TYPE_GRAPHICS_BINDING_D3D11_KHR, NULL, dev};
    XrSessionCreateInfo sci = {XR_TYPE_SESSION_CREATE_INFO, &gb, 0, sys};
    XrSession ses; CHECK(xrCreateSession(inst, &sci, &ses));
    XrEventDataBuffer ev = {XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(inst, &ev) == XR_SUCCESS) ev.type = XR_TYPE_EVENT_DATA_BUFFER;
    XrSessionBeginInfo bi = {XR_TYPE_SESSION_BEGIN_INFO, NULL, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
    CHECK(xrBeginSession(ses, &bi));
    XrReferenceSpaceCreateInfo rs = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO, NULL, XR_REFERENCE_SPACE_TYPE_STAGE, {{0, 0, 0, 1}, {0, 0, 0}}};
    XrSpace stage; CHECK(xrCreateReferenceSpace(ses, &rs, &stage));
    uint32_t w = vcv[0].recommendedImageRectWidth, h = vcv[0].recommendedImageRectHeight;
    XrSwapchainCreateInfo sc = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 1, w * 2, h, 1, 1, 1};
    XrSwapchain swap; CHECK(xrCreateSwapchain(ses, &sc, &swap));
    // MSAA swapchains are accepted (rendered unaliased) instead of rejected
    XrSwapchainCreateInfo scMS = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 2, w, h, 1, 1, 1};
    XrSwapchain swapMS; CHECK(xrCreateSwapchain(ses, &scMS, &swapMS)); CHECK(xrDestroySwapchain(swapMS));
    XrSwapchainImageD3D11KHR imgs[3] = {{XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}};
    CHECK(xrEnumerateSwapchainImages(swap, 3, &n, (XrSwapchainImageBaseHeader *)imgs));

    HANDLE f = CreateFileA(VR4_SHM_PATH_WIN, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
    VR4Shm *shm = MapViewOfFile(CreateFileMappingA(f, NULL, PAGE_READONLY, 0, VR4_SHM_SIZE, NULL), FILE_MAP_READ, 0, 0, VR4_SHM_SIZE);
    uint32_t seq0 = shm->frame_seq;

    XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}}; XrTime lastT = 0;
    for (int frame = 0; frame < frames; frame++) {
        XrFrameState fs = {XR_TYPE_FRAME_STATE};
        CHECK(xrWaitFrame(ses, NULL, &fs)); CHECK(xrBeginFrame(ses, NULL));
        lastT = fs.predictedDisplayTime;
        XrViewState vs = {XR_TYPE_VIEW_STATE};
        XrViewLocateInfo vl = {XR_TYPE_VIEW_LOCATE_INFO, NULL, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, fs.predictedDisplayTime, stage};
        CHECK(xrLocateViews(ses, &vl, &vs, 2, &n, views));
        uint32_t idx; CHECK(xrAcquireSwapchainImage(swap, NULL, &idx));
        XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, NULL, XR_INFINITE_DURATION}; CHECK(xrWaitSwapchainImage(swap, &wi));
        D3D11_RENDER_TARGET_VIEW_DESC rd = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, D3D11_RTV_DIMENSION_TEXTURE2D};
        ID3D11RenderTargetView *rtv;
        if (FAILED(ID3D11Device_CreateRenderTargetView(dev, (ID3D11Resource *)imgs[idx].texture, &rd, &rtv))) { printf("FAIL RTV\n"); return 1; }
        float red[4] = {1, 0, 0, 1};
        ID3D11DeviceContext_ClearRenderTargetView(ctx, rtv, red);
        ID3D11RenderTargetView_Release(rtv);
        CHECK(xrReleaseSwapchainImage(swap, NULL));
        XrCompositionLayerProjectionView pv[2];
        for (int e = 0; e < 2; e++) {
            XrCompositionLayerProjectionView v = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW, NULL, views[e].pose, views[e].fov, {swap, {{(int32_t)(e * w), 0}, {(int32_t)w, (int32_t)h}}, 0}};
            pv[e] = v;
        }
        XrCompositionLayerProjection proj = {XR_TYPE_COMPOSITION_LAYER_PROJECTION, NULL, 0, stage, 2, pv};
        const XrCompositionLayerBaseHeader *layers[] = {(XrCompositionLayerBaseHeader *)&proj};
        XrFrameEndInfo fe = {XR_TYPE_FRAME_END_INFO, NULL, fs.predictedDisplayTime, XR_ENVIRONMENT_BLEND_MODE_OPAQUE, 1, layers};
        CHECK(xrEndFrame(ses, &fe));
    }
    uint32_t b = shm->frame_seq % 2;
    const uint32_t *px = (const uint32_t *)((const uint8_t *)shm + VR4_FRAME_OFFSET + b * VR4_FRAME_MAX);
    printf("eye %ux%u, frames %u, frame %ux%u, first px %08x, right-eye px %08x\n", w, h, shm->frame_seq - seq0,
           shm->frame_w[b], shm->frame_h[b], px[0], px[w + 5]);
    int ok = shm->frame_seq - seq0 == (uint32_t)frames && px[0] == 0xffff0000u && px[w + 5] == 0xffff0000u && shm->frame_w[b] == 2 * w;
    if (!ok) { printf("FAIL projection\n"); return 1; }

    // Re-submitting the same displayTime publishes nothing (duplicate-submit skip)
    {
        uint32_t idx; CHECK(xrAcquireSwapchainImage(swap, NULL, &idx));
        XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, NULL, XR_INFINITE_DURATION}; CHECK(xrWaitSwapchainImage(swap, &wi));
        CHECK(xrReleaseSwapchainImage(swap, NULL));
        XrCompositionLayerProjectionView pv[2];
        for (int e = 0; e < 2; e++) {
            XrCompositionLayerProjectionView v = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW, NULL, views[e].pose, views[e].fov, {swap, {{(int32_t)(e * w), 0}, {(int32_t)w, (int32_t)h}}, 0}};
            pv[e] = v;
        }
        XrCompositionLayerProjection proj = {XR_TYPE_COMPOSITION_LAYER_PROJECTION, NULL, 0, stage, 2, pv};
        const XrCompositionLayerBaseHeader *layers[] = {(XrCompositionLayerBaseHeader *)&proj};
        XrFrameEndInfo fe = {XR_TYPE_FRAME_END_INFO, NULL, lastT, XR_ENVIRONMENT_BLEND_MODE_OPAQUE, 1, layers};
        uint32_t seqBefore = shm->frame_seq;
        CHECK(xrEndFrame(ses, &fe));
        if (shm->frame_seq != seqBefore) { printf("FAIL dedup: seq %u -> %u\n", seqBefore, shm->frame_seq); return 1; }
        printf("dedup ok (seq stays %u)\n", seqBefore);
    }

    // Quad-only submit lands letterboxed in both eyes (quad fallback)
    {
        XrSwapchainCreateInfo scQ = {XR_TYPE_SWAPCHAIN_CREATE_INFO, NULL, 0, XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, 1, w, h, 1, 1, 1};
        XrSwapchain qswap; CHECK(xrCreateSwapchain(ses, &scQ, &qswap));
        XrSwapchainImageD3D11KHR qimgs[3] = {{XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR}};
        CHECK(xrEnumerateSwapchainImages(qswap, 3, &n, (XrSwapchainImageBaseHeader *)qimgs));
        uint32_t seqQ = shm->frame_seq;
        for (int attempt = 0; attempt < 10 && shm->frame_seq == seqQ; attempt++) {
            XrFrameState fs2 = {XR_TYPE_FRAME_STATE};
            CHECK(xrWaitFrame(ses, NULL, &fs2)); CHECK(xrBeginFrame(ses, NULL));
            uint32_t idx; CHECK(xrAcquireSwapchainImage(qswap, NULL, &idx));
            XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, NULL, XR_INFINITE_DURATION}; CHECK(xrWaitSwapchainImage(qswap, &wi));
            D3D11_RENDER_TARGET_VIEW_DESC rd = {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, D3D11_RTV_DIMENSION_TEXTURE2D};
            ID3D11RenderTargetView *rtv;
            if (FAILED(ID3D11Device_CreateRenderTargetView(dev, (ID3D11Resource *)qimgs[idx].texture, &rd, &rtv))) { printf("FAIL RTV\n"); return 1; }
            float green[4] = {0, 1, 0, 1};
            ID3D11DeviceContext_ClearRenderTargetView(ctx, rtv, green);
            ID3D11RenderTargetView_Release(rtv);
            CHECK(xrReleaseSwapchainImage(qswap, NULL));
            XrCompositionLayerQuad quad = {XR_TYPE_COMPOSITION_LAYER_QUAD, NULL, 0, stage, XR_EYE_VISIBILITY_BOTH,
                {qswap, {{0, 0}, {w, h}}, 0}, {{0, 0, 0, 1}, {0, 0, -1}}, {1.0f, 1.0f}};
            const XrCompositionLayerBaseHeader *layers[] = {(XrCompositionLayerBaseHeader *)&quad};
            XrFrameEndInfo fe = {XR_TYPE_FRAME_END_INFO, NULL, fs2.predictedDisplayTime, XR_ENVIRONMENT_BLEND_MODE_OPAQUE, 1, layers};
            CHECK(xrEndFrame(ses, &fe));
        }
        if (shm->frame_seq != seqQ + 1) { printf("FAIL quad: seq %u -> %u\n", seqQ, shm->frame_seq); return 1; }
        b = shm->frame_seq % 2;
        px = (const uint32_t *)((const uint8_t *)shm + VR4_FRAME_OFFSET + b * VR4_FRAME_MAX);
        if (px[0] != 0xff00ff00u || px[w + 5] != 0xff00ff00u) { printf("FAIL quad pixels %08x %08x\n", px[0], px[w + 5]); return 1; }
        printf("quad ok (green both eyes)\n");
        CHECK(xrDestroySwapchain(qswap));
    }
    printf("PASS\n");
    return 0;
}
