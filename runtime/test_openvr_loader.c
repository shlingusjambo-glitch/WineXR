/* Probe the real game's OpenVR entry DLL, unlike test_xr's direct runtime load.
 * Run from the plugin directory or pass its absolute Windows DLL path.
 * Success proves loader initialization/interfaces, not rendered gameplay. */
#include <windows.h>
#include <stdint.h>
#include <shlobj.h>
#include <stdio.h>
typedef uint32_t (__cdecl *Init2)(int *, int, const char *);
typedef uint32_t (__cdecl *Init1)(int *, int);
typedef void (__cdecl *Shutdown)(void);
typedef void *(__cdecl *Interface)(const char *, int *);
typedef const char *(__cdecl *ErrorText)(int);
typedef void (__cdecl *TargetSize)(uint32_t *, uint32_t *);
int main(int argc, char **argv) {
    if(!freopen("C:\\VR4Mac\\loader-probe.txt", "w", stdout))
        fprintf(stderr,"warning: log redirect failed, using console\n");
    setvbuf(stdout,NULL,_IONBF,0);
    printf("Probe started argc=%d\n",argc);
    const char *path=(argc>1 && argv[1][0]) ? argv[1] : "C:\\Program Files (x86)\\Steam\\steamapps\\common\\Gorilla Tag\\Gorilla Tag_Data\\Plugins\\x86_64\\openvr_api.dll";
    printf("DLL=%s\n",path);
    HMODULE dll=LoadLibraryA(path);
    if(!dll && argc == 1) {
        path="C:\\Program Files (x86)\\Steam\\steamapps\\downloading\\1533390\\Gorilla Tag_Data\\Plugins\\x86_64\\openvr_api.dll";
        printf("Trying downloaded game DLL=%s\n",path); dll=LoadLibraryA(path);
    }
    if(!dll) { printf("FAIL LoadLibrary %s winerr=%lu\n",path,GetLastError()); return 1; }
    Init2 init2=(Init2)GetProcAddress(dll,"VR_InitInternal2");
    Init1 init1=(Init1)GetProcAddress(dll,"VR_InitInternal");
    Shutdown stop=(Shutdown)GetProcAddress(dll,"VR_ShutdownInternal");
    Interface get=(Interface)GetProcAddress(dll,"VR_GetGenericInterface");
    ErrorText text=(ErrorText)GetProcAddress(dll,"VR_GetVRInitErrorAsEnglishDescription");
    if((!init2 && !init1) || !stop || !get) { puts("FAIL missing OpenVR entry exports; supply game openvr_api.dll, not vrclient_x64.dll"); FreeLibrary(dll); return 2; }
    char local[MAX_PATH]={0}; SHGetFolderPathA(NULL,CSIDL_LOCAL_APPDATA,NULL,SHGFP_TYPE_CURRENT,local);
    printf("Actual LOCAL_APPDATA=%s\n",local);
    char override[MAX_PATH]={0}; GetEnvironmentVariableA("VR_OVERRIDE",override,MAX_PATH); printf("VR_OVERRIDE=%s\n",override);
    const char *deps[]={"MSVCP140.dll","VCRUNTIME140.dll","VCRUNTIME140_1.dll","vulkan-1.dll","D3DCOMPILER_47.dll"};
    for(unsigned i=0;i<sizeof(deps)/sizeof(deps[0]);i++) {
        HMODULE dep=LoadLibraryA(deps[i]);
        printf("Dependency %s load=%s error=%lu\n",deps[i],dep?"OK":"FAIL",dep?0:GetLastError());
        if(dep)FreeLibrary(dep);
    }
    HMODULE client=LoadLibraryA("C:\\VR4Mac\\OpenComposite\\bin\\win64\\vrclient_x64.dll");
    printf("Direct vrclient load=%s error=%lu\n",client?"OK":"FAIL",client?0:GetLastError());
    if(client) {
        typedef void *(__cdecl *Factory)(const char *,int *);
        Factory factory=(Factory)GetProcAddress(client,"VRClientCoreFactory"); int factoryError=0;
        printf("IVRClientCore_003 factory=%p error=%d\n",factory?factory("IVRClientCore_003",&factoryError):NULL,factoryError);
        FreeLibrary(client);
    }
    typedef unsigned char (__cdecl *RuntimePath)(char *,uint32_t,uint32_t *);
    RuntimePath runtimePath=(RuntimePath)GetProcAddress(dll,"VR_GetRuntimePath"); char root[MAX_PATH]={0};uint32_t required=0;
    if(runtimePath) { int ok=runtimePath(root,MAX_PATH,&required);printf("OpenVR runtime path ok=%d required=%u root=%s\n",ok,required,root); }
    int error=0;
    uint32_t token=init2 ? init2(&error,1,"") : init1(&error,1); /* VRApplication_Scene */
    printf("OpenVR init token=%u error=%d %s\n",token,error,text?text(error):"");
    if(error == 102) {
        puts("Diagnostic retry with process-local VR_OVERRIDE=C:\\VR4Mac\\OpenComposite (does not fix game registration)");
        SetEnvironmentVariableA("VR_OVERRIDE","C:\\VR4Mac\\OpenComposite");
        error=0; token=init2 ? init2(&error,1,"") : init1(&error,1);
        printf("Override init token=%u error=%d %s\n",token,error,text?text(error):"");
    }
    if(error || !token) { FreeLibrary(dll); return 3; }
    const char *versions[]={"FnTable:IVRSystem_022","FnTable:IVRSystem_021","FnTable:IVRSystem_020","FnTable:IVRSystem_019"};
    void *system=NULL; const char *chosen=NULL;
    for(unsigned i=0;i<sizeof(versions)/sizeof(versions[0]);i++) { error=0; system=get(versions[i],&error); if(system && !error) { chosen=versions[i];break; } }
    int result=0;
    if(!system) { printf("FAIL IVRSystem interface error=%d\n",error); result=4; }
    else {
        uint32_t w=0,h=0;
        TargetSize size=((TargetSize *)system)[0];
        if(!size) { puts("FAIL null GetRecommendedRenderTargetSize"); result=5; }
        else { size(&w,&h); printf("%s recommended eye=%ux%u\n",chosen,w,h); if(!w||!h)result=6; }
    }
    stop(); FreeLibrary(dll);
    if(!result) puts("PASS real OpenVR loader + system interface. Confirm custom runtime log and perform rendering/input/audio tests separately.");
    fflush(stdout);
    return result;
}
