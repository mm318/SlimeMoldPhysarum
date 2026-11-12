#include "ofMain.h"
#include "ofApp.h"

#ifdef TARGET_LINUX
#include <GLFW/glfw3.h>
#endif

//========================================================================
int main() {
#if defined(TARGET_LINUX) && defined(GLFW_PLATFORM_X11)
    // OF 0.12.1 uses native X11 handles; GLFW 3.4 may select Wayland otherwise.
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
    ofGLFWWindowSettings settings;
    settings.setGLVersion(4, 4); // Match the GLSL 4.40 shaders in bin/data.
    settings.setSize(1280, 800);
    ofCreateWindow(settings);
    // this kicks off the running of my app
    ofRunApp(new ofApp());
}
