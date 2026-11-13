#include "ofMain.h"
#include "ofApp.h"

#ifdef TARGET_LINUX
#include <GLFW/glfw3.h>
#endif

int main() {
#if defined(TARGET_LINUX) && defined(GLFW_PLATFORM_X11)
    // OF 0.12.1 uses native X11 handles; GLFW 3.4 may select Wayland otherwise.
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
    ofGLFWWindowSettings settings;
    settings.setGLVersion(3, 2);
    settings.setSize(1280, 800);
    ofCreateWindow(settings);
    ofRunApp(new ofApp());
}
