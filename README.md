# Slime Mold Physarum

An openFrameworks slime mold simulation using OpenGL compute shaders.

## Build on Linux

The CMake build uses [mm318/ofxCMake](https://github.com/mm318/ofxCMake), pinned
to revision `7a92d86054eb41a75584b2ddedcc0c72a3c2f325`, and openFrameworks
0.12.1. It requires CMake 3.16+, Git, pkg-config, and a C++17-capable compiler.
ofxCMake selects C++20 by default with GCC 10 or newer.

Use the existing extracted release at
`/home/user/of_v0.12.1_linux64_gcc6_release`; no new openFrameworks download
or separate core build is needed. The default path is that release folder next
to this repository. Configure and build from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOF_DIRECTORY_BY_USER=/home/user/of_v0.12.1_linux64_gcc6_release
cmake --build build --parallel 4
./build/bin/Slime_Mold
```

The first configure fetches the pinned ofxCMake revision. To use the existing
local checkout instead (including when offline), add the following to the
configure command:

```bash
-DFETCHCONTENT_SOURCE_DIR_OFXCMAKE=/home/user/ofxCMake
```

For another installation, set `OF_DIRECTORY_BY_USER` to its extracted official
0.12.1 Linux release directory. If system development libraries are missing,
install the openFrameworks dependencies using the distribution's script, for
example on Ubuntu:

```bash
sudo /home/user/of_v0.12.1_linux64_gcc6_release/scripts/linux/ubuntu/install_dependencies.sh
```

The build compiles the openFrameworks core, the release's `ofxGui` addon, and
the application together. Generated libraries stay in `build/lib`; the
executable and a copy of `bin/data` (shaders and settings) go in `build/bin`.
The openFrameworks installation is left unchanged. A graphical session and an
OpenGL 4.4-capable driver are required by the bundled GLSL 4.40 shaders. This
ofxCMake revision supports desktop Linux; the existing Visual Studio files are
legacy projects. The app uses X11 (XWayland in a Wayland session), matching
openFrameworks 0.12.1's Linux window backend.
