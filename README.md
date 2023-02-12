# ROS2 humble setup tips for M2 Mac, running macOS Monterey with a Python virtual environment

I spent so many hours trying to get ROS2 humble to compile on my M2 MacBook
Air, that I thought I should share the learnings so nobody else has to go
through it again.

I only had one unusual requirement, which is that I wanted to use a Python
virtual environment so the ROS Python dependencies didn't clutter up my
system. But this is not that unusual (I install everything this way), and
most of the problems I hit were not actually related to Python.

Many thanks to the online references that got me through this epic journey:

* [Kliment Mamykin's blog on building ROS2 galactic on Big Sur](http://mamykin.com/posts/building-ros2-on-macos-big-sur-m1/)
* [dpbnasika's work on getting a ROS2 galactic build on M2](https://github.com/dpbnasika/ros2)
* [Yadu Nund's issue to fix rviz on Mac](https://github.com/ros2/rviz/issues/943)
    * Including [these commits](https://github.com/ros2/rviz/pull/944/commits/9cd559afb72dc9a7f09082a4fe3e0afcbb3a6803)
      to patch `rviz_ogre_vendor`, which was a good start, but required more
      changes (see below)


## 0. Don't disable System Integrity Protection (SIP)

I'm always sceptical when I see some random software installation guide asking
me to disable an operating-system-wide security feature like SIP. It really
looks like the maintainers don't know what they're doing on the platform.

In this case, I didn't need to disable SIP on my Mac to get it to build, so
I would suggest you don't disable it either.

## 1. Create your Python virtual environment outside the ROS directory

ROS2 uses a tool called `colcon` to build all its code in parallel. Very
helpfully, `colcon` will find libraries anywhere under the directory where
it is run. So you don't want to put any code it shouldn't touch (like a set
of Python libraries in a virtual environment) underneath your ROS2 humble
build directory.

So put it somewhere else:

```sh
$ mkdir -p ~/src/ros2/ros2-humble-venv
$ cd ~/src/ros2/ros2-humble-venv
$ python3 -m venv venv
$ activate
$ pip install ...      # install all the ROS2 Python requirements
```

From now on, you need to make sure this virtual environment is active
whenever you are building or rebuilding ROS components. So if you get a
`ModuleNotFound` error at the start of a colcon build, forgetting to
activate the virtual environment is probably the reason for that.

## 2. Get the source normally

[Follow the regular instructions](https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html)
to get the source code using the `vcs` tool. There is no need to 
requirements here - you should be able to build all the components when you
are done with the fixes here.

## 3. Build with a script that sets the environment properly

The script in this repository, `build.sh` sets the environment properly for
a Python virtual environment and Homebrew setup on Mac. Here it is for
reference:

```sh
#!/bin/bash

if [ -z "$VIRTUAL_ENV" ]; then
    echo "Python virtual env not activated"
    exit 1
fi

if [ -z "$PYTHONPATH" ]; then
    export PYTHONPATH=`python -c 'import sysconfig; print(sysconfig.get_path("purelib"))'`
    echo "Set PYTHONPATH=$PYTHONPATH"
fi

if [ $# -gt 0 ]; then
    PACKAGES_SELECT="--packages-select $@"
fi

colcon build \
  --symlink-install \
  --merge-install \
  --event-handlers console_cohesion+ console_package_list+ desktop_notification- \
  $PACKAGES_SELECT \
  --packages-skip-by-dep python_qt_binding \
  --cmake-clean-cache \
  --cmake-args \
    --no-warn-unused-cli \
    -DBUILD_TESTING=OFF \
    -DINSTALL_EXAMPLES=ON \
    -DCMAKE_OSX_SYSROOT=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk \
    -DCMAKE_OSX_ARCHITECTURES="arm64" \
    -DCMAKE_PREFIX_PATH="$(brew --prefix);$(brew --prefix qt@5)" \
    -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"
```

Most of these came from [Kliment Mamykin's blog](http://mamykin.com/posts/building-ros2-on-macos-big-sur-m1/)
and I can't speak to the importance of all of them, but here are the ones I
added:

* Initial check for `$VIRTUAL_ENV` so you don't forgot to activate it.
* For the virtual environment to be picked up by the colcon build, you need to
  set PYTHONPATH to the `site-packages` directory in your venv. (I use
  `sysconfig.get_path()` to find the directory programmatically.)

If you don't set PYTHONPATH, and the build can't find the Python
dependencies, you'll get errors like this:

```test
-- Found ament_cmake: 1.3.3 (/Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake/cmake)
Traceback (most recent call last):
  File "/Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake_core/cmake/package_templates/templates_2_cmake.py", line 21, in <module>
    from ament_package.templates import get_environment_hook_template_path
ModuleNotFoundError: No module named 'ament_package'
CMake Error at /Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake_core/cmake/ament_cmake_package_templates-extras.cmake:41 (message):
  execute_process(/Users/mryall/src/mawson/ros_ws/venv/bin/python
  /Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake_core/cmake/package_templates/templates_2_cmake.py
  /Users/mryall/opt/ros2/ros2-humble/build/orocos_kdl_vendor/ament_cmake_package_templates/templates.cmake)
  returned error code 1
Call Stack (most recent call first):
  /Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake_core/cmake/ament_cmake_coreConfig.cmake:41 (include)
  /Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake/cmake/ament_cmake_export_dependencies-extras.cmake:15 (find_package)
  /Users/mryall/opt/ros2/ros2-humble/install/share/ament_cmake/cmake/ament_cmakeConfig.cmake:41 (include)
  CMakeLists.txt:5 (find_package)
```

* `-DPython3_EXECUTABLE` is to ensure CMake doesn't go off trying to
  autodiscover a different Python version, instead of using the one in the
  virtual environment. This is important to avoid the incompatibility with
  Python 3.11 mentioned in _Issues I didn't fix_ below.

* `CMAKE_PREFIX_PATH` ia not quite set correctly in Kliment's example. It
  needs to be separated by semicolons rather than colons.

So that's the fixes on the build side to make sure Python works. Then there
are the source code patches to fix the build with Mac M2 (arm64) and
potentially Monterey-specific issues that I hit.

## 4. Patch the source code

This repository has patch files with the necessary changes. You should be
able to apply them like this:

... (patched coming soon)


## Issues I didn't fix

There was one further issues I uncovered with building ROS2 humble with Python
3.11, which was found by CMake's `FindPython3` in my Homebrew environment.
The failure was in compiling the `python_orocos_kdl_vendor` module:

```text
Starting >>> python_orocos_kdl_vendor
--- output: python_orocos_kdl_vendor                             
[ 16%] Building CXX object _deps/python_orocos_kdl-build/CMakeFiles/PyKDL.dir/PyKDL/PyKDL.cpp.o
[ 33%] Building CXX object _deps/python_orocos_kdl-build/CMakeFiles/PyKDL.dir/PyKDL/frames.cpp.o
[ 50%] Building CXX object _deps/python_orocos_kdl-build/CMakeFiles/PyKDL.dir/PyKDL/kinfam.cpp.o
[ 66%] Building CXX object _deps/python_orocos_kdl-build/CMakeFiles/PyKDL.dir/PyKDL/framevel.cpp.o
In file included from /Users/mryall/opt/ros2/ros2-humble/build/python_orocos_kdl_vendor/_deps/python_orocos_kdl-src/python_orocos_kdl/PyKDL/PyKDL.cpp:27:
In file included from /Users/mryall/opt/ros2/ros2-humble/build/python_orocos_kdl_vendor/_deps/python_orocos_kdl-src/python_orocos_kdl/PyKDL/PyKDL.h:26:
In file included from /Users/mryall/opt/ros2/ros2-humble/install/include/pybind11_vendor/pybind11/pybind11.h:13:
In file included from /Users/mryall/opt/ros2/ros2-humble/install/include/pybind11_vendor/pybind11/attr.h:13:
In file included from /Users/mryall/opt/ros2/ros2-humble/install/include/pybind11_vendor/pybind11/cast.h:16:
/Users/mryall/opt/ros2/ros2-humble/install/include/pybind11_vendor/pybind11/detail/type_caster_base.h:482:26: error: member access into incomplete type 'PyFrameObject' (aka '_frame')
            frame = frame->f_back;
                         ^
/opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11/include/python3.11/pytypedefs.h:22:16: note: forward declaration of '_frame'
typedef struct _frame PyFrameObject;
               ^
```

From my quick research, it looks like Python 3.11 removed a deprecated API
called `PyFrameObject`, which is used the ROS Python bindings to inspect
exceptions that are returned from Python and convert them into meaningful
error messages in the C++ code.

My solution was to ensure I was using the correct version of Python (3.10 in
my case), by setting `Python3_EXECUTABLE` environment variable as part of
the `--cmake-args` setting for `colcon build` (see in script above).

If you want to build ROS2 humble with Python 3.11, you will need to work out
how to solve this.

