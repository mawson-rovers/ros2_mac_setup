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
to get the source code using the `vcs` tool.

You do not need to use a limited set of components. By following all the
fixes described here, I was able to get all the components to build on
my Mac.

## 3. Copy build script to set the environment properly

The script in this repository, [`build.sh`](build.sh) sets the environment
properly for a Python virtual environment and Homebrew setup to build
ROS2 humble on Mac. You need to copy it to your humble build directory
(where you ran the `vcs` command) so you can use it instead of 
`colcon build` later on.

### 3.1 What the build script does

Here it is for reference:

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

This repository has patch files which you can apply as follows, running
in your ros2 humble directory:

```sh
$ patch -u -d src/ros2/rviz < ros2_rviz.patch
$ patch -u -d src/ros2/performance_test_fixture < ros2_performance_test_fixture.patch
$ patch -u -d src/ros2/geometry2 < ros2_geometry2.patch
$ patch -u -d src/ros2/rosbag2 < ros2_rosbag2.patch
$ patch -u -d src/ros-visualisation/interactive_markers < ros-visualization_interactive_markers.patch
```

The rest of this section explain what the patches fix and the error messages
you see if you don't apply them. Jump straight to step 5 if you don't care
about that.

### 4.1 Patch RViz Ogre compatibility for Mac arm64

`ros2_rviz.patch` is the biggest patch and updates the vendored Ogre version
(`rviz_ogre_vendor`) with Mac arm64 compatibility. This was taken from
[this commit](https://github.com/OGRECave/ogre-next/commit/ff01338) on the
v2.x ogre-next repo, which had already been updated for Mac M1.

If you don't apply this patch, the errors you get from this start with this:

```text
... can't find this error any more, sorry :(
```

If you only apply a partial patch, like the one suggested in
[this ROS pull request](https://github.com/ros2/rviz/pull/700/files),
then it still fails because of some incompatible assembly code and x86
headers being used:

```text
[ 16%] Building CXX object OgreMain/CMakeFiles/OgreMain.dir/src/OgrePlatformInformation.cpp.o
/Users/mryall/opt/ros2/ros2-humble/build/rviz_ogre_vendor/ogre-v1.12.1-prefix/src/ogre-v1.12.1/OgreMain/src/OgrePlatformInformation.cpp:164:22: error: invalid output constraint '=a' in asm
            "cpuid": "=a" (result._eax), "=b" (result._ebx), "=c" (result._ecx), "=d" (result._edx) : "a" (query)
                     ^
1 error generated.
In file included from /Users/mryall/opt/ros2/ros2-humble/build/rviz_ogre_vendor/ogre-v1.12.1-prefix/src/ogre-v1.12.1/OgreMain/src/OgreOptimisedUtilSSE.cpp:36:
In file included from /Users/mryall/opt/ros2/ros2-humble/build/rviz_ogre_vendor/ogre-v1.12.1-prefix/src/ogre-v1.12.1/OgreMain/src/OgreSIMDHelper.h:69:
/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/lib/clang/14.0.0/include/xmmintrin.h:14:2: error: "This header is only meant to be used on x86 and x64 architecture"
#error "This header is only meant to be used on x86 and x64 architecture"
 ^
```

### 4.2 Patch `performance_test_fixture` to remove template method signatures that don't match

`ros2_performance_test_fixture.patch` just comments out some code that didn't
compile for me on my Mac. The error looks like this:

```text
Undefined symbols for architecture arm64:
  "osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(std::__1::variant<std::__1::function<void (osrf_testing_tools_cpp::memory_tools::MemoryToolsService&)>, std::__1::function<void ()>, std::nullptr_t>)", referenced from:
      performance_test_fixture::PerformanceTest::SetUp(benchmark::State&) in performance_test_fixture.cpp.o
  "osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(std::__1::variant<std::__1::function<void (osrf_testing_tools_cpp::memory_tools::MemoryToolsService&)>, std::__1::function<void ()>, std::nullptr_t>)", referenced from:
      performance_test_fixture::PerformanceTest::SetUp(benchmark::State&) in performance_test_fixture.cpp.o
  "osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(std::__1::variant<std::__1::function<void (osrf_testing_tools_cpp::memory_tools::MemoryToolsService&)>, std::__1::function<void ()>, std::nullptr_t>)", referenced from:
      performance_test_fixture::PerformanceTest::SetUp(benchmark::State&) in performance_test_fixture.cpp.o
ld: symbol(s) not found for architecture arm64
clang: error: linker command failed with exit code 1 (use -v to see invocation)
make[2]: *** [libperformance_test_fixture.dylib] Error 1
```

I checked the exported symbols in `libmemory_tools.dylib`, and the relevant
functions (e.g. `on_unexpected_calloc`) were definitely present with a
similar call signature, and the file was a valid arm64 dylib.

So I think there might be a slight incompatibility here with the Mac
compiler and the compatibility check on these complex type signatures
against the linker.

In my case, I'm not interested in running any ROS performance tests, so I
just commented out the offending code.

### 4.3 Patch for missing include on `kdl/frames.hpp`

Lastly, I had three modules fail because they couldn't compile against
`kdl/frames.hpp`, despite it being present in `/opt/homebrew/include`,
and `/opt/homebrew/` being in the `CMAKE_PREFIX_PATH` list.

```text
[ 50%] Building CXX object CMakeFiles/tf2_eigen_kdl.dir/src/tf2_eigen_kdl.cpp.o
In file included from /Users/mryall/opt/ros2/ros2-humble/src/ros2/geometry2/tf2_eigen_kdl/src/tf2_eigen_kdl.cpp:33:
/Users/mryall/opt/ros2/ros2-humble/src/ros2/geometry2/tf2_eigen_kdl/include/tf2_eigen_kdl/tf2_eigen_kdl.hpp:48:32: warning: unknown warning group '-Wclass-memaccess', ignored [-Wunknown-warning-option]
#pragma GCC diagnostic ignored "-Wclass-memaccess"
                               ^
/Users/mryall/opt/ros2/ros2-humble/src/ros2/geometry2/tf2_eigen_kdl/include/tf2_eigen_kdl/tf2_eigen_kdl.hpp:56:10: fatal error: 'kdl/frames.hpp' file not found
#include "kdl/frames.hpp"
         ^~~~~~~~~~~~~~~~
1 warning and 1 error generated.
make[2]: *** [CMakeFiles/tf2_eigen_kdl.dir/src/tf2_eigen_kdl.cpp.o] Error 1
make[1]: *** [CMakeFiles/tf2_eigen_kdl.dir/all] Error 2
make: *** [all] Error 2
```

I was already at my wit's end for why this might be happening, so I
just hard-coded the `/opt/homebrew/include` in the include path for
each affected module.

You can see this in the three remaining patch files, `ros2_geometry2.patch`,
`ros2_rosbag2.patch`, and `ros-visualization_interactive_markers.patch`.

## 5. Build ROS2 humble

You can copy `build.sh` into your humble build directory, and use it
instead of manually running `colcon build`. It will make sure all the
environment settings are done correctly.

Hopefully at the end, you see this:

```sh
$ ./build.sh

... lots and lots of log messages ...

Summary: 318 packages finished [8min 18s]
  81 packages had stderr output: action_tutorials_cpp camera_calibration_parsers camera_info_manager class_loader composition demo_nodes_cpp demo_nodes_cpp_native dummy_map_server dummy_sensors examples_rclcpp_async_client examples_rclcpp_cbg_executor examples_rclcpp_minimal_action_client examples_rclcpp_minimal_action_server examples_rclcpp_minimal_client examples_rclcpp_minimal_composition examples_rclcpp_minimal_publisher examples_rclcpp_minimal_service examples_rclcpp_minimal_subscriber examples_rclcpp_minimal_timer examples_rclcpp_multithreaded_executor examples_rclcpp_wait_set fastcdr iceoryx_hoofs image_tools image_transport interactive_markers intra_process_demo laser_geometry libstatistics_collector lifecycle logging_demo message_filters osrf_testing_tools_cpp performance_test_fixture quality_of_service_demo_cpp rcl rcl_action rcl_lifecycle rcl_logging_interface rcl_logging_noop rcl_logging_spdlog rcl_yaml_param_parser rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle rclpy rcpputils resource_retriever rmw rmw_connextdds_common rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp rmw_fastrtps_shared_cpp rmw_implementation robot_state_publisher ros2lifecycle_test_fixtures rosbag2_compression rosbag2_compression_zstd rosbag2_cpp rosbag2_py rosbag2_storage rosbag2_storage_mcap rosbag2_transport rosidl_runtime_c rosidl_typesupport_c rosidl_typesupport_cpp rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp rosidl_typesupport_introspection_cpp rti_connext_dds_cmake_module rviz2 rviz_common rviz_default_plugins rviz_rendering tf2_py tf2_ros topic_statistics_demo turtlesim urdf urdfdom
```

If you get more errors, you can pass one or more package names to `build.sh`
to just build those packages. For example:

```sh
$ ./build.sh interactive_markers
```

You can also just `cd` into the relevant directory (e.g. `build/interactive_markers`
in this case), and run `make`.

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

