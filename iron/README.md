# ROS2 iron setup tips for M2 Mac, running macOS Ventura with a Python virtual environment

I spent so many hours trying to get ROS2 iron to compile on my M2 MacBook
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
of Python libraries in a virtual environment) underneath your ROS2 iron
build directory.

So put it somewhere else:

```sh
$ mkdir -p ~/src/ros2/ros2-iron-venv
$ cd ~/src/ros2/ros2-iron-venv
$ python3 -m venv venv
$ activate
$ pip install ...      # install all the ROS2 Python requirements
```

From now on, you need to make sure this virtual environment is active
whenever you are building or rebuilding ROS components. So if you get a
`ModuleNotFound` error at the start of a colcon build, forgetting to
activate the virtual environment is probably the reason for that.

## 2. Get the source normally

[Follow the regular instructions](https://docs.ros.org/en/iron/Installation/Alternatives/macOS-Development-Setup.html)
to get the source code using the `vcs` tool.

You do not need to use a limited set of components. By following all the
fixes described here, I was able to get all the components to build on
my Mac.

You should follow the instructions to install all the regular Homebrew
and pip dependencies.

### 2.1 Fix dependencies

I needed to make some changes to dependencies installed via Homebrew and
pip, compared to the list in the normal instructions:

* I had to uninstall `qt` from Homebrew, so only `qt@5` was available
  during compilation. Unfortunately, a standard uninstall failed:

  ```
  % brew uninstall qt
  Error: Refusing to uninstall /opt/homebrew/Cellar/qt/6.5.1_3
  because it is required by opencv, pyqt and vtk, which are currently installed.
  ```

  Doing a forced removal didn't seem like a good idea, so instead I used
  `brew unlink`:

  ```
  % brew unlink qt
  Unlinking /opt/homebrew/Cellar/qt/6.5.1_3... 3792 symlinks removed.
  ```

* I had to upgrade mypy, one of the ROS2 pip dependencies, from an old version
  of 0.931 set in the ROS2 Iron instructions, to the current version that supports
  Python 3.11:

  ```
  (iron_venv) mryall@raptor:~/src/mawson/ros2-iron% pip install -U mypy
  ...
        Successfully uninstalled mypy-0.931
  Successfully installed mypy-1.5.1
  ```

Error messages and more detailed info follows, but you can skip to section
3 if you don't need that info.

#### 2.1.1 Incorrect Qt version errors

I got errors from `rviz_rendering` that looked like this:

```
-- Build files have been written to: /Users/mryall/src/mawson/ros2-iron-build/build/rviz_rendering
[  3%] Generating include/rviz_rendering/moc_render_window.cpp
[  6%] Building CXX object CMakeFiles/rviz_rendering.dir/include/rviz_rendering/moc_render_window.cpp.o
In file included from /Users/mryall/src/mawson/ros2-iron-build/build/rviz_rendering/include/rviz_rendering/moc_render_window.cpp:10:
In file included from /Users/mryall/src/mawson/ros2-iron-build/build/rviz_rendering/include/rviz_rendering/../../../../src/ros2/rviz/rviz_rendering/include/rviz_rendering/render_window.hpp:36:
In file included from /opt/homebrew/opt/qt@5/lib/QtCore.framework/Headers/QObject:1:
In file included from /opt/homebrew/opt/qt@5/lib/QtCore.framework/Headers/qobject.h:46:
In file included from /opt/homebrew/include/QtCore/qobjectdefs.h:12:
In file included from /opt/homebrew/include/QtCore/qnamespace.h:12:
In file included from /opt/homebrew/include/QtCore/qglobal.h:34:
/opt/homebrew/include/QtCore/qcompilerdetection.h:1218:6: error: "Qt requires a C++17 compiler"
#    error "Qt requires a C++17 compiler"
     ^
In file included from /Users/mryall/src/mawson/ros2-iron-build/build/rviz_rendering/include/rviz_rendering/moc_render_window.cpp:10:
In file included from /Users/mryall/src/mawson/ros2-iron-build/build/rviz_rendering/include/rviz_rendering/../../../../src/ros2/rviz/rviz_rendering/include/rviz_rendering/render_window.hpp:36:
In file included from /opt/homebrew/opt/qt@5/lib/QtCore.framework/Headers/QObject:1:
In file included from /opt/homebrew/opt/qt@5/lib/QtCore.framework/Headers/qobject.h:46:
In file included from /opt/homebrew/include/QtCore/qobjectdefs.h:12:
In file included from /opt/homebrew/include/QtCore/qnamespace.h:12:
In file included from /opt/homebrew/include/QtCore/qglobal.h:46:
/opt/homebrew/include/QtCore/qtypeinfo.h:25:46: error: no template named 'is_trivially_copyable_v' in namespace 'std'; did you mean 'is_trivially_copyable'?
inline constexpr bool qIsRelocatable =  std::is_trivially_copyable_v<T> && std::is_trivially_destructible_v<T>;
                                        ~~~~~^
/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include/c++/v1/__type_traits/is_trivially_copyable.h:21:50: note: 'is_trivially_copyable' declared here
template <class _Tp> struct _LIBCPP_TEMPLATE_VIS is_trivially_copyable
```

Some Googling turned up [this Stack Overflow answer](https://stackoverflow.com/a/72928807/143336) which
correctly identified having `qt` and `qt@5` installed as the problem.

#### 2.1.2 Outdated version of mypy (mypyc)

The failure here was in the `test_capi` module:

```
Starting >>> test_capi
--- stderr: test_capi
/Users/mryall/src/mawson/ros2-iron/iron_venv/lib/python3.11/site-packages/mypyc/lib-rt/setup.py:6: DeprecationWarning: The distutils package is deprecated and slated for removal in Python 3.12. Use setuptools or check PEP 632 for potential alternatives
  from distutils.core import setup, Extension
exc_ops.c:78:56: error: no member named 'exc_type' in 'struct _err_stackitem'
    return PyErr_GivenExceptionMatches(CPy_ExcState()->exc_type, type);
                                       ~~~~~~~~~~~~~~  ^
exc_ops.c:217:14: error: incomplete definition of type 'struct _frame'
    frame_obj->f_lineno = line;
    ~~~~~~~~~^
/opt/homebrew/Cellar/python@3.11/3.11.5/Frameworks/Python.framework/Versions/3.11/include/python3.11/pytypedefs.h:22:16: note: forward declaration of 'struct _frame'
typedef struct _frame PyFrameObject;
               ^
2 errors generated.
error: command '/usr/bin/clang' failed with exit code 1
```

Looking into this error message led me to [this issue with Cython](https://github.com/cython/cython/issues/4500)
which indicated there was a Python 3.11 change to the exception interfaces in
the Python C APIs which broke compatibility with some libraries.

Looking up the location of the failed file, `exc_ops.c`, showed that it was in the
`mypy` library in the Python virtual environment. And upgrading it with pip as shown
above made this problem go away.

But this failure may have actually been a red herring, because I forgot to do step
#1 above, and this was `colcon` trying to incorrectly build C code inside the Python
dependencies. But upgrading the library didn't seem to break anything, so I figure
it is better to use the more recent one anyway if ROS2 doesn't specifically require
mypy 0.931.

## 3. Copy build script to set the environment properly

The script in this directory, [`build.sh`](build.sh) sets the environment
properly for a Python virtual environment and Homebrew setup to build
ROS2 iron on Mac. You need to copy it to your iron build directory
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

(Note these error messages are from my ROS2 humble installation, so the exact
messages may differ with ROS2 iron.)

* `-DPython3_EXECUTABLE` is to ensure CMake doesn't go off trying to
  autodiscover a different Python version, instead of using the one in the
  virtual environment.

* `CMAKE_PREFIX_PATH` ia not quite set correctly in Kliment's example. It
  needs to be separated by semicolons rather than colons.

So that's the fixes on the build side to make sure Python works. Then there
are the source code patches to fix the build with Mac M2 (arm64) and
potentially Monterey-specific issues that I hit.

## 4. Patch the source code

This repository has one patch file which you can apply as follows, running
in your ros2 iron directory:

```sh
$ patch -u -d src/ros2/geometry2 < ros2_geometry2.patch
```

Compared to Humble, Iron needs fewer patches for compatibility with Mac M2:

* Ogre has had the Mac ARM compatibility fixes applied in Iron
* The errors in `performance_test_fixture` no longer occur; perhaps Clang
  compatibility there has been improved.

The rest of this section explains each patches fix and the error messages
you see if you don't apply them. Jump straight to step 5 if you don't care
about that.

### 4.1 Patch for missing include on `kdl/frames.hpp`

I had three modules fail because they couldn't compile against `kdl/frames.hpp`,
despite it being present in the build directory in this location:

`build/python_orocos_kdl_vendor/_deps/python_orocos_kdl-src/orocos_kdl/src/frames.hpp` 

```text
[ 50%] Building CXX object CMakeFiles/tf2_eigen_kdl.dir/src/tf2_eigen_kdl.cpp.o
In file included from /Users/mryall/src/mawson/ros2-iron-build/src/ros2/geometry2/tf2_eigen_kdl/src/tf2_eigen_kdl.cpp:33:
/Users/mryall/src/mawson/ros2-iron-build/src/ros2/geometry2/tf2_eigen_kdl/include/tf2_eigen_kdl/tf2_eigen_kdl.hpp:48:32: warning: unknown warning group '-Wclass-memaccess', ignored [-Wunknown-warning-option]
#pragma GCC diagnostic ignored "-Wclass-memaccess"
                               ^
/Users/mryall/src/mawson/ros2-iron-build/src/ros2/geometry2/tf2_eigen_kdl/include/tf2_eigen_kdl/tf2_eigen_kdl.hpp:56:10: fatal error: 'kdl/frames.hpp' file not found
#include "kdl/frames.hpp"
         ^~~~~~~~~~~~~~~~
1 warning and 1 error generated.
make[2]: *** [CMakeFiles/tf2_eigen_kdl.dir/src/tf2_eigen_kdl.cpp.o] Error 1
make[1]: *** [CMakeFiles/tf2_eigen_kdl.dir/all] Error 2
make: *** [all] Error 2
```

The patch updates the `include` path for the geometry2 module to include
the `orocos_kdl` library. This seems to fix all the modules that had
this compilation failure - I guess they are all hitting it transitively.

## 5. Build ROS2 iron

You can copy `build.sh` into your iron build directory, and use it
instead of manually running `colcon build`. It will make sure all the
environment settings are done correctly.

Hopefully at the end, you see this:

```sh
$ ./build.sh

... lots and lots of log messages ...
Finished <<< rviz2 [5.73s]

Summary: 333 packages finished [7min 56s]
  140 packages had stderr output: action_msgs action_tutorials_interfaces action_tutorials_py actionlib_msgs ament_clang_format ament_clang_tidy ament_copyright ament_cppcheck ament_cpplint ament_flake8 ament_index_python ament_lint ament_lint_cmake ament_mypy ament_package ament_pclint ament_pep257 ament_pycodestyle ament_pyflakes ament_uncrustify ament_xmllint builtin_interfaces class_loader composition_interfaces cyclonedds demo_nodes_py diagnostic_msgs domain_coordinator example_interfaces examples_rclpy_executors examples_rclpy_guard_conditions examples_rclpy_minimal_action_client examples_rclpy_minimal_action_server examples_rclpy_minimal_client examples_rclpy_minimal_publisher examples_rclpy_minimal_service examples_rclpy_minimal_subscriber examples_rclpy_pointcloud_publisher examples_tf2_py fastcdr geometry_msgs iceoryx_binding_c iceoryx_hoofs iceoryx_posh launch launch_pytest launch_ros launch_testing launch_testing_examples launch_testing_ros launch_xml launch_yaml lifecycle_msgs lifecycle_py logging_demo map_msgs nav_msgs osrf_pycommon osrf_testing_tools_cpp pendulum_msgs performance_test_fixture python_orocos_kdl_vendor quality_of_service_demo_py rcl_interfaces rcl_logging_interface rcl_logging_noop rcl_logging_spdlog rcl_yaml_param_parser rcpputils resource_retriever rmw rmw_connextdds_common rmw_cyclonedds_cpp rmw_dds_common rmw_implementation ros2action ros2bag ros2cli ros2cli_test_interfaces ros2component ros2doctor ros2interface ros2launch ros2lifecycle ros2multicast ros2node ros2param ros2pkg ros2run ros2service ros2test ros2topic ros2trace rosbag2_examples_py rosbag2_interfaces rosbag2_storage rosbag2_storage_mcap rosbag2_test_msgdefs rosgraph_msgs rosidl_cli rosidl_dynamic_typesupport rosidl_pycommon rosidl_runtime_c rosidl_runtime_py rosidl_typesupport_c rosidl_typesupport_cpp rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp rosidl_typesupport_introspection_cpp rpyutils rti_connext_dds_cmake_module rviz_rendering sensor_msgs sensor_msgs_py service_msgs shape_msgs sros2 statistics_msgs std_msgs std_srvs stereo_msgs test_launch_ros test_msgs test_ros2trace test_tracetools_launch tf2_msgs tf2_ros_py tf2_tools topic_monitor tracetools_launch tracetools_read tracetools_test tracetools_trace trajectory_msgs turtlesim type_description_interfaces unique_identifier_msgs urdfdom urdfdom_headers visualization_msgs
```

If you get more errors, you can pass one or more package names to `build.sh`
to just build those packages. For example:

```sh
$ ./build.sh interactive_markers
```

You can also just `cd` into the relevant directory (e.g. `build/interactive_markers`
in this case), and run `make`.

