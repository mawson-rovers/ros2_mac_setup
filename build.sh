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
    -DCMAKE_APPLE_SILICON_PROCESSOR="arm64" \
    -DCMAKE_PREFIX_PATH="$(brew --prefix);$(brew --prefix qt@5)" \
    -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"
