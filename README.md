# ROS2 setup tips for M2 Mac, running macOS Monterey/Ventura with a Python virtual environment

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

There are now two versions of the guide depending on which version of ROS2
you are using:

* ROS2 Iron (latest) - see [iron/README.md](iron/README.md) and the patch in the `iron/` directory
* ROS2 Humble - see [humble/README.md](humble/README.md) and the patches in the `humble/` directory

Fortunately, this process seems to be getting easier with newer versions, so
if you have the choice, go with Iron instead of Humble.

