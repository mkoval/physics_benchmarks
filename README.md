# Physics Simulation Benchmarks for Planar Pushing

## Installation

Follow the [Personal Robotics Lab development guide][develguide] to build these
packages using this [`.rosinstall` file](rosinstall). You will also need to
copy the [`or_fcl.package.xml` file](or_fcl.package.xml) into your checkout of
`or_fcl` and rename it to `package.xml`. We *strongly recommend* that you build
these packages with compiler optimizations enabled, i.e. pass
`-DCMAKE_BUILD_TYPE=Release` to cmake, to get meaningful benchmark results.

The full build process should look something like this:
```console
$ mkdir -p /path/to/my/workspace/src
$ cd /path/to/my/workspace
# Create a Catkin workspace:
$ catkin init
$ catkin config --extend /opt/ros/indigo
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# Checkout source code:
$ wget -O src/.rosinstall https://raw.githubusercontent.com/mkoval/physics_benchmarks/master/rosinstall
$ wstool update -t src
$ cp src/or_fcl.package.xml src/or_fcl/package.xml
# Install system dependencies:
$ rosdep update
$ rosdep -y --rosdistro=indigo --ignore-src --from-path=src
# Build:
$ catkin build
```

## Usage: DART Benchmark

Make sure you source `devel/setup.bash`. Then, run:
```console
$ rosrun dart_benchmark dart_benchmark
```
You can pass the `--help` flag to see more options.


## Usage: MuJoCo Benchmark

Make sure you source `devel/setup.bash`. The `mujoco_benchmark` script *must*
be run from the `mujoco_benchmark` directory. You also must place your MuJoCo
key in that directory with name `mjkey.txt`. Then, run:
```console
$ rosrun mujoco_benchmark mujoco_benchmark
```
You can pass the `--help` flag to see more options.


[develguide]: https://www.personalrobotics.ri.cmu.edu/software/development-environment
