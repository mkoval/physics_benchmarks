# Physics Simulation Benchmarks for Planar Pushing

## Installation

Follow the [Personal Robotics Lab development guide][develguide] to build these
packages using this [`.rosinstall` file](rosinstall). You will also need to
copy the [`fcl.package.xml` file](fcl.package.xml) into your checkout of
`or_fcl` and rename it to `package.xml`. We *strongly recommend* that you build
these packages with compiler optimizations enabled, i.e. pass
`-DCMAKE_BUILD_TYPE=Release` to cmake, to get meaningful benchmark results.

The full build process should look something like this. First, create a Catkin
workspace:
```console
$ mkdir -p /path/to/my/workspace/src
$ cd /path/to/my/workspace
$ catkin init
$ catkin config --extend /opt/ros/indigo
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Next, use `wstool` to checkout the source code and `rosdep` to install system
dependencies:
```console
$ wget -O src/.rosinstall https://raw.githubusercontent.com/mkoval/physics_benchmarks/master/rosinstall
$ wstool update -t src
$ cp src/physics_benchmarks/fcl.package.xml src/fcl/package.xml
$ rosdep update
$ rosdep install -y --rosdistro=indigo --ignore-src --from-path=src
```
Finally, use `catkin-tools` to build the workspace:
```console
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
