# Physics Simulation Benchmarks for Planar Pushing

Follow the [Personal Robotics Lab development guide][develguide] to build these
packages using the following `.rosinstall` file:
```yaml
- git:
    local-name: aikido
    uri: https://github.com/personalrobotics/aikido.git
- git:
    local-name: dart
    uri: https://github.com/dartsim/dart.git
- git:
    local-name: fcl
    uri: https://github.com/flexible-collision-library/fcl.git
    version: 0.4.0
- git:
    local-name: mujoco
    uri: https://github.com/personalrobotics/MuJoCo.git
- git:
    local-name: physics_benchmarks
    uri: https://github.com/mkoval/physics_benchmarks.git
```

You will also need to copy this `package.xml` file in the `fcl` directory:
```xml
<package format="2">
  <name>fcl</name>
  <version>0.4.0</version>
  <description>FCL is a collision checking library</description>
  <author email="panj@cs.unc.edu">Jia Pan</author>
  <maintainer email="isucan@willowgarage.com">Ioan Sucan</maintainer>
  <license>BSD</license>
  <url>http://gamma.cs.unc.edu/FCL/</url>
  <buildtool_depend>catkin</buildtool_depend>
  <buildtool_depend>pkg-config</buildtool_depend>
  <depend>octomap</depend>
  <depend>libccd</depend>
  <depend>boost</depend>
  <!-- These are required by REP-136. -->
  <exec_depend>catkin</exec_depend>
  <export>
    <build_type>cmake</build_type>
  </export>
</package>
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
