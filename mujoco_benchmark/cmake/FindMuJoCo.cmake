find_path(MUJOCO_INCLUDE_DIR "MuJoCo/mujoco.h" DOC "MuJoCo header path")
set(MUJOCO_INCLUDE_DIRS "${MUJOCO_INCLUDE_DIR}")

find_library(MUJOCO_CORE_LIBRARY mujoco DOC "MuJoCo core library")
find_library(MUJOCO_GLFW_LIBRARY libglfw.so.3 DOC "MuJoCo GLFW library")
set(MUJOCO_LIBRARIES "${MUJOCO_CORE_LIBRARY}" "${MUJOCO_GLFW_LIBRARY}")

find_package_handle_standard_args(MuJoCo DEFAULT_MSG
  MUJOCO_INCLUDE_DIRS MUJOCO_LIBRARIES)
