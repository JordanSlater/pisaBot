# from https://medium.com/@tahsincankose/cross-compiling-ros-project-for-arm-263642b405ac
# modified to fit aarch64-linux-gnu toolchain and noetic
#File raspberrytoolchain.cmake for ROS and system packages to cross compile.
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm64)

SET(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Below call is necessary to avoid non-RT problem.
SET(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

SET(RASPBERRY_ROOT_PATH ${CMAKE_CURRENT_LIST_DIR}/arm_rpi)
SET(RASPBERRY_NOETIC_PATH ${RASPBERRY_ROOT_PATH}/opt/ros/noetic)

SET(CMAKE_FIND_ROOT_PATH ${RASPBERRY_ROOT_PATH} ${CATKIN_DEVEL_PREFIX})

#If you have installed cross compiler to somewhere else, please specify that path.
SET(COMPILER_ROOT /usr/aarch64-linux-gnu)

#Have to set this one to BOTH, to allow CMake to find rospack
#This set of variables controls whether the CMAKE_FIND_ROOT_PATH and CMAKE_SYSROOT are used for find_xxx() operations.
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

SET(CMAKE_PREFIX_PATH ${RASPBERRY_NOETIC_PATH} ${RASPBERRY_ROOT_PATH}/usr)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)

SET(LD_LIBRARY_PATH ${RASPBERRY_NOETIC_PATH}/lib)

# added from https://github.com/mktk1117/ROS_ARM_CROSSCOMPILE/blob/master/ros_indigo/rostoolchain.cmake
SET(PYTHON_EXECUTABLE /usr/bin/python3)
SET(CMAKE_CROSSCOMPILING true)
SET(CATKIN_ENABLE_TESTING false)