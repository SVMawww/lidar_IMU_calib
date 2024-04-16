# Install script for directory: /home/ctw/code/ros/catkin_li_calib/ceres

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres" TYPE FILE FILES
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/autodiff_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/autodiff_local_parameterization.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/c_api.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/ceres.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/conditioned_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/context.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/cost_function_to_functor.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/covariance.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/crs_matrix.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/cubic_interpolation.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/dynamic_autodiff_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/dynamic_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/dynamic_cost_function_to_functor.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/dynamic_numeric_diff_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/evaluation_callback.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/fpclassify.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/gradient_checker.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/gradient_problem.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/gradient_problem_solver.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/iteration_callback.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/jet.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/local_parameterization.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/loss_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/normal_prior.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/numeric_diff_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/numeric_diff_options.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/ordered_groups.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/problem.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/rotation.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/sized_cost_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/solver.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/tiny_solver.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/tiny_solver_autodiff_function.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/tiny_solver_cost_function_adapter.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/types.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/version.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/autodiff.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/disable_warnings.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/eigen.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/fixed_array.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/macros.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/manual_constructor.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/numeric_diff.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/port.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/reenable_warnings.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/scoped_ptr.h"
    "/home/ctw/code/ros/catkin_li_calib/ceres/include/ceres/internal/variadic_evaluate.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES "/home/ctw/code/ros/catkin_li_calib/ceres/bin/config/ceres/internal/config.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake"
         "/home/ctw/code/ros/catkin_li_calib/ceres/bin/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/ctw/code/ros/catkin_li_calib/ceres/bin/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/ctw/code/ros/catkin_li_calib/ceres/bin/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE RENAME "CeresConfig.cmake" FILES "/home/ctw/code/ros/catkin_li_calib/ceres/bin/CeresConfig-install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES
    "/home/ctw/code/ros/catkin_li_calib/ceres/bin/CeresConfigVersion.cmake"
    "/home/ctw/code/ros/catkin_li_calib/ceres/cmake/FindEigen.cmake"
    "/home/ctw/code/ros/catkin_li_calib/ceres/cmake/FindGlog.cmake"
    "/home/ctw/code/ros/catkin_li_calib/ceres/cmake/FindGflags.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ctw/code/ros/catkin_li_calib/ceres/bin/internal/ceres/cmake_install.cmake")
  include("/home/ctw/code/ros/catkin_li_calib/ceres/bin/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ctw/code/ros/catkin_li_calib/ceres/bin/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
