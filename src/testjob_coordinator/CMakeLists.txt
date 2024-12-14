cmake_minimum_required(VERSION 3.5)
project(testjob_coordinator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

###########
# install #
###########
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
    scripts/run_tests
  DESTINATION lib/${PROJECT_NAME}
)

########
# test #
########
if(BUILD_TESTING)
  # Python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_core.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
