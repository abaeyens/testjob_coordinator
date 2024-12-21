cmake_minimum_required(VERSION 3.8)
project(app)

find_package(ament_cmake REQUIRED)


#########
# build #
#########


###########
# install #
###########


########
# test #
########

if(BUILD_TESTING)
  # Python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_unit.py
    # add other test pytest files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

  # C++ unit tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_unit_cpp test/test_unit.cpp)
  # add other unit test files here

  # Integration tests
  find_package(testjob_coordinator REQUIRED)
  testjob_coordinator_init()
  testjob_coordinator_add_launch_test(test/test_integration.py
      NUM_CPU_CORES 1 RAM_GB 0.5)
  testjob_coordinator_add_launch_test(test/test_integration_bis.py
      NUM_CPU_CORES 3 RAM_GB 0.5)
  # add other integration test files here
endif()

ament_package()
