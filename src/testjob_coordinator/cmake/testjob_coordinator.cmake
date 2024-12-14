
#
# Clears all existing test file
#
# Not strictly necessary but good practice, avoids cache issues on rebuild
function(testjob_coordinator_init)
  file(REMOVE_RECURSE "${CMAKE_CURRENT_BINARY_DIR}/coordinator_integration_tests")
endfunction()

#
# Add a coordinated integration test
#
# :param FILENAME: The launch test file containing the test to run
# :type FILENAME: string
# :param NUM_CPU_CORES: Number of CPU cores to be used for this test.
# :type NUM_CPU_CORES: number (optional)
# :param RAM_MB: Megabytes of RAM to be used for this test.
# :type RAM_MB: number (optional)
# :param EXCLUSIVE_RESOURCES: resources that cannot be shared between tests.
# :type EXCLUSIVE_RESOURCES: comma-separated string (optional)
function(testjob_coordinator_add_launch_test FILENAME)
  set(options)
  set(oneValueArgs
    TIMEOUT
    NUM_CPU_CORES
    RAM_GB
    EXCLUSIVE_RESOURCES
  )
  set(multiValueArgs)

  cmake_parse_arguments(COORDINATOR
      "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  file(TO_CMAKE_PATH "${FILENAME}" FILENAME)

  # create directory and file
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/testjob_coordinator_integration_tests")
  set(outname "${CMAKE_CURRENT_BINARY_DIR}/testjob_coordinator_integration_tests/${FILENAME}.yml")
  file(WRITE ${outname} "---\n")
  # filename
  file(APPEND ${outname} "filename: ${FILENAME}\n")
  # result
  string(REPLACE "/" "_" FILENAME_CLEANED ${FILENAME})
  set(COORDINATOR_RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${FILENAME_CLEANED}.xunit.xml")
  file(APPEND ${outname} "result: ${COORDINATOR_RESULT_FILE}\n")
  # other keys
  if(DEFINED COORDINATOR_TIMEOUT)
    file(APPEND ${outname} "timeout: ${COORDINATOR_TIMEOUT}\n")
  endif()
  if(DEFINED COORDINATOR_NUM_CPU_CORES)
    file(APPEND ${outname} "num_cpu_cores: ${COORDINATOR_NUM_CPU_CORES}\n")
  endif()
  if(DEFINED COORDINATOR_RAM_MB)
    file(APPEND ${outname} "ram_mb: ${COORDINATOR_RAM_MB}\n")
  endif()
  if(DEFINED COORDINATOR_EXCLUSIVE_RESOURCES)
    file(APPEND ${outname} "exclusive_resources: ${COORDINATOR_EXCLUSIVE_RESOURCES}\n")
  endif()
endfunction()
