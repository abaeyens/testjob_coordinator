# testjob_coordinator
A ROS 2 package that runs `launch_testing` (integration) tests
in parallel in a coordinated way.

The goal of this package is threefold:
- Run integration test suites significantly faster
  (up to about 64 times, depending on the host machine) than `colcon test`,
  making full use of the available host hardware.
- Avoid test contamination of all forms common in ROS 2 integration tests -
  such as interchanged messages due to common `ROS_DOMAIN_ID`s,
  timeouts due to insufficient CPU resources available,
  and the host crashing due to running out of memory.
- Minimize developer effort - no host specific configurations etc.

This package is a **PROOF OF CONCEPT**.
It has not been elaborately designed or polished.
It focuses just on core functionality -
"easily run integration tests in parallel".
Expect rough edges and spaghetti code, and no bells and whistles.
It's purpose is to see whether it's worth continuing pursuit of this approach
before spending resources on a proper design.


## How to use this package
### Standalone, see what it's like
A ROS 2 Jazzy Docker setup and an example integration test (package 'app')
are included to get started quickly.
```
git clone git@github.com:abaeyens/testjob_coordinator.git
cd testjob_coordinator
docker compose build --pull
docker compose run --rm app bash

colcon build
source install/setup.bash
colcon test --event-handlers console_direct+
ros2 run testjob_coordinator run_tests

colcon test-result --all
xunit-viewer -r build/app/test_results -c
```

To read `run_tests` argument documentation:  
`ros2 run testjob_coordinator run_tests --help`.

To list tests:  
`ros2 run testjob_coordinator list_tests`.


### In your project
- Clone or copy it into your workspace, alongside your other packages.
- Add your [launch_testing](https://index.ros.org/p/launch_testing/)
  tests with the CMake macro `testjob_coordinator_add_launch_test`
  (instead of with [launch_testing_ament_cmake](
      https://index.ros.org/p/launch_testing_ament_cmake/)'s
  `add_launch_test`).
  If desired, specify on how many CPU cores the test should be run with,
  how much time and RAM it's allowed to use,
  and which resources it requires that cannot be shared with other tests.
  Example:
  ```cmake
  testjob_coordinator_add_launch_test(test/test_integration_123.py
      TIMEOUT 60
      NUM_CPU_CORES 3
      RAM_GB 0.5
      EXCLUSIVE_RESOURCES robot_arm)
  ```
  (There's a more complete example in
  [src/app/CMakeLists.txt](src/app/CMakeLists.txt).)
- Build and source as usual: `colcon build && source install/setup.bash`.
- Run your tests with
  ```
  ros2 run testjob_coordinator run_tests
  ```
  This
  - Gathers all `testjob_coordinator` tests.
  - Sorts them for fast parallel execution.
  - Runs each of them with a unique `ROS_DOMAIN_ID` (to avoid crosstalk)
    and constrained to the assigned resources (CPU, RAM, ...),
    as many of them in parallel as the host machine allows.
  - Test output (xunit files) are stored in the same place
    as `colcon test` would do, so all existing tools for analyzing test output
    (e.g. `xunit-viewer`) 'just work'.
- View your test results with `colcon test-result --all`
  or, more visually, with (see [Dockerfile](Dockerfile) for install commands)
  ```
  xunit-viewer -r build -c
  ```

As `testjob_coordinator` is still just a PoC,
please understand that some exotic configurations may not work.


## Provide feedback
Is `testjob_coordinator` useful for you? Why (not)?
Which features do you find useful, which not? Which would you like to see?
I'd greatly appreciate if you could share your feedback as a GitHub issue.
