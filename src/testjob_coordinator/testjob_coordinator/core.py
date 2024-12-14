import dataclasses
from multiprocessing import Array, Process, Value
import os
import resource
import socket
import subprocess
import time


import domain_coordinator


PORT_BASE = 32768


@dataclasses.dataclass
class Test:
    package: str
    filename: str
    result: str = None
    timeout: int = 60
    num_cpu_cores: int = 1
    ram_mb: int = 512
    exclusive_resources: set = dataclasses.field(default_factory=set)

    def __post_init__(self):
        if not isinstance(self.exclusive_resources, set):
            if isinstance(self.exclusive_resources, str):
                self.exclusive_resources = set(self.exclusive_resources.split(','))
            elif type(self.exclusive_resources) in (tuple, list):
                self.exclusive_resources = set(self.exclusive_resources)
            else:
                raise ValueError(
                    f'Failed to parse exclusive resources, got type '
                    f'{self.exclusive_resources} but expected set or str.')

    def __str__(self):
        return (
            f'Test(package: {self.package}, filename: {self.filename}, '
            f'{self.num_cpu_cores} CPU cores, {self.ram_mb} MB RAM)')


def sort_tests(tests):
    """Sort tests to the rule 'most demanding ones first'
    Idea is to start with a large test (many cores, many exclusive resources)
    and fill up what's left with smaller tests, and avoid the situation
    where we start with many small tests simultaneously and having to finish
    with each large test running alone.
    Heuristic: primarily sort on number of exclusive resources required,
    secondarily sort on number of CPU cores required
    """
    # Sort them on number of CPU cores required, decreasing
    tests = sorted(tests, key=lambda t: t.num_cpu_cores, reverse=True)
    # Sort them on number of exclusive resources required, decreasing
    tests = sorted(tests, key=lambda t: len(t.exclusive_resources), reverse=True)
    return tests


def execute_test(
        test, domain_id, domain_socket, cores_to_use,
        cores_in_use, ram_mb_in_use, exclusive_resources_in_use,
        exclusive_resources_name_to_idx):
    """Run the test as a subprocess"""
    assert len(cores_to_use) > 0

    # Set domain id
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    # Set memory resource limits
    # use RLIMIT_DATA as proxy for RAM usage
    resource.setrlimit(
        resource.RLIMIT_DATA, (test.ram_mb * 1024**2, test.ram_mb * 1024**2))

    # Run test in subprocess
    cores = ','.join(str(c) for c in cores_to_use)
    cmd = ['taskset', '-c', cores,
           'python3',
           '-m', 'launch_testing.launch_test',
           '--package-name', test.package,
           os.path.join('src', test.package, test.filename),
           '--junit-xml', test.result]
    print(f'Starting test {test.package}/{test.filename} on core(s) {cores} '
          f'with ROS_DOMAIN_ID {domain_id} '
          f'and exclusive resources {", ".join(test.exclusive_resources)}, '
          f'command: {" ".join(cmd)}', flush=True)
    subprocess.run(cmd, shell=False, timeout=Test.timeout)

    # Release ROS_DOMAIN_ID
    domain_socket.close()
    # Release cores, memory and exclusive resources
    with cores_in_use.get_lock():
        for j in cores_to_use:
            assert cores_in_use[j] == 1
            cores_in_use[j] = 0
        ram_mb_in_use.value -= test.ram_mb
        assert ram_mb_in_use.value >= 0
        for er in test.exclusive_resources:
            eri = exclusive_resources_name_to_idx[er]
            assert exclusive_resources_in_use[eri] == 1
            exclusive_resources_in_use[eri] = 0


class TestJobCoordinator:
    def __init__(self, tests, num_cpu_cores, ram_mb):
        self.tests_pending = tests
        self.total_cores = int(num_cpu_cores)
        self.total_ram_mb = int(ram_mb)
        self._domain_id_selector = domain_coordinator.impl.default_selector()

        # For the exclusive resources, we share the usage between processes
        # as an array and keep a map from name to index in that array,
        # because a set cannot be easily shared between processes.
        exclusive_resource_names = sorted(list(set(
            er for t in self.tests_pending for er in t.exclusive_resources)))
        self._exclusive_resources_name_to_idx = \
            {name: idx for idx, name in enumerate(exclusive_resource_names)}

        # Resource counters are shared to subprocess so are lockable
        # General principle: allocation happens in this process subject to GIL
        # and release occurs in the spawned subprocesses.
        self._cores_in_use = Array('b', num_cpu_cores)
        self._ram_mb_in_use = Value('i', 0)
        self._exclusive_resources_in_use = \
            Array('b', len(self._exclusive_resources_name_to_idx))

        # Handle case single test requires more than host offers
        for test in self.tests_pending:
          if test.num_cpu_cores > self.total_cores:
            print(f'WARNING: reducing test cores from {test.num_cpu_cores} to {self.total_cores}.')
            test.num_cpu_cores = self.total_cores
          if test.ram_mb > self.total_ram_mb:
            print(f'WARNING: reducing test RAM size from {test.ram_mb} to {self.total_ram_mb} MB.')
            test.ram_mb = self.total_ram_mb

    def run(self):
        processes = []
        while self.tests_pending:
            for test in self.tests_pending[:]:
                test_all_exclusive_resources_available = all(
                    not self._exclusive_resources_in_use[self._exclusive_resources_name_to_idx[er]]
                    for er in test.exclusive_resources)
                if sum(self._cores_in_use) + test.num_cpu_cores <= self.total_cores \
                        and self._ram_mb_in_use.value + test.ram_mb <= self.total_ram_mb \
                        and test_all_exclusive_resources_available:
                    # Acquire a ROS_DOMAIN_ID
                    for _ in range(100):
                        domain_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        try:
                            domain_id = self._domain_id_selector()
                            domain_socket.bind(('', PORT_BASE + domain_id))
                            break
                        except Exception:
                            domain_socket.close()
                    else:
                        break
                    # Acquire cores and memory
                    cores_to_use = []
                    with self._cores_in_use.get_lock():
                        # TODO can we do this cleaner?
                        for _ in range(test.num_cpu_cores):
                            for j in range(len(self._cores_in_use)):
                                if self._cores_in_use[j] == 0:
                                    self._cores_in_use[j] = 1
                                    cores_to_use.append(j)
                                    break
                        assert len(cores_to_use) == test.num_cpu_cores
                        self._ram_mb_in_use.value += test.ram_mb
                        for er in test.exclusive_resources:
                            eri = self._exclusive_resources_name_to_idx[er]
                            self._exclusive_resources_in_use[eri] = 1
                    # Handle running test in separate process to escape GIL
                    process = Process(target=execute_test, args=(
                        test, domain_id, domain_socket, cores_to_use,
                        self._cores_in_use, self._ram_mb_in_use,
                        self._exclusive_resources_in_use,
                        self._exclusive_resources_name_to_idx))
                    process.start()
                    processes.append(process)
                    self.tests_pending.remove(test)
            # TODO no polling and start new tests when existing ones return
            time.sleep(0.5)
        # Wait until all processes finished
        for process in processes:
            process.join()
