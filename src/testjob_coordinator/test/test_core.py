import itertools

import pytest

import testjob_coordinator.core


def test_Test():
    test = testjob_coordinator.core.Test(
        package='app',
        filename='test/test_mylaunchtest.py',
        exclusive_resources='GPU,robot_arm',
    )
    assert test.exclusive_resources == set(('GPU', 'robot_arm'))

def test_sort_tests():
    def gen_test(name, exclusive_resources, num_cpu_cores):
        return testjob_coordinator.core.Test(
            package='app', filename=name,
            num_cpu_cores=num_cpu_cores, exclusive_resources=exclusive_resources)
    tests = [
        gen_test('0', {'GPU0', 'GPU1'}, 3),
        gen_test('1', {'GPU0', 'GPU1'}, 2),
        gen_test('2', {'GPU0'}, 4),
        gen_test('3', {'GPU1'}, 1),
        gen_test('4', set(), 2),
    ]
    for tests_permutated in itertools.permutations(tests):
        tests_sorted = testjob_coordinator.core.sort_tests(tests_permutated)
        assert [t.filename for t in tests] == [t.filename for t in tests_sorted]
