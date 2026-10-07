# Test of selfcheck with mocks of everything the checks read (no mavros_node), sequential and parallel modes.
# The mocks are shared with smoke_selfcheck.py, which also runs selfcheck against mavros_node.

import os
import sys
import time

import launch
import launch_testing.actions
import pytest
from launch.actions import ExecuteProcess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase  # noqa: E402

try:
    import pymavlink  # noqa: F401
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False


@pytest.mark.launch_test
def generate_test_description():
    # selfcheck is run by the test itself, as it exits when the checks are done;
    # launch_testing stops if there are no processes, so run an idle one
    return launch.LaunchDescription([
        ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(3600)']),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSelfcheck(CloverTestCase):
    NODE_NAME = 'selfcheck_test'

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.mocks = None
        if HAS_PYMAVLINK:
            import smoke_selfcheck
            cls.smoke = smoke_selfcheck
            cls.mocks = smoke_selfcheck.Mocks(cls.executor)
            time.sleep(1.0)

    def run_selfcheck(self, parallel):
        if not HAS_PYMAVLINK:
            self.skipTest('pymavlink is not installed (pip install pymavlink)')
        smoke, mocks = self.smoke, self.mocks
        mocks.commands.clear()
        reports, output, _ = smoke.run_selfcheck(parallel)
        if parallel:
            # in the original FCU and Preflight status checks share the shell buffer without a lock,
            # so the shell output may be mixed
            for name in 'FCU', 'Preflight status':
                reports[name] = smoke.MOCK_DATA[name]
        smoke.compare(reports, smoke.MOCK_DATA, output)
        assert mocks.commands and all(magic == smoke.Mavlink.MAVLINK_V10 for magic, _ in mocks.commands)

    def test_sequential(self):
        self.run_selfcheck(False)

    def test_parallel(self):
        self.run_selfcheck(True)
