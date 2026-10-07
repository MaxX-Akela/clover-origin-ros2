# Test of the web pages installation: roswww_static update makes a working link to www directory of the package.
# The pages themselves are not tested here (a browser is needed).

import os
import subprocess
import sys
import tempfile
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import TEST_DIR  # noqa: E402

SOURCE_DIR = os.path.join(os.path.dirname(TEST_DIR), 'www')
PAGES = ['index.html', 'topics.html', 'gcs.html', 'viz.html', 'console.html', 'aruco_map.html']
SCRIPTS = ['gcs.js', 'topics.js', 'viz.js', 'roslib.js', 'ros3d.js', 'three.min.js', 'yaml.js', 'eventemitter2.js']


@pytest.mark.launch_test
def generate_test_description():
    # update is run by the test itself; launch_testing stops if there are no processes, so run an idle one
    return launch.LaunchDescription([
        ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(3600)']),
        launch_testing.actions.ReadyToTest(),
    ])


class TestWww(unittest.TestCase):
    def update(self, home, **variables):
        env = dict(os.environ, HOME=home, **variables)
        for name in 'ROS_HOME', 'ROSWWW_INDEX', 'ROSWWW_DEFAULT':
            if name not in variables:
                env.pop(name, None)
        res = subprocess.run(['ros2', 'run', 'roswww_static', 'update'], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        output = res.stdout.decode()
        assert res.returncode == 0, output
        assert 'found www path for clover package' in output, output
        return os.path.join(home, '.ros', 'www')

    def test_update(self):
        with tempfile.TemporaryDirectory() as home:
            www = self.update(home)
            link = os.path.join(www, 'clover')
            assert os.path.islink(link)
            assert os.path.realpath(link) == os.path.realpath(
                os.path.join(get_package_share_directory('clover'), 'www'))
            with open(os.path.join(www, 'index.html')) as f:
                assert '<li><a href="clover/">clover</a></li>' in f.read()

            # the pages and the scripts are available by the link and are the same as in the repository
            for name in PAGES + ['js/' + name for name in SCRIPTS]:
                with open(os.path.join(link, name), 'rb') as installed, open(os.path.join(SOURCE_DIR, name), 'rb') as source:
                    assert installed.read() == source.read(), name

            # the scripts used by the pages exist
            for page in PAGES:
                with open(os.path.join(link, page)) as f:
                    html = f.read()
                for name in SCRIPTS:
                    assert ('js/' + name in html) == (name in html)

            # the files provided by the Clover image are links, the placeholders are not installed
            assert os.readlink(os.path.join(link, 'clover.log')) == '/var/log/clover.log'
            assert os.readlink(os.path.join(link, 'clover_version')) == '/etc/clover_version'
            assert not os.path.lexists(os.path.join(link, 'docs'))
            assert not os.path.lexists(os.path.join(link, 'CATKIN_IGNORE'))

    def test_default_package(self):
        # as on the Clover image
        with tempfile.TemporaryDirectory() as home:
            www = self.update(home, ROSWWW_DEFAULT='clover')
            with open(os.path.join(www, 'index.html')) as f:
                assert f.read() == '<meta http-equiv=refresh content="0; url=clover/">'
