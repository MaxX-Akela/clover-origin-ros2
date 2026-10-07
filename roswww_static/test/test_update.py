# Test of the update script with fake packages in a temporary prefix and a temporary ROS home.

import os
import subprocess
import sys

import pytest

UPDATE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src', 'update')


@pytest.fixture
def prefix(tmp_path):
    # ament prefix with two packages, one of them has www directory
    prefix = tmp_path / 'prefix'
    for name in 'with_www', 'without_www':
        (prefix / 'share' / 'ament_index' / 'resource_index' / 'packages').mkdir(parents=True, exist_ok=True)
        (prefix / 'share' / 'ament_index' / 'resource_index' / 'packages' / name).write_text('')
        (prefix / 'share' / name).mkdir(parents=True)
    www = prefix / 'share' / 'with_www' / 'www'
    (www / 'js').mkdir(parents=True)
    (www / 'index.html').write_text('<h1>test</h1>')
    (www / 'js' / 'test.js').write_text('var test;')
    return prefix


def update(tmp_path, prefix, home=None, **variables):
    env = dict(os.environ)
    for name in 'ROS_HOME', 'ROSWWW_INDEX', 'ROSWWW_DEFAULT':
        env.pop(name, None)
    env['AMENT_PREFIX_PATH'] = str(prefix)  # only the fake packages
    env['HOME'] = str(tmp_path / 'home')
    if home is not None:
        env['ROS_HOME'] = str(home)
    env.update(variables)
    res = subprocess.run([sys.executable, UPDATE], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = res.stdout.decode()
    assert res.returncode == 0, output
    return output


def check_links(www, prefix):
    assert sorted(os.listdir(www)) == ['index.html', 'with_www']
    link = www / 'with_www'
    assert link.is_symlink()
    assert os.path.realpath(link) == os.path.realpath(prefix / 'share' / 'with_www' / 'www')
    # the links work
    assert (link / 'index.html').read_text() == '<h1>test</h1>'
    assert (link / 'js' / 'test.js').read_text() == 'var test;'


def test_packages_list(tmp_path, prefix):
    # ~/.ros/www is used by default
    output = update(tmp_path, prefix)
    www = tmp_path / 'home' / '.ros' / 'www'
    assert 'using www dir: %s' % www in output
    assert 'found www path for with_www package' in output and 'without_www' not in output
    check_links(www, prefix)
    assert (www / 'index.html').read_text() == \
        '<h1>Packages list</h1>\n<ul>\n<li><a href="with_www/">with_www</a></li>'


def test_ros_home(tmp_path, prefix):
    home = tmp_path / 'ros_home'
    home.mkdir()
    update(tmp_path, prefix, home=home)
    check_links(home / 'www', prefix)
    assert not (tmp_path / 'home' / '.ros').exists()


def test_reset(tmp_path, prefix):
    # the content of www directory is reset
    home = tmp_path / 'ros_home'
    (home / 'www' / 'old').mkdir(parents=True)
    (home / 'www' / 'old.html').write_text('old')
    update(tmp_path, prefix, home=home)
    check_links(home / 'www', prefix)


def test_default_package(tmp_path, prefix):
    home = tmp_path / 'ros_home'
    home.mkdir()
    update(tmp_path, prefix, home=home, ROSWWW_DEFAULT='with_www')
    check_links(home / 'www', prefix)
    assert (home / 'www' / 'index.html').read_text() == '<meta http-equiv=refresh content="0; url=with_www/">'


def test_index_file(tmp_path, prefix):
    home = tmp_path / 'ros_home'
    home.mkdir()
    index = tmp_path / 'custom.html'
    index.write_text('custom index')
    output = update(tmp_path, prefix, home=home, ROSWWW_INDEX=str(index))
    assert 'symlinking index file' in output
    check_links(home / 'www', prefix)
    assert (home / 'www' / 'index.html').is_symlink()
    assert (home / 'www' / 'index.html').read_text() == 'custom index'
