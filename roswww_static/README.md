# roswww_static

roswww_static creates a static web directory for your ROS-powered system with symlinks to all the `www` subdirectories found in your ROS packages. This way you can use any external web server (e. g. [nginx](https://nginx.org/), [Monkey](https://github.com/monkey/monkey), [Caddy](https://caddyserver.com)) to serve you static data, in compatible with `roswww` manner.

This is the ROS 2 port of `roswww_static` package from [CopterExpress/clover](https://github.com/CopterExpress/clover).

Note: you should configure your web server to make it follow symlinks.

## Instructions

* Run `update` script and it will generate the symlinks and index file: `ros2 run roswww_static update`.
* Point your static web server path to `~/.ros/www`.

You can rerun `update` if the list of installed packages changes.

The `www` subdirectory of a package is searched in its share directory (`<prefix>/share/<package>/www`), so the package should install it, for example:

```cmake
install(DIRECTORY www DESTINATION share/${PROJECT_NAME})
```

Only the packages of the sourced workspaces (`AMENT_PREFIX_PATH`) are found.

## Parameters

Parameters are passed through environment variables:

* `ROSWWW_INDEX` – path for index page, otherwise packages list would be generated.
* `ROSWWW_DEFAULT` – if set then the index page would redirect to this package's page.
* `ROS_HOME` – ROS home directory, the web directory is `$ROS_HOME/www` (`~/.ros/www` by default).
