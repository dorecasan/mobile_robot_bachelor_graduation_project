#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/dorecasan/my_ros/robot_lab/src/rosserial/rosserial_vex_cortex"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/dorecasan/my_ros/robot_lab/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/dorecasan/my_ros/robot_lab/install/lib/python2.7/dist-packages:/home/dorecasan/my_ros/robot_lab/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/dorecasan/my_ros/robot_lab/build" \
    "/usr/bin/python2" \
    "/home/dorecasan/my_ros/robot_lab/src/rosserial/rosserial_vex_cortex/setup.py" \
     \
    build --build-base "/home/dorecasan/my_ros/robot_lab/build/rosserial/rosserial_vex_cortex" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/dorecasan/my_ros/robot_lab/install" --install-scripts="/home/dorecasan/my_ros/robot_lab/install/bin"
