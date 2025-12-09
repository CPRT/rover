.. _tutorial:

Tutorial
******************************************************************
This tutorial will guide you through the basic usage of elevation_mapping_cupy.
You will learn how to run the core node and the TurtleBot example.

If you want to use your own custom plugins, please refer to the :ref:`plugins`.


Run
==================================================================

Basic usage.

.. code-block:: bash

  roslaunch elevation_mapping_cupy elevation_mapping_cupy.launch


Run TurtleBot example
==================================================================

First, install turtlebot simulation.

.. code-block:: bash

  sudo apt install ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-teleop


Then, you can run the examples. For the basic version:

.. code-block:: bash

  export TURTLEBOT3_MODEL=waffle
  roslaunch elevation_mapping_cupy turtlesim_simple_example.launch


For fusing semantics into the map such as rgb from a multi modal pointcloud:

.. image:: ../../media/turtlebot.png
    :alt: Elevation map examples

To control the robot with a keyboard, a new terminal window needs to be opened.
Then run

.. code-block:: bash

  export TURTLEBOT3_MODEL=waffle
  roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


Velocity inputs can be sent to the robot by pressing the keys `a`, `w`, `d`, `x`. To stop the robot completely, press `s`.

.. note::
   The previous semantic sensor and plane segmentation demos were removed from this workspace because they have not been validated on ROS 2.  Use a historical revision if you still need them.



Errors
==================================================================

If you build with the install flag under ros melodic, you might get issues with the modules not found:

.. code-block:: bash

  terminate called after throwing an instance of 'pybind11::error_already_set'
    what():  ModuleNotFoundError: No module named 'elevation_mapping_cupy'

This is because python3 modules are installed into a different location.

This can be fixed by including also the python3 modules location in the `PYTHONPATH` by adding following line into the launch file:

.. code-block:: xml

  <env name="PYTHONPATH" value="<path_to_your_install>/lib/python3/dist-packages:$(env PYTHONPATH)" />

If you get error such as

.. code-block:: none

  Make Error at /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
    Could NOT find PythonInterp: Found unsuitable version "2.7.18", but
    required is at least "3" (found /usr/bin/python)


Build with option.

.. code-block:: bash

  catkin build elevation_mapping_cupy -DPYTHON_EXECUTABLE=$(which python3)


