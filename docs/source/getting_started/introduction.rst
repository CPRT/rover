.. _introduction:



Introduction
******************************************************************
The Elevaton Mapping CuPy software package represents an advancement in robotic navigation and locomotion. 
Integrating with the Robot Operating System (ROS) and utilizing GPU acceleration, this framework enhances point cloud registration and ray casting,
crucial for efficient and accurate robotic movement, particularly in legged robots.

Used for Various Real-World Applications
-------------------------------------------------------------------
This software package has been rigorously tested in challenging environments, including the DARPA Subterranean Challenge,
demonstrating its effectiveness in complex, real-world scenarios.
It supports a wide range of applications, from underground exploration to advanced research in legged locomotion and autonomous navigation.

* **DARPA Subterranean Challenge**: This package was used by Team CERBERUS in the DARPA Subterranean Challenge. 

  `Team Cerberus <https://www.subt-cerberus.org/>`_

  `CERBERUS in the DARPA Subterranean Challenge (Science Robotics) <https://www.science.org/doi/10.1126/scirobotics.abp9742>`_

* **ESA / ESRIC Space Resources Challenge**: This package was used for the Space Resources Challenge.

  `Scientific exploration of challenging planetary analog environments with a team of legged robots (Science Robotics) <https://www.science.org/doi/full/10.1126/scirobotics.ade9548>`_




Key Features
-------------------------------------------------------------------
* **Height Drift Compensation**: Tackles state estimation drifts that can create mapping artifacts, ensuring more accurate terrain representation.

* **Visibility Cleanup and Artifact Removal**: Raycasting methods and an exclusion zone feature are designed to remove virtual artifacts and correctly interpret overhanging obstacles, preventing misidentification as walls.

* **Learning-based Traversability Filter**: Assesses terrain traversability using local geometry, improving path planning and navigation.

* **Versatile Locomotion Tools**: Incorporates smoothing filters and, in legacy setups, plane segmentation to optimize movement across various terrains.

* **Multi-Modal Elevation Map (MEM) Framework**: Allows seamless integration of diverse data like geometry, semantics, and RGB information, enhancing multi-modal robotic perception.

* **GPU-Enhanced Efficiency**: Facilitates rapid processing of large data structures, crucial for real-time applications.

Overview
-------------------------------------------------------------------

.. image:: ../../media/overview.png
   :alt: Overview of multi-modal elevation map structure

Overview of our multi-modal elevation map structure. The framework takes multi-modal images (purple) and multi-modal (blue) point clouds as
input. This data is input into the elevation map by first associating the data to the cells and then fused with different fusion algorithms into the various
layers of the map. Finally the map can be post-processed with various custom plugins to generate new layers (e.g. traversability) or process layer for
external components (e.g. line detection).

Subscribed Topics
-------------------------------------------------------------------
The subscribed topics are specified under **subscribers** parameter.

Example setup is in **elevation_mapping_cupy/config/core/example_setup.yaml**.

* **/<point_cloud_topic>** ([sensor_msgs/PointCloud2])

  The point cloud topic. It can have additional channels for RGB, intensity, etc.

* **/<image_topic>** ([sensor_msgs/Image])

  The image topic. It can have additional channels for RGB, semantic probabilities, image features etc.

* **/<camera_info>** ([sensor_msgs/CameraInfo])

  The camera info topic. It is used to project the point cloud into the image plane.

* **/<channel_info>** ([elevation_map_msgs/ChannelInfo])

  If this topic is configured, the node will subscribe to it and use the information to associate the image channels to the elevation map layers.

* **/tf** ([tf/tfMessage])

  The transformation tree.


Published Topics
-------------------------------------------------------------------
For elevation_mapping_cupy, topics are published as set in the rosparam.
You can specify which layers to publish in which fps.

Under publishers, you can specify the topic_name, layers basic_layers and fps.

.. code-block:: yaml

  publishers:
      your_topic_name:
        layers: [ 'list_of_layer_names', 'layer1', 'layer2' ] # Choose from 'elevation', 'variance', 'traversability', 'time' + plugin layers
        basic_layers: [ 'list of basic layers', 'layer1' ] # basic_layers for valid cell computation (e.g. Rviz): Choose a subset of `layers`.
        fps: 5.0  # Publish rate. Use smaller value than `map_acquire_fps`.


Example setting in `config/parameters.yaml`.

* **elevation_map_raw** ([grid_map_msg/GridMap])

  The entire elevation map.

* **elevation_map_recordable** ([grid_map_msg/GridMap])

  The entire elevation map with slower update rate for visualization and logging.

* **elevation_map_filter** ([grid_map_msg/GridMap])

  The filtered maps using plugins.
