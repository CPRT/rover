# compressed_telemetry_cpp

C++ rewrite of the `compressed_telemetry` Python package. It provides four ROS 2 nodes that compress and decompress costmap and gridmap telemetry for low-bandwidth links. All nodes are registered as composable components.

## Nodes

| Node | Plugin class | Subscribes | Publishes |
| --- | --- | --- | --- |
| CostmapCompressor | `compressed_telemetry_cpp::CostmapCompressor` | `/global_costmap/costmap`, `/global_costmap/costmap_updates` | `/telemetry/costmap_full_compressed`, `/telemetry/costmap_updates_compressed` |
| CostmapDecompressor | `compressed_telemetry_cpp::CostmapDecompressor` | `/telemetry/costmap_full_compressed`, `/telemetry/costmap_updates_compressed` | `/viz/global_costmap/costmap`, `/viz/global_costmap/costmap_updates` |
| GridmapCompressor | `compressed_telemetry_cpp::GridmapCompressor` | `/traversability_map` | `/telemetry/gridmap_compressed` |
| GridmapDecompressor | `compressed_telemetry_cpp::GridmapDecompressor` | `/telemetry/gridmap_compressed` | `/viz/traversability_map` |

## Composable usage

### Python launch example (all four nodes in one container)

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name="compressed_telemetry_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="compressed_telemetry_cpp",
                    plugin="compressed_telemetry_cpp::CostmapCompressor",
                    name="costmap_compressor",
                ),
                ComposableNode(
                    package="compressed_telemetry_cpp",
                    plugin="compressed_telemetry_cpp::CostmapDecompressor",
                    name="costmap_decompressor",
                ),
                ComposableNode(
                    package="compressed_telemetry_cpp",
                    plugin="compressed_telemetry_cpp::GridmapCompressor",
                    name="gridmap_compressor",
                ),
                ComposableNode(
                    package="compressed_telemetry_cpp",
                    plugin="compressed_telemetry_cpp::GridmapDecompressor",
                    name="gridmap_decompressor",
                ),
            ],
            output="screen",
        )
    ])
```

### CLI example (load nodes into an existing container)

Assuming you already have a container running (for example `component_container_mt`):

```bash
ros2 component load /compressed_telemetry_container compressed_telemetry_cpp compressed_telemetry_cpp::CostmapCompressor
ros2 component load /compressed_telemetry_container compressed_telemetry_cpp compressed_telemetry_cpp::CostmapDecompressor
ros2 component load /compressed_telemetry_container compressed_telemetry_cpp compressed_telemetry_cpp::GridmapCompressor
ros2 component load /compressed_telemetry_container compressed_telemetry_cpp compressed_telemetry_cpp::GridmapDecompressor
```

## Parameters

| Node | Parameter | Default | Description |
| --- | --- | --- | --- |
| CostmapCompressor | `log_compression_stats` | `true` | Log per-message compression stats. |
| CostmapCompressor | `log_mbps_stats` | `true` | Log 10-second average compressed bandwidth. |
| GridmapCompressor | `log_compression_stats` | `true` | Log per-message compression stats. |
| GridmapCompressor | `log_mbps_stats` | `true` | Log 10-second average compressed bandwidth. |
