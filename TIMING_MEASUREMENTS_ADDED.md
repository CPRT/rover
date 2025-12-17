# Timing Measurements Added to elevation_mapping_node.py

## Summary
Applied comprehensive timing measurements to identify the bottleneck between input processing (42ms) and output publishing (causing 0.6 Hz output rate = ~1.6s cycles).

## Key Hypothesis from Analysis
The problem is NOT in pointcloud processing (42ms is fast), but in the **publish_map** function which is suspected to take ~1.5-1.6 seconds and blocks the SingleThreadedExecutor.

## Timing Measurements Added

### 1. **publish_map() - Main Publisher Function**
This is the suspected bottleneck. Now measures:

- **Overall timing**: Total time for entire publish_map call
- **Breakdown by stage**:
  - `get_center`: Getting map center position
  - `init_message`: Initializing GridMap message structure
  - `all_layers_total`: Total time processing all layers
  - `publish_call`: Actual ROS publish call
  
- **Per-layer detailed timing** for each layer published:
  - `gpu_to_cpu`: Time to transfer data from GPU VRAM to CPU RAM via `get_map_with_name_ref()`
  - `serialization`: Time to convert numpy array to ROS Float32MultiArray
  - `total`: Total time for that layer

**Recorded metrics**:
- `publish_map_{key}` - Total duration
- `publish_{key}_{stage}` - Each breakdown stage
- `publish_{key}_layer_{name}_gpu_to_cpu` - GPU transfer per layer
- `publish_{key}_layer_{name}_serialization` - Serialization per layer
- `publish_{key}_layer_{name}_total` - Total per layer

### 2. **_numpy_to_multiarray() - Serialization Helper**
Measures the expensive numpy → ROS message conversion:
- Records timing only if duration > 1ms (to avoid log spam)
- Metric: `_numpy_to_multiarray`

### 3. **handle_save_map() - Map Saving Service**
Comprehensive breakdown of map saving operation:
- `prepare_paths`: Path validation and setup
- `collect_layers`: Gathering layer names
- `export_fused`: Exporting fused layers from GPU
- `export_raw`: Exporting raw layers from GPU
- `build_fused_msg`: Building fused GridMap message
- `build_raw_msg`: Building raw GridMap message
- `write_fused_bag`: Writing fused bag to disk
- `write_raw_bag`: Writing raw bag to disk

**Recorded metrics**:
- `handle_save_map` - Total duration
- `save_map_{stage}` - Each breakdown stage

### 4. **_build_grid_map_message() - Message Construction**
Measures message building and serialization:
- `init`: Message initialization
- `all_layers`: Time to serialize all layers
- Per-layer: `build_msg_layer_{name}` for each individual layer

**Recorded metrics**:
- `_build_grid_map_message` - Total duration
- `build_msg_{stage}` - Breakdown stages
- `build_msg_layer_{name}` - Per-layer serialization

### 5. **Enhanced pointcloud_callback() - Already Added**
Previously added detailed breakdown:
- `numpify`: Converting ROS message to numpy
- `validation`: Data validation checks
- `tf_lookup`: TF transform lookup
- `transform_extract`: Extracting transform data
- `point_extraction`: Extracting XYZ and channels
- `input_pointcloud`: Main CUDA processing

**Recorded metrics**:
- `pointcloud_callback_{sub_key}` - Total duration
- `pointcloud_{sub_key}_{stage}` - Each breakdown stage

## Expected Findings

Based on the analysis provided, we expect to find:

1. **pointcloud_callback**: ~42ms (already measured, this is GOOD ✅)
2. **publish_map**: ~1500-1600ms (THE BOTTLENECK 🛑)
   - Likely dominated by:
     - GPU→CPU transfer: Large map data moving from VRAM to RAM
     - Serialization: Converting numpy arrays to Float32MultiArray (Python overhead)

3. **Occasional spikes** (100-200ms) in input_pointcloud due to:
   - CUDA memory paging
   - Map expansion/shifting when entering new territory
   - These are ACCEPTABLE

## Next Steps (Recommended by Analysis)

### Immediate:
1. **Confirm the bottleneck**: Run with these timings and verify publish_map takes ~1.5s
2. **Check timing CSV**: `/tmp/elevation_mapping_timing.csv`

### Fixes to Apply:
1. **Switch to MultiThreadedExecutor** (separate PR):
   ```python
   from rclpy.executors import MultiThreadedExecutor
   from rclpy.callback_groups import ReentrantCallbackGroup
   ```
   
2. **Reduce publisher FPS** in config:
   - Lower from 1.0 Hz to 0.5 Hz or 0.2 Hz
   - Publishing full GridMap at 1 Hz is unnecessary for most applications

3. **Optimize serialization** (if confirmed as bottleneck):
   - Consider using buffer protocols
   - Investigate zero-copy serialization options
   - Profile `encode_layer_to_multiarray()` function

## Files Modified
- `/workspaces/CPRT/rover/src/third-party/elevation_mapping_cupy/elevation_mapping_cupy/scripts/elevation_mapping_node.py`

## Verification Commands

```bash
# Run the node
ros2 run elevation_mapping_cupy elevation_mapping_node

# Check output rates
ros2 topic hz /elevation_mapping/elevation_map_recordable

# Analyze timing CSV
python3 timing_csv_graphs.py  # if exists

# Or manually inspect
tail -f /tmp/elevation_mapping_timing.csv
```

## Expected Log Output Example

```
[elevation_mapping_node]: Performance timing enabled. Logging to /tmp/elevation_mapping_timing.csv
...
[elevation_mapping_node]: === Performance Timing Stats ===
[elevation_mapping_node]: pointcloud_callback_front: count=100, avg=42.50ms, min=38.20ms, max=189.45ms
[elevation_mapping_node]: publish_map_elevation_map_recordable: count=10, avg=1523.40ms, min=1450.00ms, max=1687.22ms
[elevation_mapping_node]: publish_elevation_map_recordable_all_layers_total: count=10, avg=1400.23ms, ...
[elevation_mapping_node]: publish_elevation_map_recordable_publish_call: count=10, avg=98.45ms, ...
[elevation_mapping_node]: ================================
```

This will pinpoint exactly where the 1.6s delay comes from!
