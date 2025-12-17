# Timing CSV Graphs Script Updates

## Summary
Updated `timing_csv_graphs.py` to parse and visualize all the new timing measurements added to `elevation_mapping_node.py`.

## New Callback Types Supported

### 1. **publish_map Callbacks**
The script now fully parses and analyzes:
- `publish_map_{key}` - Main publishing function
- `publish_{key}_get_center` - Getting map center position
- `publish_{key}_init_message` - Initializing GridMap message
- `publish_{key}_all_layers_total` - Total time processing all layers
- `publish_{key}_publish_call` - Actual ROS publish call
- `publish_{key}_layer_{name}_gpu_to_cpu` - GPU to CPU transfer per layer
- `publish_{key}_layer_{name}_serialization` - Serialization per layer
- `publish_{key}_layer_{name}_total` - Total time per layer

### 2. **save_map Callbacks**
- `handle_save_map` - Main save function
- `save_map_prepare_paths` - Path validation and setup
- `save_map_collect_layers` - Gathering layer names
- `save_map_export_fused` - Exporting fused layers
- `save_map_export_raw` - Exporting raw layers
- `save_map_build_fused_msg` - Building fused GridMap message
- `save_map_build_raw_msg` - Building raw GridMap message
- `save_map_write_fused_bag` - Writing fused bag to disk
- `save_map_write_raw_bag` - Writing raw bag to disk

### 3. **build_grid_map_message Callbacks**
- `_build_grid_map_message` - Main message building function
- `build_msg_init` - Message initialization
- `build_msg_all_layers` - Serializing all layers
- `build_msg_layer_{name}` - Per-layer serialization

### 4. **Other Callbacks**
- `_numpy_to_multiarray` - Numpy to ROS message conversion
- `timer_pose_update` - Pose update timer
- `timer_update_variance` - Variance update timer
- `timer_update_time` - Time update timer
- `image_callback_{sub_key}` - Image processing

## New Analysis Functions

### `analyze_publish_map(csv_path, output_path=None)`
Performs comprehensive analysis of publish_map performance:
- Shows mean, median, min, max durations for each publisher
- Breaks down timing by stage (get_center, init_message, all_layers_total, publish_call)
- Analyzes per-layer GPU transfer and serialization times
- Calculates publishing rate in Hz
- **Detects bottlenecks** (warns if median > 1s)
- **Provides optimization suggestions** based on measured data
- Creates stacked area chart visualization showing breakdown by stage

**Example output:**
```
📊 Publisher: elevation_map_recordable
  Total calls: 100
  Mean:   1523.450 ms (1.523 s)
  Median: 1500.230 ms (1.500 s)
  
  Breakdown by stage:
    get_center          : avg=   0.120 ms, median=   0.115 ms, max=   0.250 ms
    init_message        : avg=   2.340 ms, median=   2.300 ms, max=   3.100 ms
    all_layers_total    : avg=1400.230 ms, median=1380.450 ms, max=1687.220 ms
    publish_call        : avg=  98.450 ms, median=  95.200 ms, max= 120.340 ms
  
  ⚠️  WARNING: This publisher is SLOW (>1s median latency)
      This is likely the main bottleneck!
  
  💡 Optimization suggestions:
      • Layer processing takes 1400ms - consider:
        - Reduce number of published layers
        - Publish different layers at different rates
        - Use MultiThreadedExecutor to avoid blocking
```

### `analyze_save_map(csv_path)`
Analyzes map saving performance:
- Shows statistics for handle_save_map calls
- Breaks down timing by save stage
- Identifies which stages are slow (export, serialization, or disk I/O)

### `create_publisher_comparison(csv_path, output_path=None)`
Creates a comprehensive comparison plot showing:
- All callback types grouped by category:
  - Pointcloud Input
  - Image Input
  - Map Publishing
  - Timers
  - Services
- Time-series plots for each category
- Critical threshold lines (e.g., 1s line for publishers)

## Updated Command-Line Options

### New Arguments:
```bash
--analyze-publish      Analyze publish_map performance with detailed breakdown
--analyze-save         Analyze handle_save_map performance  
--compare-all          Create comparison plot of all callback types
```

### Usage Examples:

```bash
# Analyze the publish_map bottleneck
python timing_csv_graphs.py --analyze-publish

# Analyze map saving performance
python timing_csv_graphs.py --analyze-save

# Compare all callback types
python timing_csv_graphs.py --compare-all

# Full diagnostic analysis
python timing_csv_graphs.py --analyze-publish --analyze-save --analyze-outliers

# Generate visualization with breakdown
python timing_csv_graphs.py --analyze-publish -o publish_analysis.png

# Get statistics for all callbacks
python timing_csv_graphs.py --stats-only

# Filter for specific callbacks and plot
python timing_csv_graphs.py --filter publish_map --breakdown
```

## Updated Breakdown Pattern Filtering

The script now properly filters out all breakdown measurements when showing main callbacks:

```python
breakdown_patterns = [
    '_numpify', '_validation', '_tf_lookup', 
    '_transform_extract', '_point_extraction', '_input_pointcloud',
    '_get_center', '_init_message', '_all_layers_total', '_publish_call',
    '_layer_.*_(gpu_to_cpu|serialization|total)',  # NEW: layer details
    'save_map_(prepare_paths|collect_layers|...)',  # NEW: save stages
    'build_msg_(init|all_layers|layer_.*)',         # NEW: build stages
    '_numpy_to_multiarray'                          # NEW: serialization
]
```

This ensures that:
- Default plots show only top-level callbacks (cleaner view)
- Use `--breakdown` flag to see detailed timing stages
- Analysis functions automatically access breakdown data

## Compatibility with TIMING_MEASUREMENTS_ADDED.md

The script now fully supports all measurements documented in `TIMING_MEASUREMENTS_ADDED.md`:

✅ Pointcloud callback breakdown (already existed, preserved)  
✅ **publish_map breakdown** (NEW - main bottleneck analysis)  
✅ **save_map breakdown** (NEW - service call analysis)  
✅ **build_grid_map_message breakdown** (NEW - message construction)  
✅ **Layer-level timing** (NEW - GPU transfer + serialization per layer)  
✅ **Timer callbacks** (NEW - pose_update, update_variance, update_time)  
✅ Image callbacks (already existed, preserved)  

## Expected Workflow

### 1. Collect timing data:
```bash
ros2 run elevation_mapping_cupy elevation_mapping_node
# Data automatically logged to /tmp/elevation_mapping_timing.csv
```

### 2. Quick diagnostics:
```bash
# Get statistics summary
python timing_csv_graphs.py --stats-only

# Identify the bottleneck
python timing_csv_graphs.py --analyze-publish
```

### 3. Deep dive analysis:
```bash
# Full breakdown with visualizations
python timing_csv_graphs.py --analyze-publish --analyze-save --compare-all -o analysis.png
```

### 4. Investigate specific callbacks:
```bash
# Focus on pointcloud processing
python timing_csv_graphs.py --filter pointcloud --breakdown

# Check for outliers
python timing_csv_graphs.py --analyze-outliers --outlier-threshold 200
```

## Key Benefits

1. **Bottleneck Detection**: Automatically identifies slow callbacks (>1s) and provides optimization suggestions
2. **Detailed Breakdown**: Shows exactly where time is spent (GPU transfer vs serialization vs ROS publish)
3. **Visual Analysis**: Stacked area charts show how timing changes over the run
4. **Actionable Insights**: Suggests concrete fixes based on measured data
5. **Flexible Filtering**: Can focus on specific callback types or time periods
6. **Performance Tracking**: Compare timing before/after optimizations

## Files Modified
- `/workspaces/rover/timing_csv_graphs.py` - Updated with new parsing and analysis functions

## Related Files
- `/workspaces/rover/TIMING_MEASUREMENTS_ADDED.md` - Documents what timing was added to the node
- `/workspaces/rover/src/third-party/elevation_mapping_cupy/elevation_mapping_cupy/scripts/elevation_mapping_node.py` - The node with timing instrumentation
- `/tmp/elevation_mapping_timing.csv` - Output CSV file with timing data
