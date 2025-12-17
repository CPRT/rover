# Quick Reference: timing_csv_graphs.py

## Common Use Cases

### 1. **Identify the Bottleneck** ⚡
```bash
python timing_csv_graphs.py --analyze-publish
```
This will tell you:
- Which publishers are slow (>1s)
- Where time is spent (GPU transfer, serialization, etc.)
- Optimization suggestions

Expected output if publish_map is the bottleneck:
```
⚠️  WARNING: This publisher is SLOW (>1s median latency)
    This is likely the main bottleneck!

💡 Optimization suggestions:
    • Layer processing takes 1400ms - consider:
      - Reduce number of published layers
      - Publish different layers at different rates
      - Use MultiThreadedExecutor to avoid blocking
```

### 2. **Get Overall Statistics** 📊
```bash
python timing_csv_graphs.py --stats-only
```
Shows mean, median, min, max, std dev for all callbacks.

### 3. **Check Pointcloud Processing** 🎯
```bash
python timing_csv_graphs.py --filter pointcloud --breakdown
```
See detailed breakdown of pointcloud processing stages.

### 4. **Investigate Occasional Spikes** 🔍
```bash
python timing_csv_graphs.py --analyze-outliers --outlier-threshold 100
```
Identifies which callbacks have timing spikes and when they occur.

### 5. **Compare All Callbacks** 📈
```bash
python timing_csv_graphs.py --compare-all -o comparison.png
```
Creates time-series plots grouped by callback type.

### 6. **Full Diagnostic** 🔬
```bash
python timing_csv_graphs.py --analyze-publish --analyze-save --analyze-outliers --stats-only
```
Runs all analysis functions for comprehensive diagnostics.

---

## Understanding the Output

### Callback Types

| Callback Pattern | What It Measures | Expected Duration |
|-----------------|------------------|-------------------|
| `pointcloud_callback_*` | Pointcloud input processing | 30-50ms (good), >100ms (spikes OK) |
| `publish_map_*` | Map publishing to ROS topic | <100ms (good), >1000ms (BOTTLENECK) |
| `timer_pose_update` | Pose update timer | <1ms |
| `timer_update_variance` | Variance update | 5-10ms |
| `handle_save_map` | Map saving service | 1-3s (rare, OK) |

### Breakdown Stages

#### Pointcloud Processing:
- `numpify`: ROS message → numpy array (should be <5ms)
- `tf_lookup`: TF transform lookup (should be <2ms)
- `point_extraction`: Extract XYZ + channels (should be <5ms)
- `input_pointcloud`: CUDA processing (20-40ms, occasionally 100-200ms)

#### Map Publishing:
- `get_center`: Get map center (should be <1ms)
- `init_message`: Initialize ROS message (should be <5ms)
- `all_layers_total`: Process all layers (**THIS IS USUALLY THE BOTTLENECK**)
  - `layer_*_gpu_to_cpu`: GPU→CPU transfer (200-300ms per layer)
  - `layer_*_serialization`: Numpy→ROS serialization (150-250ms per layer)
- `publish_call`: Actual ROS publish (50-100ms)

---

## Interpreting Results

### ✅ GOOD Performance
```
pointcloud_callback_front: avg=42.50ms, median=40.20ms, max=189.45ms
publish_map_elevation_map_recordable: avg=150.23ms, median=145.10ms
```
- Pointcloud: <50ms average with occasional spikes
- Publishing: <200ms average

### ⚠️ BOTTLENECK Detected
```
publish_map_elevation_map_recordable: avg=1523.40ms, median=1500.23ms
  all_layers_total: avg=1400.23ms
```
- Publishing: >1000ms average
- `all_layers_total` dominates the time
- **This blocks the entire node** (SingleThreadedExecutor)

### 🔧 Fix Priority
1. **Highest**: publish_map >1s → Switch to MultiThreadedExecutor OR reduce publish rate
2. **Medium**: pointcloud processing >100ms consistently → Check CUDA memory
3. **Low**: Occasional spikes (<5% of calls) → Expected, safe to ignore

---

## Command-Line Arguments

### Basic Options
```
--csv PATH              CSV file path (default: /tmp/elevation_mapping_timing.csv)
--output FILE, -o FILE  Save figure to file
--filter PATTERN, -f    Filter callbacks containing pattern
--stats-only, -s        Print statistics only (no plots)
```

### Plot Types
```
--breakdown, -b         Include detailed breakdown in plot
--comparison, -c        Callback breakdown comparison
--histogram             Distribution histograms
--compare-all           Compare all callback types
```

### Analysis Functions
```
--analyze-publish       Analyze publish_map (MOST IMPORTANT)
--analyze-save          Analyze handle_save_map
--analyze-outliers      Detailed outlier analysis
--outlier-threshold MS  Outlier threshold (default: 100ms)
```

---

## Troubleshooting

### "No data found"
- Check that the node has been run: `ros2 run elevation_mapping_cupy elevation_mapping_node`
- Verify CSV exists: `ls -lh /tmp/elevation_mapping_timing.csv`
- Check CSV has data: `wc -l /tmp/elevation_mapping_timing.csv`

### "No publish_map callbacks found"
- The publishers might not have been triggered yet
- Wait for the node to publish at least once
- Check publisher configuration in YAML file

### Import errors
- Make sure matplotlib is installed: `pip install matplotlib pandas`

---

## Example Workflow

### Step 1: Collect data
```bash
# Terminal 1: Run the node
ros2 run elevation_mapping_cupy elevation_mapping_node

# Terminal 2: Send some pointcloud data
ros2 bag play your_data.bag
```

### Step 2: Quick check
```bash
# Check if data is being collected
tail -f /tmp/elevation_mapping_timing.csv

# Get basic stats
python timing_csv_graphs.py --stats-only
```

### Step 3: Identify bottleneck
```bash
# This is the KEY command - shows if publish_map is slow
python timing_csv_graphs.py --analyze-publish
```

### Step 4: Deep dive
```bash
# If publish_map is slow, check layer breakdown
python timing_csv_graphs.py --filter publish --breakdown -o publish_breakdown.png

# Check for other issues
python timing_csv_graphs.py --analyze-outliers

# Generate full report
python timing_csv_graphs.py --analyze-publish --analyze-save --compare-all -o full_analysis.png
```

---

## Files

| File | Purpose |
|------|---------|
| `/tmp/elevation_mapping_timing.csv` | Output data from node |
| `timing_csv_graphs.py` | Analysis script |
| `TIMING_MEASUREMENTS_ADDED.md` | Documents timing instrumentation |
| `TIMING_CSV_GRAPHS_UPDATES.md` | Documents script updates |
| `test_timing_graphs.py` | Test script with sample data |
