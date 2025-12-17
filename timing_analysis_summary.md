# Elevation Mapping Performance Analysis Summary

## Key Findings

### Overall Performance
- **Median Performance**: 42.5ms (✅ Excellent - supports 10Hz goal)
- **95th Percentile**: 77.1ms (✅ Good - still under 100ms)
- **99th Percentile**: 116.5ms (⚠️ Acceptable but close to limit)
- **Outliers**: 4 calls out of 197 (2.0%) exceeded 100ms

### Outlier Analysis

The **900+ms spike** only happens **ONCE** - on the very first pointcloud:
- **Call #1**: 977.7ms (initialization)
- **Call #67**: 269.2ms  
- **Call #192**: 110.2ms
- **Call #195**: 106.7ms

### Breakdown of Time (Median)
```
Total pointcloud_callback:     42.5ms  (100%)
├── input_pointcloud:          39.0ms  (92%)   ← Main bottleneck
├── point_extraction:           2.2ms  (5%)
├── tf_lookup:                  0.5ms  (1%)
├── transform_extract:          0.2ms  (0.5%)
├── numpify:                    0.1ms  (0.2%)
└── validation:                 0.005ms (negligible)
```

### Timer Callbacks
- **pose_update**: 11.0ms median (some spikes to 200ms)
- **update_variance**: 0.4ms median
- **update_time**: 0.2ms median

## Conclusions

### ✅ Good News
1. **Median performance is excellent** - 42.5ms leaves plenty of margin for 10Hz (100ms budget)
2. **The 977ms spike is ONLY initialization** - happens once at startup
3. **Python overhead is minimal** - only ~3-4ms total (numpify + validation + point_extraction)
4. **92% of time is in `input_pointcloud()`** - the CUDA/C++ layer is doing the work

### ⚠️ Areas of Concern
1. **Occasional spikes at calls #67, #192, #195** (110-270ms)
   - Gap pattern: 66 calls, then 125 calls, then 3 calls
   - Might correlate with:
     - Map reallocation/expansion
     - Garbage collection in CUDA memory
     - Large pose jumps triggering map shifts
     
2. **Timer callbacks can spike** - pose_update can take up to 200ms
   - This could interfere with meeting 10Hz if it coincides with pointcloud processing

## Recommendations

### Immediate Actions
1. **Ignore the first call spike** - it's initialization, unavoidable
2. **Monitor calls #67, #192, etc.** - Add logging to identify what triggers these:
   ```python
   if duration > 100:
       self.get_logger().warn(f"Slow pointcloud processing: {duration:.1f}ms, "
                             f"points={len(pts)}, map_moved={map_moved}")
   ```

### Investigation Priorities
1. **Investigate `input_pointcloud()` internals**:
   - Is it doing memory allocation?
   - CUDA kernel launch overhead?
   - Map shifting/expansion?

2. **Check for patterns**:
   - Does it correlate with point cloud size?
   - Does it happen after large robot movements?
   - Does it happen at regular intervals (suggesting GC/memory management)?

### Optimization Strategy
Since Python is only 5-8% of the cost, **focus on the C++/CUDA layer**:
1. Profile `input_pointcloud()` with CUDA profiler
2. Check if map expansion can be made async
3. Consider pre-allocating CUDA memory to avoid spikes

## Files Generated
- `elevation_mapping_timing.csv` - Raw timing data
- `histogram.png` - Distribution visualization
- `timing_csv_graphs.py` - Analysis script with options:
  - `--analyze-outliers` - Detailed outlier analysis
  - `--histogram` - Distribution plots
  - `--comparison` - Breakdown comparison
  - `--breakdown` - Include sub-timing in plots

## Next Steps
1. Run with `--analyze-outliers --outlier-threshold 50` to catch more edge cases
2. Add point cloud size logging to correlate with timing
3. Check if calls #67, #192, #195 correlate with specific map operations
