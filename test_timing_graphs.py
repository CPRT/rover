#!/usr/bin/env python3
"""
Quick test to verify timing_csv_graphs.py can parse the new callback types.
Creates sample timing data and runs the analysis functions.
"""

import pandas as pd
import numpy as np
from pathlib import Path
import sys

def create_sample_timing_data(output_path='/tmp/test_elevation_mapping_timing.csv'):
    """Create sample timing data with all the new callback types."""
    
    np.random.seed(42)
    
    # Simulate 100 iterations
    n_iterations = 100
    base_time = 1700000000.0  # Unix timestamp
    
    data = []
    
    for i in range(n_iterations):
        counter = i + 1
        timestamp = base_time + i * 1.0  # 1 second intervals
        
        # Pointcloud callback (existing)
        pc_total = 42.0 + np.random.normal(0, 3)
        data.append([timestamp, 'pointcloud_callback_front', pc_total, counter])
        data.append([timestamp, 'pointcloud_front_numpify', 2.5 + np.random.normal(0, 0.5), counter])
        data.append([timestamp, 'pointcloud_front_tf_lookup', 1.2 + np.random.normal(0, 0.2), counter])
        data.append([timestamp, 'pointcloud_front_input_pointcloud', 35.0 + np.random.normal(0, 2), counter])
        
        # Publish map callback (NEW - the suspected bottleneck)
        if i % 10 == 0:  # Publish every 10 iterations
            publish_total = 1500.0 + np.random.normal(0, 100)
            key = 'elevation_map_recordable'
            data.append([timestamp, f'publish_map_{key}', publish_total, counter])
            data.append([timestamp, f'publish_{key}_get_center', 0.12 + np.random.normal(0, 0.02), counter])
            data.append([timestamp, f'publish_{key}_init_message', 2.3 + np.random.normal(0, 0.3), counter])
            data.append([timestamp, f'publish_{key}_all_layers_total', 1400.0 + np.random.normal(0, 80), counter])
            data.append([timestamp, f'publish_{key}_publish_call', 95.0 + np.random.normal(0, 10), counter])
            
            # Layer-level breakdown
            for layer_name in ['elevation', 'variance', 'traversability']:
                data.append([timestamp, f'publish_{key}_layer_{layer_name}_gpu_to_cpu', 
                           250.0 + np.random.normal(0, 20), counter])
                data.append([timestamp, f'publish_{key}_layer_{layer_name}_serialization', 
                           200.0 + np.random.normal(0, 15), counter])
                data.append([timestamp, f'publish_{key}_layer_{layer_name}_total', 
                           450.0 + np.random.normal(0, 30), counter])
        
        # Timer callbacks (NEW)
        data.append([timestamp, 'timer_pose_update', 0.5 + np.random.normal(0, 0.1), counter])
        data.append([timestamp, 'timer_update_variance', 5.0 + np.random.normal(0, 0.5), counter])
        
        # Save map service (NEW - occasional)
        if i == 50:  # One save operation
            data.append([timestamp, 'handle_save_map', 2500.0 + np.random.normal(0, 200), counter])
            data.append([timestamp, 'save_map_prepare_paths', 1.0, counter])
            data.append([timestamp, 'save_map_collect_layers', 0.5, counter])
            data.append([timestamp, 'save_map_export_fused', 800.0, counter])
            data.append([timestamp, 'save_map_export_raw', 600.0, counter])
            data.append([timestamp, 'save_map_build_fused_msg', 400.0, counter])
            data.append([timestamp, 'save_map_build_raw_msg', 300.0, counter])
            data.append([timestamp, 'save_map_write_fused_bag', 200.0, counter])
            data.append([timestamp, 'save_map_write_raw_bag', 150.0, counter])
    
    # Create DataFrame
    df = pd.DataFrame(data, columns=['timestamp', 'callback_name', 'duration_ms', 'counter'])
    
    # Save to CSV
    df.to_csv(output_path, index=False)
    print(f"✅ Created sample timing data: {output_path}")
    print(f"   Total records: {len(df)}")
    print(f"   Unique callbacks: {df['callback_name'].nunique()}")
    print(f"   Callback types: {sorted(df['callback_name'].unique())}")
    
    return output_path


def test_script(csv_path):
    """Test the timing_csv_graphs.py script with sample data."""
    import sys
    import importlib.util
    
    # Load the script as a module
    script_path = Path('/workspaces/rover/timing_csv_graphs.py')
    spec = importlib.util.spec_from_file_location("timing_csv_graphs", script_path)
    module = importlib.util.module_from_spec(spec)
    
    print("\n" + "="*80)
    print("TESTING timing_csv_graphs.py")
    print("="*80)
    
    try:
        spec.loader.exec_module(module)
        
        print("\n✅ Module loaded successfully")
        
        # Test statistics function
        print("\n📊 Testing print_statistics()...")
        module.print_statistics(csv_path)
        
        # Test publish_map analysis
        print("\n📊 Testing analyze_publish_map()...")
        module.analyze_publish_map(csv_path)
        
        # Test save_map analysis
        print("\n📊 Testing analyze_save_map()...")
        module.analyze_save_map(csv_path)
        
        print("\n" + "="*80)
        print("✅ ALL TESTS PASSED")
        print("="*80)
        print("\nThe script can now parse all new callback types!")
        print("Try running:")
        print(f"  python timing_csv_graphs.py --csv {csv_path} --analyze-publish")
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


if __name__ == '__main__':
    # Create sample data
    csv_path = create_sample_timing_data()
    
    # Test the script
    success = test_script(csv_path)
    
    sys.exit(0 if success else 1)
