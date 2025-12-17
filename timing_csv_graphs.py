#!/usr/bin/env python3
"""
Script to parse and visualize elevation mapping timing data.
Displays timing measurements over time with different colors for each callback.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path


def parse_and_plot(csv_path, output_path=None, show_breakdown=False, filter_pattern=None):
    """
    Parse timing CSV and create visualization.
    
    Args:
        csv_path: Path to the CSV file
        output_path: Optional path to save the figure
        show_breakdown: If True, include detailed breakdown measurements
        filter_pattern: Optional string to filter callback names
    """
    # Read CSV
    df = pd.read_csv(csv_path)
    
    # Calculate relative time (seconds since first measurement)
    df['relative_time'] = df['timestamp'] - df['timestamp'].min()
    
    # Filter callbacks if pattern provided
    if filter_pattern:
        df = df[df['callback_name'].str.contains(filter_pattern, case=False)]
    
    # Separate main callbacks from breakdown measurements
    if not show_breakdown:
        # Only show top-level callbacks (not the detailed breakdown)
        breakdown_patterns = [
            '_numpify', '_validation', '_tf_lookup', 
            '_transform_extract', '_point_extraction', '_input_pointcloud',
            '_get_center', '_init_message', '_all_layers_total', '_publish_call',
            '_layer_.*_(gpu_to_cpu|serialization|total)',  # publish map layer details
            'save_map_(prepare_paths|collect_layers|export_fused|export_raw|build_fused_msg|build_raw_msg|write_fused_bag|write_raw_bag)',
            'build_msg_(init|all_layers|layer_.*)',
            '_numpy_to_multiarray'
        ]
        mask = ~df['callback_name'].str.contains('|'.join(breakdown_patterns), regex=True)
        df = df[mask]
    
    # Get unique callback names
    callback_names = df['callback_name'].unique()
    
    # Create figure
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Plot each callback with a different color
    colors = plt.cm.tab20(range(len(callback_names)))
    
    for idx, callback_name in enumerate(callback_names):
        callback_df = df[df['callback_name'] == callback_name]
        # Convert to numpy arrays to avoid pandas indexing issues
        x_data = callback_df['relative_time'].values
        y_data = callback_df['duration_ms'].values
        ax.scatter(x_data, y_data, 
                  label=callback_name, alpha=0.6, s=20, color=colors[idx])
        # Also plot a line to show trends
        ax.plot(x_data, y_data, 
               alpha=0.3, linewidth=1, color=colors[idx])
    
    # Formatting
    ax.set_xlabel('Time since start (seconds)', fontsize=12)
    ax.set_ylabel('Duration (ms)', fontsize=12)
    ax.set_title('Elevation Mapping Callback Timing Performance', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
    
    # Tight layout to prevent label cutoff
    plt.tight_layout()
    
    # Save or show
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {output_path}")
    else:
        plt.show()


def print_statistics(csv_path, filter_pattern=None):
    """Print summary statistics for each callback."""
    df = pd.read_csv(csv_path)
    
    if filter_pattern:
        df = df[df['callback_name'].str.contains(filter_pattern, case=False)]
    
    print("\n" + "="*80)
    print("TIMING STATISTICS SUMMARY")
    print("="*80)
    
    for callback_name in sorted(df['callback_name'].unique()):
        callback_df = df[df['callback_name'] == callback_name]
        durations = callback_df['duration_ms']
        
        # Calculate outliers (values > 95th percentile or > 2*median)
        p95 = durations.quantile(0.95)
        median = durations.median()
        outlier_threshold = max(p95, 2 * median)
        outliers = durations[durations > outlier_threshold]
        
        print(f"\n{callback_name}:")
        print(f"  Count:   {len(durations)}")
        print(f"  Mean:    {durations.mean():.3f} ms")
        print(f"  Median:  {durations.median():.3f} ms")
        print(f"  Std Dev: {durations.std():.3f} ms")
        print(f"  Min:     {durations.min():.3f} ms")
        print(f"  Max:     {durations.max():.3f} ms")
        print(f"  95th %%:  {durations.quantile(0.95):.3f} ms")
        print(f"  99th %%:  {durations.quantile(0.99):.3f} ms")
        
        if len(outliers) > 0:
            outlier_pct = (len(outliers) / len(durations)) * 100
            print(f"  Outliers: {len(outliers)} ({outlier_pct:.1f}%) over {outlier_threshold:.1f} ms")
            
            # Show when outliers occur (first few)
            outlier_indices = durations[durations > outlier_threshold].index
            counters = [callback_df.loc[idx, 'counter'] for idx in outlier_indices[:5]]
            print(f"  Outlier occurrences (first 5): {counters}")
    
    print("\n" + "="*80)


def create_histogram_plot(csv_path, output_path=None, filter_pattern=None):
    """Create histogram showing distribution of callback durations."""
    df = pd.read_csv(csv_path)
    
    if filter_pattern:
        df = df[df['callback_name'].str.contains(filter_pattern, case=False)]
    
    # Focus on main callbacks
    breakdown_patterns = [
        '_numpify', '_validation', '_tf_lookup', 
        '_transform_extract', '_point_extraction', '_input_pointcloud',
        '_get_center', '_init_message', '_all_layers_total', '_publish_call',
        '_layer_.*_(gpu_to_cpu|serialization|total)',
        'save_map_(prepare_paths|collect_layers|export_fused|export_raw|build_fused_msg|build_raw_msg|write_fused_bag|write_raw_bag)',
        'build_msg_(init|all_layers|layer_.*)',
        '_numpy_to_multiarray'
    ]
    mask = ~df['callback_name'].str.contains('|'.join(breakdown_patterns), regex=True)
    df = df[mask]
    
    callback_names = df['callback_name'].unique()
    n_callbacks = len(callback_names)
    
    fig, axes = plt.subplots(n_callbacks, 1, figsize=(12, 4*n_callbacks), squeeze=False)
    
    for idx, callback_name in enumerate(callback_names):
        ax = axes[idx, 0]
        callback_df = df[df['callback_name'] == callback_name]
        durations = callback_df['duration_ms'].values
        
        # Create histogram
        ax.hist(durations, bins=50, alpha=0.7, edgecolor='black')
        
        # Add vertical lines for statistics
        median = np.median(durations)
        p95 = np.percentile(durations, 95)
        p99 = np.percentile(durations, 99)
        
        ax.axvline(median, color='green', linestyle='--', linewidth=2, label=f'Median: {median:.1f}ms')
        ax.axvline(p95, color='orange', linestyle='--', linewidth=2, label=f'95th: {p95:.1f}ms')
        ax.axvline(p99, color='red', linestyle='--', linewidth=2, label=f'99th: {p99:.1f}ms')
        
        ax.set_xlabel('Duration (ms)', fontsize=10)
        ax.set_ylabel('Frequency', fontsize=10)
        ax.set_title(f'Distribution: {callback_name}', fontsize=12, fontweight='bold')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Histogram saved to: {output_path}")
    else:
        plt.show()


def create_comparison_plot(csv_path, output_path=None):
    """Create a comparison plot showing main callbacks vs their breakdowns."""
    df = pd.read_csv(csv_path)
    df['relative_time'] = df['timestamp'] - df['timestamp'].min()
    
    # Find pointcloud callbacks
    pointcloud_callbacks = [name for name in df['callback_name'].unique() 
                           if name.startswith('pointcloud_callback_')]
    
    if not pointcloud_callbacks:
        print("No pointcloud callbacks found for comparison plot")
        return
    
    # Create subplots for each pointcloud callback
    n_callbacks = len(pointcloud_callbacks)
    fig, axes = plt.subplots(n_callbacks, 1, figsize=(14, 5*n_callbacks), squeeze=False)
    
    for idx, main_callback in enumerate(pointcloud_callbacks):
        ax = axes[idx, 0]
        
        # Get the prefix for breakdown measurements
        prefix = main_callback.replace('pointcloud_callback_', 'pointcloud_')
        
        # Find all related measurements
        related = df[df['callback_name'].str.startswith(prefix)]
        
        # Plot each component
        for callback_name in related['callback_name'].unique():
            callback_df = related[related['callback_name'] == callback_name]
            # Convert to numpy arrays to avoid pandas indexing issues
            x_data = callback_df['relative_time'].values
            y_data = callback_df['duration_ms'].values
            ax.plot(x_data, y_data, 
                   label=callback_name.replace(prefix, ''), alpha=0.7, linewidth=1.5)
        
        ax.set_xlabel('Time since start (seconds)', fontsize=10)
        ax.set_ylabel('Duration (ms)', fontsize=10)
        ax.set_title(f'Breakdown: {main_callback}', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Comparison figure saved to: {output_path}")
    else:
        plt.show()


def analyze_outliers(csv_path, callback_pattern='pointcloud_callback', threshold_ms=100):
    """Detailed analysis of timing outliers."""
    df = pd.read_csv(csv_path)
    df['relative_time'] = df['timestamp'] - df['timestamp'].min()
    
    # Filter for the callback of interest
    callback_df = df[df['callback_name'].str.contains(callback_pattern, case=False)]
    
    # Find main callback
    breakdown_patterns = [
        '_numpify', '_validation', '_tf_lookup', 
        '_transform_extract', '_point_extraction', '_input_pointcloud',
        '_get_center', '_init_message', '_all_layers_total', '_publish_call',
        '_layer_.*_(gpu_to_cpu|serialization|total)',
        'save_map_(prepare_paths|collect_layers|export_fused|export_raw|build_fused_msg|build_raw_msg|write_fused_bag|write_raw_bag)',
        'build_msg_(init|all_layers|layer_.*)',
        '_numpy_to_multiarray'
    ]
    
    main_callbacks = [name for name in callback_df['callback_name'].unique() 
                     if not any(df['callback_name'].str.match(f'.*{pattern}').any() 
                               for pattern in breakdown_patterns)]
    
    if not main_callbacks:
        print("No main callback found matching pattern")
        return
    
    print("\n" + "="*80)
    print(f"OUTLIER ANALYSIS (Threshold: {threshold_ms}ms)")
    print("="*80)
    
    for main_callback in main_callbacks:
        main_df = df[df['callback_name'] == main_callback].copy()
        outliers = main_df[main_df['duration_ms'] > threshold_ms]
        
        if len(outliers) == 0:
            print(f"\nNo outliers found for {main_callback}")
            continue
        
        print(f"\n{main_callback}:")
        print(f"  Total calls: {len(main_df)}")
        print(f"  Outliers: {len(outliers)} ({100*len(outliers)/len(main_df):.1f}%)")
        
        # Analyze outlier pattern
        outlier_counters = outliers['counter'].values
        print(f"  Outlier call numbers: {outlier_counters.tolist()}")
        
        # Check if first call
        if 1 in outlier_counters:
            print("  ⚠️  First call is an outlier (likely initialization)")
        
        # Check spacing between outliers
        if len(outlier_counters) > 1:
            gaps = np.diff(outlier_counters)
            print(f"  Gaps between outliers: {gaps.tolist()}")
            if len(gaps) > 0:
                print(f"  Average gap: {np.mean(gaps):.1f} calls")
        
        # Show detailed breakdown for outliers
        prefix = main_callback.replace('pointcloud_callback_', 'pointcloud_')
        print(f"\n  Detailed breakdown for outliers:")
        print(f"  {'Counter':<10} {'Total':<10} {'numpify':<10} {'tf_lookup':<12} {'point_ext':<12} {'input_pc':<12}")
        
        for counter in outlier_counters[:10]:  # Show first 10
            row = main_df[main_df['counter'] == counter].iloc[0]
            total_time = row['duration_ms']
            
            # Find breakdown times
            breakdown = {}
            for suffix in ['numpify', 'tf_lookup', 'point_extraction', 'input_pointcloud']:
                breakdown_name = f"{prefix}{suffix}"
                breakdown_row = df[(df['callback_name'] == breakdown_name) & 
                                  (df['counter'] == counter)]
                if not breakdown_row.empty:
                    breakdown[suffix] = breakdown_row.iloc[0]['duration_ms']
                else:
                    breakdown[suffix] = 0.0
            
            print(f"  {int(counter):<10} {total_time:<10.1f} {breakdown.get('numpify', 0):<10.2f} "
                  f"{breakdown.get('tf_lookup', 0):<12.2f} {breakdown.get('point_extraction', 0):<12.2f} "
                  f"{breakdown.get('input_pointcloud', 0):<12.1f}")
    
    print("\n" + "="*80)


def analyze_publish_map(csv_path, output_path=None):
    """Analyze publish_map performance with detailed breakdown."""
    df = pd.read_csv(csv_path)
    df['relative_time'] = df['timestamp'] - df['timestamp'].min()
    
    # Find all publish_map callbacks
    publish_map_callbacks = [name for name in df['callback_name'].unique() 
                            if name.startswith('publish_map_')]
    
    if not publish_map_callbacks:
        print("\n⚠️  No publish_map callbacks found in timing data")
        return
    
    print("\n" + "="*80)
    print("PUBLISH_MAP DETAILED ANALYSIS")
    print("="*80)
    
    for main_callback in publish_map_callbacks:
        key = main_callback.replace('publish_map_', '')
        print(f"\n📊 Publisher: {key}")
        
        main_df = df[df['callback_name'] == main_callback]
        if main_df.empty:
            continue
            
        durations = main_df['duration_ms']
        print(f"  Total calls: {len(durations)}")
        print(f"  Mean:   {durations.mean():.3f} ms ({durations.mean()/1000:.3f} s)")
        print(f"  Median: {durations.median():.3f} ms ({durations.median()/1000:.3f} s)")
        print(f"  Min:    {durations.min():.3f} ms")
        print(f"  Max:    {durations.max():.3f} ms ({durations.max()/1000:.3f} s)")
        
        # Analyze breakdown stages
        breakdown_stages = ['get_center', 'init_message', 'all_layers_total', 'publish_call']
        print(f"\n  Breakdown by stage:")
        
        stage_stats = {}
        for stage in breakdown_stages:
            stage_name = f'publish_{key}_{stage}'
            stage_df = df[df['callback_name'] == stage_name]
            if not stage_df.empty:
                stage_durations = stage_df['duration_ms']
                stage_stats[stage] = {
                    'mean': stage_durations.mean(),
                    'median': stage_durations.median(),
                    'max': stage_durations.max()
                }
                print(f"    {stage:20s}: avg={stage_durations.mean():8.3f} ms, "
                      f"median={stage_durations.median():8.3f} ms, "
                      f"max={stage_durations.max():8.3f} ms")
        
        # Analyze layer-level breakdown
        layer_patterns = [
            (f'publish_{key}_layer_.*_gpu_to_cpu', 'GPU→CPU Transfer'),
            (f'publish_{key}_layer_.*_serialization', 'Serialization'),
            (f'publish_{key}_layer_.*_total', 'Layer Total')
        ]
        
        print(f"\n  Per-layer timing summary:")
        for pattern, label in layer_patterns:
            layer_df = df[df['callback_name'].str.match(pattern)]
            if not layer_df.empty:
                # Group by layer name
                layer_names = layer_df['callback_name'].str.extract(f'publish_{key}_layer_(.*)_(gpu_to_cpu|serialization|total)')[0].unique()
                
                for layer_name in layer_names:
                    layer_specific = df[df['callback_name'] == f'publish_{key}_layer_{layer_name}_{pattern.split("_")[-1]}']
                    if not layer_specific.empty:
                        durations = layer_specific['duration_ms']
                        print(f"    {layer_name:20s} {label:20s}: avg={durations.mean():8.3f} ms")
        
        # Calculate publish rate
        if len(main_df) > 1:
            time_span = main_df['relative_time'].max() - main_df['relative_time'].min()
            publish_rate = (len(main_df) - 1) / time_span if time_span > 0 else 0
            print(f"\n  Publishing rate: {publish_rate:.2f} Hz ({1000/publish_rate:.1f} ms interval)")
        
        # Check if this is the bottleneck
        if durations.median() > 1000:
            print(f"\n  ⚠️  WARNING: This publisher is SLOW (>{1}s median latency)")
            print(f"      This is likely the main bottleneck!")
            
            # Suggest fixes
            print(f"\n  💡 Optimization suggestions:")
            if 'all_layers_total' in stage_stats and stage_stats['all_layers_total']['mean'] > 500:
                print(f"      • Layer processing takes {stage_stats['all_layers_total']['mean']:.0f}ms - consider:")
                print(f"        - Reduce number of published layers")
                print(f"        - Publish different layers at different rates")
                print(f"        - Use MultiThreadedExecutor to avoid blocking")
    
    print("\n" + "="*80)
    
    # Create visualization if requested
    if output_path and publish_map_callbacks:
        fig, axes = plt.subplots(len(publish_map_callbacks), 1, 
                                figsize=(14, 5*len(publish_map_callbacks)), squeeze=False)
        
        for idx, main_callback in enumerate(publish_map_callbacks):
            ax = axes[idx, 0]
            key = main_callback.replace('publish_map_', '')
            
            # Get breakdown stages
            breakdown_stages = ['get_center', 'init_message', 'all_layers_total', 'publish_call']
            
            # Plot stacked area chart
            stage_data = {}
            for stage in breakdown_stages:
                stage_name = f'publish_{key}_{stage}'
                stage_df = df[df['callback_name'] == stage_name]
                if not stage_df.empty:
                    # Align by counter
                    stage_data[stage] = stage_df.set_index('counter')['duration_ms']
            
            if stage_data:
                # Create DataFrame from all stages
                combined = pd.DataFrame(stage_data)
                combined = combined.fillna(0)
                
                # Plot stacked area
                combined.plot.area(ax=ax, alpha=0.7)
                
                ax.set_xlabel('Call number', fontsize=10)
                ax.set_ylabel('Duration (ms)', fontsize=10)
                ax.set_title(f'publish_map_{key} - Breakdown by Stage', 
                           fontsize=12, fontweight='bold')
                ax.legend(loc='upper left', fontsize=9)
                ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✅ Publish map analysis figure saved to: {output_path}")


def analyze_save_map(csv_path):
    """Analyze handle_save_map performance with detailed breakdown."""
    df = pd.read_csv(csv_path)
    
    # Find save_map calls
    save_map_df = df[df['callback_name'] == 'handle_save_map']
    
    if save_map_df.empty:
        print("\n⚠️  No handle_save_map calls found in timing data")
        return
    
    print("\n" + "="*80)
    print("HANDLE_SAVE_MAP DETAILED ANALYSIS")
    print("="*80)
    
    durations = save_map_df['duration_ms']
    print(f"\nTotal save operations: {len(durations)}")
    print(f"Mean:   {durations.mean():.3f} ms ({durations.mean()/1000:.3f} s)")
    print(f"Median: {durations.median():.3f} ms ({durations.median()/1000:.3f} s)")
    print(f"Min:    {durations.min():.3f} ms")
    print(f"Max:    {durations.max():.3f} ms ({durations.max()/1000:.3f} s)")
    
    # Analyze breakdown stages
    save_stages = [
        'prepare_paths', 'collect_layers', 'export_fused', 'export_raw',
        'build_fused_msg', 'build_raw_msg', 'write_fused_bag', 'write_raw_bag'
    ]
    
    print(f"\nBreakdown by stage:")
    for stage in save_stages:
        stage_name = f'save_map_{stage}'
        stage_df = df[df['callback_name'] == stage_name]
        if not stage_df.empty:
            stage_durations = stage_df['duration_ms']
            print(f"  {stage:20s}: avg={stage_durations.mean():8.3f} ms, "
                  f"median={stage_durations.median():8.3f} ms, "
                  f"max={stage_durations.max():8.3f} ms")
    
    print("\n" + "="*80)


def create_publisher_comparison(csv_path, output_path=None):
    """Create a comparison plot of all publishers and their rates."""
    df = pd.read_csv(csv_path)
    df['relative_time'] = df['timestamp'] - df['timestamp'].min()
    
    # Find all main callback types
    callback_categories = {
        'Pointcloud Input': [name for name in df['callback_name'].unique() 
                            if name.startswith('pointcloud_callback_')],
        'Image Input': [name for name in df['callback_name'].unique() 
                       if name.startswith('image_callback_')],
        'Map Publishing': [name for name in df['callback_name'].unique() 
                          if name.startswith('publish_map_')],
        'Timers': [name for name in df['callback_name'].unique() 
                  if name.startswith('timer_')],
        'Services': ['handle_save_map', 'handle_load_map', '_build_grid_map_message']
    }
    
    # Filter to only existing callbacks
    callback_categories = {cat: [name for name in names if name in df['callback_name'].unique()] 
                          for cat, names in callback_categories.items()}
    callback_categories = {cat: names for cat, names in callback_categories.items() if names}
    
    if not callback_categories:
        print("No callbacks found for comparison")
        return
    
    # Create figure with subplots
    n_cats = len(callback_categories)
    fig, axes = plt.subplots(n_cats, 1, figsize=(14, 4*n_cats), squeeze=False)
    
    for idx, (category, callback_names) in enumerate(callback_categories.items()):
        ax = axes[idx, 0]
        
        # Plot each callback in this category
        for callback_name in callback_names:
            callback_df = df[df['callback_name'] == callback_name]
            if not callback_df.empty:
                x_data = callback_df['relative_time'].values
                y_data = callback_df['duration_ms'].values
                ax.plot(x_data, y_data, label=callback_name, alpha=0.7, linewidth=1.5, marker='.')
        
        ax.set_xlabel('Time since start (seconds)', fontsize=10)
        ax.set_ylabel('Duration (ms)', fontsize=10)
        ax.set_title(f'{category} - Timing Comparison', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='best')
        
        # Add horizontal line at critical thresholds
        if category == 'Map Publishing':
            ax.axhline(y=1000, color='red', linestyle='--', alpha=0.5, label='1s threshold')
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✅ Publisher comparison saved to: {output_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize elevation mapping timing data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic plot
  python timing_csv_graphs.py
  
  # Include detailed breakdowns
  python timing_csv_graphs.py --breakdown
  
  # Filter for specific callbacks
  python timing_csv_graphs.py --filter pointcloud
  
  # Show statistics only
  python timing_csv_graphs.py --stats-only
  
  # Create comparison plot
  python timing_csv_graphs.py --comparison
  
  # Analyze publish_map bottleneck
  python timing_csv_graphs.py --analyze-publish
  
  # Analyze save_map performance
  python timing_csv_graphs.py --analyze-save
  
  # Compare all callback types
  python timing_csv_graphs.py --compare-all
  
  # Full analysis with all diagnostics
  python timing_csv_graphs.py --analyze-publish --analyze-save --analyze-outliers
  
  # Save to file
  python timing_csv_graphs.py --output timing_plot.png
        """
    )
    
    parser.add_argument(
        '--csv',
        type=str,
        default='/tmp/elevation_mapping_timing.csv',
        help='Path to timing CSV file (default: /tmp/elevation_mapping_timing.csv)'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        help='Output file path for saving the figure (e.g., plot.png)'
    )
    parser.add_argument(
        '--breakdown', '-b',
        action='store_true',
        help='Include detailed breakdown measurements in plot'
    )
    parser.add_argument(
        '--filter', '-f',
        type=str,
        help='Filter callback names containing this string'
    )
    parser.add_argument(
        '--stats-only', '-s',
        action='store_true',
        help='Print statistics only, no plots'
    )
    parser.add_argument(
        '--comparison', '-c',
        action='store_true',
        help='Create comparison plot showing callback breakdowns'
    )
    parser.add_argument(
        '--histogram',
        action='store_true',
        help='Create histogram showing distribution of callback durations'
    )
    parser.add_argument(
        '--analyze-outliers',
        action='store_true',
        help='Perform detailed outlier analysis'
    )
    parser.add_argument(
        '--outlier-threshold',
        type=float,
        default=100.0,
        help='Threshold in ms for outlier detection (default: 100.0)'
    )
    parser.add_argument(
        '--analyze-publish',
        action='store_true',
        help='Analyze publish_map performance with detailed breakdown'
    )
    parser.add_argument(
        '--analyze-save',
        action='store_true',
        help='Analyze handle_save_map performance'
    )
    parser.add_argument(
        '--compare-all',
        action='store_true',
        help='Create comparison plot of all callback types'
    )
    
    args = parser.parse_args()
    
    # Check if CSV exists
    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"Error: CSV file not found at {csv_path}")
        print("Make sure the elevation mapping node has been run and timing data was collected.")
        return
    
    print(f"Reading timing data from: {csv_path}")
    
    # Print statistics
    print_statistics(args.csv, args.filter)
    
    # Perform outlier analysis if requested
    if args.analyze_outliers:
        analyze_outliers(args.csv, threshold_ms=args.outlier_threshold)
    
    # Analyze publish_map if requested
    if args.analyze_publish:
        output = args.output if args.output else None
        analyze_publish_map(args.csv, output)
    
    # Analyze save_map if requested
    if args.analyze_save:
        analyze_save_map(args.csv)
    
    # Create comparison plot if requested
    if args.compare_all:
        output = args.output if args.output else None
        create_publisher_comparison(args.csv, output)
    
    # Create plots if not stats-only
    if not args.stats_only:
        if args.comparison:
            create_comparison_plot(args.csv, args.output)
        elif args.histogram:
            create_histogram_plot(args.csv, args.output, args.filter)
        else:
            parse_and_plot(args.csv, args.output, args.breakdown, args.filter)


if __name__ == '__main__':
    main()
