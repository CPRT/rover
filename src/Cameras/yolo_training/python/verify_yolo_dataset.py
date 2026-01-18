import argparse
import yaml
import os
from pathlib import Path
from collections import Counter, defaultdict

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def scan_labels(dataset_dir, split_name, class_names):
    """
    Scans a specific split directory (e.g., train/labels) and counts class occurrences.
    """
    labels_path = Path(dataset_dir) / split_name / 'labels'
    if not labels_path.exists():
        return None, []

    class_counts = Counter()
    errors = []
    
    # Iterate over all .txt files
    files = list(labels_path.glob('*.txt'))
    
    for file in files:
        try:
            with open(file, 'r') as f:
                lines = f.readlines()
                
            for line_num, line in enumerate(lines):
                parts = line.strip().split()
                if not parts:
                    continue
                
                # YOLO format: class_id x_center y_center width height
                try:
                    class_id = int(parts[0])
                    
                    # Check for Out of Bounds
                    if class_id < 0 or class_id >= len(class_names):
                        errors.append(f"{file.name}: Line {line_num+1} has invalid Class ID '{class_id}' (Max allowed: {len(class_names)-1})")
                    else:
                        class_counts[class_id] += 1
                        
                except ValueError:
                    errors.append(f"{file.name}: Line {line_num+1} is malformed: '{line.strip()}'")
                    
        except Exception as e:
            errors.append(f"Could not read file {file.name}: {e}")
            
    return class_counts, errors

def main():
    parser = argparse.ArgumentParser(description="Verify integrity of a YOLO dataset.")
    parser.add_argument('--dir', type=str, required=True, help="Path to the generated dataset directory")
    args = parser.parse_args()

    dataset_dir = Path(args.dir)
    data_yaml_path = dataset_dir / 'data.yaml'

    if not data_yaml_path.exists():
        print(f"❌ Error: data.yaml not found at {data_yaml_path}")
        return

    # 1. Load Configuration
    config = load_yaml(data_yaml_path)
    class_names = config.get('names', [])
    
    if isinstance(class_names, dict):
        # Handle case where names is a dict {0: 'person', ...}
        class_names = [class_names[i] for i in sorted(class_names.keys())]

    print(f"\n🔎 Scanning Dataset: {dataset_dir}")
    print(f"ℹ️  Defined Classes ({len(class_names)}): {class_names}")
    print("-" * 60)

    # 2. Scan Splits
    total_counts = Counter()
    
    for split in ['train', 'valid', 'test']:
        print(f"\n📂 Checking split: {split.upper()} ...")
        counts, errors = scan_labels(dataset_dir, split, class_names)
        
        if counts is None:
            print(f"   ⚠️  Skipping (Directory not found)")
            continue

        # Print Errors immediately
        if errors:
            print(f"   ❌ Found {len(errors)} critical errors:")
            for e in errors[:5]: # Show first 5
                print(f"      - {e}")
            if len(errors) > 5: print(f"      ... and {len(errors)-5} more.")
        else:
            print("   ✅ Format check passed.")

        # Print Distribution for this split
        print(f"   📊 Class Distribution:")
        if not counts:
            print("      (No labels found)")
        
        sorted_ids = sorted(counts.keys())
        for cid in sorted_ids:
            name = class_names[cid] if cid < len(class_names) else "UNKNOWN"
            print(f"      - ID {cid} ({name}): {counts[cid]} instances")
            total_counts[cid] += counts[cid]

    # 3. Final Summary
    print("\n" + "="*30)
    print("       FINAL SUMMARY       ")
    print("="*30)
    
    print(f"{'ID':<5} {'Name':<20} {'Total Count':<10}")
    print("-" * 35)
    for i, name in enumerate(class_names):
        count = total_counts[i]
        print(f"{i:<5} {name:<20} {count:<10}")
    
    # Check for empty classes
    empty_classes = [name for i, name in enumerate(class_names) if total_counts[i] == 0]
    if empty_classes:
        print("\n⚠️  WARNING: The following classes have 0 instances across all splits:")
        print(f"   {empty_classes}")
        print("   (This might indicate a mapping error in your generation script)")
    else:
        print("\n✅ All classes are represented.")

if __name__ == "__main__":
    main()