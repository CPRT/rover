import os
import argparse
import yaml
import shutil
from pathlib import Path
from typing import List, Tuple, Dict, Set
from sklearn.model_selection import train_test_split


def find_image_label_pairs(dataset_dir: str) -> List[Tuple[Path, Path]]:
    """
    Find all image-label pairs in a dataset directory.
    Checks for 'train', 'test', and both 'val'/'valid'.
    """
    pairs = []
    dataset_path = Path(dataset_dir)
    
    # Supported image extensions
    image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
    
    # Map standard splits to possible directory names
    # Priority: check 'valid' first (Roboflow default), then 'val' (YOLO default)
    split_variations = {
        'train': ['train'],
        'test': ['test'], 
        'val': ['valid', 'val'] # check both
    }
    
    for split_type, possible_dirs in split_variations.items():
        found_dir = False
        for dir_name in possible_dirs:
            images_dir = dataset_path / dir_name / 'images'
            labels_dir = dataset_path / dir_name / 'labels'
            
            # If we find the images dir, we assume this is the correct folder name for this split
            if images_dir.exists():
                found_dir = True
                if not labels_dir.exists():
                    print(f"Warning: Found images at {images_dir} but no labels dir.")
                    continue

                for image_file in images_dir.iterdir():
                    if image_file.suffix.lower() not in image_extensions:
                        continue
                    
                    # Find corresponding label file
                    label_file = labels_dir / (image_file.stem + '.txt')
                    if label_file.exists():
                        pairs.append((image_file.resolve(), label_file.resolve()))
                
                # Stop checking other variations for this split (e.g. if found 'valid', don't check 'val')
                break 
    
    return pairs


def parse_split(split_str: str) -> Tuple[float, float, float]:
    """Parse split string like '70_20_10' into (train, val, test) ratios."""
    parts = split_str.split('_')
    if len(parts) != 3:
        raise ValueError(f"Invalid split format: {split_str}. Expected format: XX_YY_ZZ")
    
    train, val, test = int(parts[0]), int(parts[1]), int(parts[2])
    if train + val + test != 100:
        raise ValueError(f"Split percentages must sum to 100, got {train + val + test}")
    
    return train / 100, val / 100, test / 100


def copy_file(src: Path, dst: Path):
    """Copy a file, creating parent directories if needed."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def generate_dataset(
    pairs: List[Tuple[Path, Path, int]],  # (image, label, dataset_idx)
    output_dir: Path,
    split_ratios: Tuple[float, float, float],
    class_names: List[str],
    class_mappings: Dict[int, Dict[int, int]]
):
    """Generate the new dataset by copying files with proper splits and remapping labels."""
    train_ratio, val_ratio, test_ratio = split_ratios
    
    # First split: train+val vs test
    if test_ratio > 0:
        train_val, test_data = train_test_split(
            pairs, test_size=test_ratio, random_state=42, shuffle=True
        )
    else:
        train_val = pairs
        test_data = []
    
    # Second split: train vs val
    val_ratio_adjusted = val_ratio / (train_ratio + val_ratio) if (train_ratio + val_ratio) > 0 else 0
    if val_ratio_adjusted > 0:
        train_data, val_data = train_test_split(
            train_val, test_size=val_ratio_adjusted, random_state=42, shuffle=True
        )
    else:
        train_data = train_val
        val_data = []
    
    print(f"Split sizes - Train: {len(train_data)}, Val: {len(val_data)}, Test: {len(test_data)}")
    
    # Create directory structure
    for split in ['train', 'valid', 'test']:
        (output_dir / split / 'images').mkdir(parents=True, exist_ok=True)
        (output_dir / split / 'labels').mkdir(parents=True, exist_ok=True)
    
    # Track used filenames to handle duplicates
    used_names: Dict[str, Set[str]] = {'train': set(), 'valid': set(), 'test': set()}
    
    def get_unique_name(base_name: str, split: str) -> str:
        """Get a unique filename, adding suffix if needed."""
        name = base_name
        counter = 1
        while name in used_names[split]:
            name = f"{Path(base_name).stem}_{counter}{Path(base_name).suffix}"
            counter += 1
        used_names[split].add(name)
        return name
    
    def process_split(data: List[Tuple[Path, Path, int]], split_name: str):
        """Process a split and copy files."""
        for image_path, label_path, dataset_idx in data:
            # Get unique filenames
            image_name = get_unique_name(image_path.name, split_name)
            label_name = Path(image_name).stem + '.txt'
            
            # Copy image
            image_dst = output_dir / split_name / 'images' / image_name
            copy_file(image_path, image_dst)
            
            # Handle label file - remap class IDs if needed
            label_dst = output_dir / split_name / 'labels' / label_name
            
            # Check if class mapping is needed for this dataset
            mapping = class_mappings.get(dataset_idx, {})
            needs_remapping = any(old != new for old, new in mapping.items())
            
            if needs_remapping:
                # Read, remap, and write label
                with open(label_path, 'r') as f:
                    lines = f.readlines()
                
                remapped_lines = []
                for line in lines:
                    parts = line.strip().split()
                    if parts:
                        old_class = int(parts[0])
                        new_class = mapping.get(old_class, old_class)
                        remapped_lines.append(f"{new_class} {' '.join(parts[1:])}\n")
                
                label_dst.parent.mkdir(parents=True, exist_ok=True)
                with open(label_dst, 'w') as f:
                    f.writelines(remapped_lines)
            else:
                # Just copy the label
                copy_file(label_path, label_dst)
    
    process_split(train_data, 'train')
    process_split(val_data, 'valid')
    process_split(test_data, 'test')
    
    # Generate data.yaml
    data_yaml = {
        'names': class_names,
        'nc': len(class_names),
        'train': './train/images',
        'val': './valid/images',
        'test': './test/images'
    }
    
    with open(output_dir / 'data.yaml', 'w') as f:
        yaml.dump(data_yaml, f, default_flow_style=False)
    
    print(f"Generated data.yaml with {len(class_names)} classes: {class_names}")


def load_class_names(dataset_dir: str) -> List[str]:
    """Load class names from data.yaml in dataset directory."""
    data_yaml = Path(dataset_dir) / 'data.yaml'
    if data_yaml.exists():
        with open(data_yaml, 'r') as f:
            data = yaml.safe_load(f)
            return data.get('names', [])
    return []


def build_label_mapping(label_mapping_config: Dict) -> Tuple[List[str], Dict[str, int]]:
    """
    Build label mapping from config.
    Returns: (final_class_names, mapping_dict)
    where mapping_dict maps old_class_name -> new_class_id
    """
    final_classes = list(label_mapping_config.keys())
    mapping = {}
    
    for new_class, old_class_list in label_mapping_config.items():
        new_class_id = final_classes.index(new_class)
        for old_class in old_class_list:
            mapping[old_class] = new_class_id
    
    return final_classes, mapping


def merge_class_names(all_class_names: List[List[str]], label_mapping: Dict[str, int]) -> Tuple[List[str], Dict[int, Dict[int, int]]]:
    """
    Merge class names from multiple datasets using label mapping.
    Returns: (merged_class_names, class_id_mappings)
    where class_id_mappings[dataset_idx][old_class_id] = new_class_id
    """
    # Get all unique mapped class names
    mapped_classes = set(label_mapping.values())
    
    # Create merged class list (sorted by class ID for consistency)
    merged = sorted(set(label_mapping.values()))
    
    # Build mapping from dataset_idx -> old_id -> new_id
    class_mappings = {}
    
    for dataset_idx, names in enumerate(all_class_names):
        class_mappings[dataset_idx] = {}
        for old_id, old_name in enumerate(names):
            if old_name in label_mapping:
                # Map to new class ID
                new_class_id = merged.index(label_mapping[old_name])
                class_mappings[dataset_idx][old_id] = new_class_id
            else:
                # Class not in mapping - this shouldn't happen if config is correct
                print(f"Warning: Class '{old_name}' in dataset {dataset_idx} not found in label_mapping")
                # Add unmapped class to merged list
                if old_name not in merged:
                    merged.append(old_name)
                class_mappings[dataset_idx][old_id] = merged.index(old_name)
    
    return merged, class_mappings


def main():
    parser = argparse.ArgumentParser(
        description="Combine and re-split multiple YOLO datasets for Google Colab"
    )
    parser.add_argument(
        "--config", 
        type=str, 
        required=True, 
        help="YAML config file containing list of dataset paths and duplicate counts"
    )
    parser.add_argument(
        "--split", 
        type=str, 
        required=True,
        choices=['70_20_10', '80_10_10', '60_20_20', '80_20_0'],
        help="Split ratio for train_val_test (e.g., 70_20_10 or 80_10_10)"
    )
    parser.add_argument(
        "--output-dir", 
        type=str, 
        required=True, 
        help="Output directory for the combined dataset"
    )
    
    args = parser.parse_args()
    
    # Load config
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)
    
    datasets_config = config.get('datasets', [])
    if not datasets_config:
        print("Error: No datasets specified in config file")
        print("Config file should have format:")
        print("datasets:")
        print("  - path: /path/to/dataset1")
        print("    duplicate: 1")
        print("  - path: /path/to/dataset2")
        print("    duplicate: 2")
        print("label_mapping:")
        print("  final_class_name:")
        print("    - original_class_1")
        print("    - original_class_2")
        return 1
    
    # Load label mapping
    label_mapping_config = config.get('label_mapping', {})
    if not label_mapping_config:
        print("Error: No label_mapping specified in config file")
        return 1
    
    # Build the label mapping
    final_classes, label_mapping = build_label_mapping(label_mapping_config)
    print(f"Label mapping configured:")
    for new_class, old_classes in label_mapping_config.items():
        print(f"  {new_class}: {old_classes}")
    print(f"Final classes: {final_classes}")
    
    # Parse split ratios
    split_ratios = parse_split(args.split)
    print(f"Using split ratios - Train: {split_ratios[0]:.0%}, Val: {split_ratios[1]:.0%}, Test: {split_ratios[2]:.0%}")
    
    # Collect all image-label pairs and class names
    all_pairs = []
    all_class_names = []
    
    for dataset_config in datasets_config:
        # Handle both dict and string formats
        if isinstance(dataset_config, dict):
            dataset_dir = dataset_config.get('path')
            duplicate_count = dataset_config.get('duplicate', 1)
        else:
            dataset_dir = dataset_config
            duplicate_count = 1
        
        if not dataset_dir:
            print(f"Warning: Dataset config missing 'path': {dataset_config}")
            continue
        
        # Convert to absolute path if relative
        dataset_path = Path(dataset_dir)
        if not dataset_path.is_absolute():
            # Relative to the config file location
            config_dir = Path(args.config).parent
            dataset_path = (config_dir / dataset_path).resolve()
        
        if not dataset_path.is_dir():
            print(f"Error: Dataset directory does not exist: {dataset_path}")
            return 1
        
        print(f"Scanning dataset: {dataset_path} (duplicate: {duplicate_count}x)")
        pairs = find_image_label_pairs(str(dataset_path))
        print(f"  Found {len(pairs)} image-label pairs")
        
        # Add dataset index to each pair and duplicate based on count
        for _ in range(duplicate_count):
            for img, lbl in pairs:
                all_pairs.append((img, lbl, len(all_class_names)))
        
        class_names = load_class_names(str(dataset_path))
        all_class_names.append(class_names)
        print(f"  Classes: {class_names}")
    
    if not all_pairs:
        print("Error: No image-label pairs found in any dataset")
        return 1
    
    print(f"\nTotal pairs collected (including duplicates): {len(all_pairs)}")
    
    # Merge class names using label mapping
    merged_classes, class_mappings = merge_class_names(all_class_names, label_mapping)
    print(f"Merged classes: {merged_classes}")
    print(f"Class ID mappings created for {len(class_mappings)} datasets")
    
    # Create output directory (handle relative paths)
    output_path = Path(args.output_dir)
    if not output_path.is_absolute():
        config_dir = Path(args.config).parent
        output_path = (config_dir / output_path).resolve()
    
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Generate the new dataset
    generate_dataset(
        all_pairs,
        output_path,
        split_ratios,
        merged_classes,
        class_mappings
    )
    
    print(f"\nDataset successfully created at: {output_path}")
    return 0


if __name__ == "__main__":
    exit(main())
