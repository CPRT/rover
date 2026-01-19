import roboflow
import os
import argparse
import yaml

def get_dataset_url(url: str) -> str:
    """
    Get the dataset URL from the given URL
    """
    if "universe.roboflow.com" in url:
        return url[url.index("universe.roboflow.com") + len("universe.roboflow.com"):]

    return url

def _parse_url(url: str):
    """Parse Roboflow URL to extract workspace, project, and version"""
    url = get_dataset_url(url)
    parts = url.strip('/').split('/')
    workspace = parts[0] if len(parts) > 0 else None
    project = parts[1] if len(parts) > 1 else None
    
    # Look for a numeric version in the remaining parts
    version = None
    for i in range(2, len(parts)):
        # Try to parse as integer, skip non-numeric parts like "dataset"
        try:
            version = str(int(parts[i]))
            break
        except ValueError:
            continue
    
    return workspace, project, version

def roboflow_download_func(datasetUrl: str, modelFormat: str, outputDir: str, api_key: str):
    rf = roboflow.Roboflow(api_key=api_key)
    w, p, v = _parse_url(datasetUrl)
    project = rf.workspace(w).project(p)
    if not v:
        versions = project.versions()
        if not versions:
            print(f"project {p} does not have any version. exiting")
            exit(1)
        version = versions[-1]
        print(f"Version not provided. Downloading last one ({version.version})")
    else:
        version = project.version(int(v))
    version.download(modelFormat, location=outputDir, overwrite=True)

def download_roboflow_dataset(url: str, dataset_version: int, model_format: str, output_dir: str, api_key: str) -> bool:
    """
    Download the dataset from the given URL to the output directory
    """
    dataset_url = f"{url}/{dataset_version}"
    dataset_name = dataset_url.replace("/", "_").replace("https:", "").replace("universe.roboflow.com", "")
    dataset_path = os.path.join(output_dir, dataset_name)

    if os.path.exists(dataset_path):
        print(f"Dataset already exists at {dataset_path}")
        return False

    os.makedirs(output_dir, exist_ok=True)

    print(f"Downloading dataset to {dataset_path}")
    roboflow_download_func(dataset_url, model_format, dataset_path, api_key)
    print(f"Dataset downloaded to {dataset_path}")
    return True

def load_config(config_path: str) -> dict:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def download_datasets_from_config(config_path: str, model_format: str, base_output_dir: str, api_key: str):
    """Download all datasets specified in the config file"""
    config = load_config(config_path)
    
    if 'datasets' not in config:
        print("No datasets found in config file")
        return
    
    datasets = config['datasets']
    print(f"Found {len(datasets)} dataset(s) in config")
    
    for i, dataset in enumerate(datasets, 1):
        if 'url' not in dataset or 'path' not in dataset:
            print(f"Skipping dataset {i}: missing 'url' or 'path' field")
            continue
        
        url = dataset['url']
        relative_path = dataset['path']
        dataset_version = dataset.get('version', 1)  # Default to version 1 if not specified
        dataset_name = dataset.get('name', f'dataset_{i}')
        
        # Construct the full output directory
        output_dir = os.path.join(base_output_dir, relative_path)
        
        print(f"\n[{i}/{len(datasets)}] Processing dataset: {dataset_name}")
        print(f"  URL: {url}")
        print(f"  Version: {dataset_version}")
        print(f"  Output: {output_dir}")
        
        try:
            download_roboflow_dataset(url, dataset_version, model_format, base_output_dir, api_key)
        except Exception as e:
            print(f"  Error downloading dataset: {e}")
            continue

def main():
    parser = argparse.ArgumentParser(description="Download datasets from Roboflow using a config file")
    parser.add_argument("--config", type=str, required=True, help="Path to YAML configuration file")
    parser.add_argument("--model-format", type=str, default="yolov9", help="Model format (default: yolov9)")
    parser.add_argument("--output-dir", type=str, required=True, help="Base output directory for all datasets")
    parser.add_argument("--api-key", type=str, help="Roboflow API key (defaults to ROBOFLOW_API_KEY environment variable)")
    
    args = parser.parse_args()
    
    # Get API key from argument or environment variable
    api_key = args.api_key or os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        parser.error("API key must be provided via --api-key argument or ROBOFLOW_API_KEY environment variable")
    
    if not os.path.exists(args.config):
        parser.error(f"Config file not found: {args.config}")
    
    download_datasets_from_config(args.config, args.model_format, args.output_dir, api_key)
    print("\nAll datasets processed successfully")

if __name__ == "__main__":
    main()

