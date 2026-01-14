import roboflow
import os
import argparse

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

def main():
    parser = argparse.ArgumentParser(description="Download datasets from Roboflow")
    parser.add_argument("--url", type=str, required=True, help="Roboflow dataset URL")
    parser.add_argument("--dataset-version", type=int, default=1, help="Dataset version (default: 1)")
    parser.add_argument("--model-format", type=str, default="yolov9", help="Model format (default: yolov9)")
    parser.add_argument("--output-dir", type=str, required=True, help="Output directory for the dataset")
    parser.add_argument("--api-key", type=str, help="Roboflow API key (defaults to ROBOFLOW_API_KEY environment variable)")
    
    args = parser.parse_args()
    
    # Get API key from argument or environment variable
    api_key = args.api_key or os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        parser.error("API key must be provided via --api-key argument or ROBOFLOW_API_KEY environment variable")
    
    if download_roboflow_dataset(args.url, args.dataset_version, args.model_format, args.output_dir, api_key):
        print("Dataset downloaded successfully")

if __name__ == "__main__":
    main()

