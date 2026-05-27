import paramiko
import yaml
import re
import os

# --- SSH CONFIGURATION ---
SSH_HOST = "192.168.0.55"
SSH_USER = "cprt"
SSH_PASS = "cprt" # Or set to None if using key
SSH_KEY_PATH = None   # e.g., "~/.ssh/id_rsa"

def run_ssh_command(ssh, command):
    print(f"[SSH DEBUG] Executing: {command}")
    stdin, stdout, stderr = ssh.exec_command(command)
    return stdout.read().decode().strip()

def parse_v4l2_output(output):
    controls = {}
    # Matches lines like: brightness 0x00980900 (int) : min=0 max=255 step=1 default=128 value=128
    pattern = re.compile(r"^\s*([a-zA-Z0-9_]+)\s+(0x[0-9a-fA-F]+)\s+\(([^)]+)\)\s+:\s+(.*)$")
    
    for line in output.split('\n'):
        match = pattern.match(line)
        if match:
            name = match.group(1)
            ctrl_type = match.group(3)
            props_str = match.group(4)
            
            props = {'type': ctrl_type}
            # Extract key=value pairs from the properties string
            for prop in props_str.split():
                if '=' in prop:
                    k, v = prop.split('=', 1)
                    try:
                        props[k] = int(v)
                    except ValueError:
                        props[k] = v
            controls[name] = props
    return controls

def main():
    if not os.path.exists("cameras.yaml"):
        print("cameras.yaml not found. Run find_cameras.py first.")
        return

    with open("cameras.yaml", "r") as f:
        cameras = yaml.safe_load(f)

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        if SSH_KEY_PATH:
            ssh.connect(SSH_HOST, username=SSH_USER, key_filename=SSH_KEY_PATH)
        else:
            ssh.connect(SSH_HOST, username=SSH_USER, password=SSH_PASS)
            
        full_config = {}
        for cam_name, cam_path in cameras.items():
            print(f"Fetching controls for {cam_name} ({cam_path})...")
            cmd = f"v4l2-ctl -d {cam_path} --list-ctrls"
            output = run_ssh_command(ssh, cmd)
            
            if "Failed to open" in output or not output:
                print(f"Warning: Could not read controls for {cam_name}. Is it plugged in?")
                continue
                
            controls = parse_v4l2_output(output)
            full_config[cam_name] = {'path': cam_path, 'controls': controls}
            
        with open("camera_config.yaml", "w") as f:
            yaml.dump(full_config, f, default_flow_style=False)
            
        print("Configuration saved to camera_config.yaml")
        
    finally:
        ssh.close()

if __name__ == "__main__":
    main()