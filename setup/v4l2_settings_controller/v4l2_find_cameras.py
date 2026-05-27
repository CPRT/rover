import paramiko
import yaml
import sys

# --- SSH CONFIGURATION ---
SSH_HOST = "192.168.0.55"
SSH_USER = "cprt"
SSH_PASS = "cprt" # Or set to None if using key
SSH_KEY_PATH = None   # e.g., "~/.ssh/id_rsa"

def run_ssh_command(ssh, command):
    print(f"[SSH DEBUG] Executing: {command}")
    stdin, stdout, stderr = ssh.exec_command(command)
    err = stderr.read().decode().strip()
    if err and "No such file" not in err:
        print(f"[SSH ERROR] {err}")
    return stdout.read().decode().strip()

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        if SSH_KEY_PATH:
            ssh.connect(SSH_HOST, username=SSH_USER, key_filename=SSH_KEY_PATH)
        else:
            ssh.connect(SSH_HOST, username=SSH_USER, password=SSH_PASS)
            
        print("Scanning for V4L devices by-id...")
        output = run_ssh_command(ssh, "ls -1 /dev/v4l/by-id/")
        
        if not output:
            print("No cameras found in /dev/v4l/by-id/")
            sys.exit(1)
            
        cameras = {}
        for idx, line in enumerate(output.split('\n')):
            if line.strip():
                # Provide a default name that you can edit later
                cameras[f"Camera_{idx}"] = f"/dev/v4l/by-id/{line.strip()}"
                
        with open("cameras.yaml", "w") as f:
            yaml.dump(cameras, f, default_flow_style=False)
            
        print("Saved to cameras.yaml. Please edit the keys to name your cameras (e.g., 'Drive', 'EEF').")
        
    finally:
        ssh.close()

if __name__ == "__main__":
    main()