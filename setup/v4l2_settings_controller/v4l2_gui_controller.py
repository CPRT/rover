import tkinter as tk
from tkinter import ttk, messagebox
import paramiko
import yaml
import os
import re

# --- SSH CONFIGURATION ---
SSH_HOST = "192.168.0.55"
SSH_USER = "cprt"
SSH_PASS = "cprt"  # Or set to None if using key
SSH_KEY_PATH = None  # e.g., "~/.ssh/id_rsa"


class CameraControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Remote Camera Controller")
        self.root.geometry("700x750")  # Made slightly wider for menu text

        self.config = {}
        self.load_configs()

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)

        self.tabs = {}
        self.build_gui()

    def run_ssh_command(self, command):
        """Stateless execution: Connects, runs, and closes immediately."""
        print(f"[SSH DEBUG] Executing: {command}")

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            if SSH_KEY_PATH:
                ssh.connect(
                    SSH_HOST, username=SSH_USER, key_filename=SSH_KEY_PATH, timeout=5
                )
            else:
                ssh.connect(SSH_HOST, username=SSH_USER, password=SSH_PASS, timeout=5)

            stdin, stdout, stderr = ssh.exec_command(command)

            out = stdout.read().decode().strip()
            err = stderr.read().decode().strip()
            return out, err

        except Exception as e:
            print(f"[SSH CONNECTION ERROR] {e}")
            return "", str(e)

        finally:
            ssh.close()

    def load_configs(self):
        if not os.path.exists("camera_config.yaml"):
            messagebox.showerror(
                "Error", "camera_config.yaml not found. Run scanner.py first."
            )
            self.root.destroy()
            return

        with open("camera_config.yaml", "r") as f:
            self.config = yaml.safe_load(f)

    def build_gui(self):
        for cam_name, cam_data in self.config.items():
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=cam_name)

            # Header actions
            header = ttk.Frame(tab)
            header.pack(fill="x", pady=5)

            ttk.Button(
                header,
                text="Check Path Exists",
                command=lambda n=cam_name: self.check_path(n),
            ).pack(side="left", padx=5)

            light_canvas = tk.Canvas(header, width=20, height=20)
            light_canvas.pack(side="left", padx=5)
            light_id = light_canvas.create_oval(2, 2, 18, 18, fill="gray")

            ttk.Button(
                header,
                text="Get Settings",
                command=lambda n=cam_name: self.get_settings(n),
            ).pack(side="right", padx=5)

            # Scrollable control area
            container = ttk.Frame(tab)
            container.pack(fill="both", expand=True)
            canvas = tk.Canvas(container)
            scrollbar = ttk.Scrollbar(
                container, orient="vertical", command=canvas.yview
            )
            scrollable_frame = ttk.Frame(canvas)

            scrollable_frame.bind(
                "<Configure>",
                lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
            )
            canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
            canvas.configure(yscrollcommand=scrollbar.set)

            canvas.pack(side="left", fill="both", expand=True)
            scrollbar.pack(side="right", fill="y")

            # Generate controls
            gui_vars = {}
            menu_vars = {}
            last_state = {}

            for ctrl_name, props in cam_data["controls"].items():
                row = ttk.Frame(scrollable_frame)
                row.pack(fill="x", pady=5, padx=5)

                ttk.Label(row, text=ctrl_name, width=28).pack(side="left")

                initial_val = props.get("value", 0)
                var = tk.IntVar(value=initial_val)
                gui_vars[ctrl_name] = var
                last_state[ctrl_name] = initial_val

                if props["type"] in ["int", "menu"]:
                    min_val = props.get("min", 0)
                    max_val = props.get("max", 255)

                    val_label = ttk.Label(row, text=str(var.get()), width=5)

                    def update_label(*args, v=var, l=val_label):
                        l.config(text=str(v.get()))

                    var.trace_add("write", update_label)

                    scale = ttk.Scale(
                        row,
                        from_=min_val,
                        to=max_val,
                        variable=var,
                        orient="horizontal",
                    )
                    scale.pack(side="left", fill="x", expand=True, padx=5)
                    val_label.pack(side="left")

                    # Dedicated text label for menu types
                    if props["type"] == "menu":
                        menu_var = tk.StringVar(value="(Click Get Settings)")
                        menu_vars[ctrl_name] = menu_var
                        ttk.Label(
                            row, textvariable=menu_var, width=25, foreground="blue"
                        ).pack(side="left", padx=5)

                elif props["type"] == "bool":
                    ttk.Checkbutton(row, variable=var).pack(side="left")

            # Footer actions (Save/Load & Set)
            footer = ttk.Frame(tab)
            footer.pack(fill="x", pady=10, padx=5)

            file_frame = ttk.Frame(footer)
            file_frame.pack(side="left")
            ttk.Button(
                file_frame,
                text="Load Settings",
                command=lambda n=cam_name: self.load_settings(n),
            ).pack(side="left", padx=5)
            ttk.Button(
                file_frame,
                text="Save Settings",
                command=lambda n=cam_name: self.save_settings(n),
            ).pack(side="left", padx=5)

            ttk.Button(
                footer,
                text="Set Settings",
                command=lambda n=cam_name: self.set_settings(n),
            ).pack(side="right", padx=5)

            self.tabs[cam_name] = {
                "path": cam_data["path"],
                "vars": gui_vars,
                "menu_vars": menu_vars,
                "last_state": last_state,
                "light_canvas": light_canvas,
                "light_id": light_id,
            }

    def check_path(self, cam_name):
        path = self.tabs[cam_name]["path"]
        cmd = f"test -e {path} && echo 'exists' || echo 'missing'"
        out, err = self.run_ssh_command(cmd)

        canvas = self.tabs[cam_name]["light_canvas"]
        light_id = self.tabs[cam_name]["light_id"]

        if out == "exists":
            canvas.itemconfig(light_id, fill="green")
        else:
            canvas.itemconfig(light_id, fill="red")

    def get_settings(self, cam_name):
        path = self.tabs[cam_name]["path"]
        cmd = f"v4l2-ctl -d {path} --list-ctrls"
        out, err = self.run_ssh_command(cmd)

        if not out:
            print(f"Failed to get settings. Error: {err}")
            return

        # Upgraded Regex: Now captures name, value, AND optional trailing menu text in parentheses
        pattern = re.compile(
            r"^\s*([a-zA-Z0-9_]+)\s+.*value=(-?\d+)(?:\s+\(([^)]+)\))?"
        )

        for line in out.split("\n"):
            match = pattern.match(line)
            if match:
                ctrl_name = match.group(1)
                val = int(match.group(2))
                menu_text = match.group(3)

                if ctrl_name in self.tabs[cam_name]["vars"]:
                    self.tabs[cam_name]["vars"][ctrl_name].set(val)
                    self.tabs[cam_name]["last_state"][ctrl_name] = val

                    # If this is a menu control and we captured text, update the UI label
                    if ctrl_name in self.tabs[cam_name]["menu_vars"] and menu_text:
                        self.tabs[cam_name]["menu_vars"][ctrl_name].set(
                            f"({menu_text})"
                        )

        print(f"Updated GUI and synchronized state for {cam_name}")

    def save_settings(self, cam_name):
        filename = f"{cam_name}_settings.yaml"
        current_data = {k: v.get() for k, v in self.tabs[cam_name]["vars"].items()}

        try:
            with open(filename, "w") as f:
                yaml.dump(current_data, f, default_flow_style=False)
            print(f"Saved current {cam_name} configuration to {filename}")
        except Exception as e:
            print(f"Failed to save settings: {e}")

    def load_settings(self, cam_name):
        filename = f"{cam_name}_settings.yaml"

        if not os.path.exists(filename):
            print(f"No profile found at {filename}. Save one first.")
            return

        try:
            with open(filename, "r") as f:
                saved_data = yaml.safe_load(f)

            for ctrl_name, saved_val in saved_data.items():
                if ctrl_name in self.tabs[cam_name]["vars"]:
                    self.tabs[cam_name]["vars"][ctrl_name].set(saved_val)

            print(
                f"Loaded {filename} into GUI. Click 'Set Settings' to push these changes to the camera."
            )
        except Exception as e:
            print(f"Failed to load settings: {e}")

    def set_settings(self, cam_name):
        path = self.tabs[cam_name]["path"]
        changes = {}

        for ctrl_name, var in self.tabs[cam_name]["vars"].items():
            current_val = var.get()
            if current_val != self.tabs[cam_name]["last_state"][ctrl_name]:
                changes[ctrl_name] = current_val

        if not changes:
            print(f"No settings changed for {cam_name}. Skipping SSH command.")
            return

        sorted_ctrl_names = sorted(
            changes.keys(), key=lambda k: 0 if "auto" in k.lower() else 1
        )

        ctrl_strings = [f"{k}={changes[k]}" for k in sorted_ctrl_names]

        ctrl_string = ",".join(ctrl_strings)
        cmd = f"v4l2-ctl -d {path} -c {ctrl_string}"
        out, err = self.run_ssh_command(cmd)

        if err:
            print(f"[Warning] v4l2-ctl reported: {err}")
        else:
            print(f"Successfully applied: {ctrl_string}")
            for k, v in changes.items():
                self.tabs[cam_name]["last_state"][k] = v


if __name__ == "__main__":
    root = tk.Tk()
    app = CameraControllerApp(root)
    root.mainloop()
