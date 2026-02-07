# 2026 Nav2 Upgrade: Master Implementation Guide

## Part 1: Implementation Checklist (Human)

### 1. C++ Code Fixes (`cprt_costmap_plugins`)
**Goal:** Fix the "Gradient Killer" bug and the "Diamond" rotation bug in your custom layer.

- [ ] **Unclamp Lethal Threshold (Critical)**
  Open `src/gridmap_layer.cpp`. Find `getParameters()`.
  - **Change:** `lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);`
  - **To:** `lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 255), 0);`
  - *Why:* The old clamp destroyed your smart gradients, turning everything >0.4 into a solid wall.

- [ ] **Fix Bounding Box Rotation**
  Open `src/gridmap_layer.cpp`. Replace the logic in `updateBounds()` to transform all 4 corners of the grid map.
  - *Why:* The previous logic assumed the map was axis-aligned. When the robot turns 45°, the map becomes a diamond, and the old logic failed to clear the corners, leaving "ghost" obstacles.

- [ ] **Rebuild Workspace**
  Run: `colcon build --packages-select cprt_costmap_plugins`

### 2. Launch File (`bringup.launch.py`)
**Goal:** Fix fatal plugin crashes and ensure parameter consistency.

- [ ] **Fix Plugin Class Name Typo**
  In the `load_composable_nodes` list, find the `nav2_behaviors` entry.
  - **Change:** `plugin="behavior_server::BehaviorServer"`
  - **To:** `plugin="nav2_behaviors::BehaviorServer"`

- [ ] **Fix Namespace Handling**
  In the `load_nodes` group (standard nodes), add the namespace argument to ensure they read params correctly.
  - **Action:** Add `namespace=namespace` to every `Node()` entry.

### 3. Nav2 Configuration (`nav2.yaml`)
**Goal:** Enable "Point Robot" logic, setup Swerve controller, and fix frequencies.

#### A. Costmaps (The "Point Robot" Setup)
*Context: We rely on upstream "Smart Inflation" (0.8m buffer), so Nav2 treats the robot as a point.*

- [ ] **Global Costmap (Planner)**
  - **Remove:** `footprint: "[[...]]"` and `plugins: [..., inflation_layer]`
  - **Set:** `robot_radius: 0.2`
    - *Why:* 0.8m (Upstream) + 0.2m (Nav2) = 1.0m Total Buffer. Safe planning.
  - **Set:** `update_frequency: 10.0`
    - *Why:* Synced with elevation map to prevent planning on stale data.

- [ ] **Local Costmap (Controller)**
  - **Remove:** `footprint: "[[...]]"` and `plugins: [..., inflation_layer]`
  - **Set:** `robot_radius: 0.1`
    - *Why:* 0.8m (Upstream) + 0.1m (Nav2) = 0.9m. Covers your 0.85m corner swing during rotations.
  - **Set:** `update_frequency: 10.0`
    - *Why:* Critical for fast collision checking on a moving rover.
  - **Fix Typo:** Change topic from `/trasversability_map` to `/traversability_map`.

#### B. Controller Server (Swerve Setup)
*Context: Use Rotation Shim to force in-place turns before driving.*

- [ ] **Enable Rotation Shim**
  - **Change:** `controller_plugins: ["FollowPath"]`
  - **Update `FollowPath` configuration:**
    ```yaml
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      
      # Shim Parameters (The Swerve Logic)
      angular_dist_threshold: 0.5  # If path is >30 deg off, spin first
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.0 
      max_angular_accel: 2.0
      simulate_ahead_time: 1.0

      # RPP Parameters (The Driving Logic)
      desired_linear_vel: 1.0 
      lookahead_dist: 1.2
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: 0.05
      use_rotate_to_heading: false # Disable this, let Shim handle it
      allow_reversing: false       # Keep false as discussed
    ```

- [ ] **Tighten Goal Tolerance**
  - In `general_goal_checker`: Set `xy_goal_tolerance: 0.5` (Swerve is precise enough for this).

#### C. Planner Server (Smac Hybrid)
*Context: Optimize for Swerve agility and High Precision.*

- [ ] **Update GridBased Planner**
  - **Set:** `minimum_turning_radius: 0.40` (Swerve can turn tighter than skid-steer).
  - **Set:** `motion_model_for_search: "DUBIN"` (Forward-only; Shim handles turns).
  - **Set:** `downsampling_factor: 1`
    - *Why:* Downsampling "blurs" your precise upstream inflation. Factor 1 keeps the native 0.2m resolution so gaps stay open.

#### D. Behavior Server
- [ ] **Standardize Plugin Names**
  - Update all plugins to use `::` format (e.g., `nav2_behaviors::Spin`) instead of `/`.

### 4. Behavior Tree (`Maps.xml`)
**Goal:** Implement the "Back Up & Spin" recovery for forward-only planning.

- [ ] **Replace `RecoveryFallback` Block**
  Replace the bottom `<ReactiveFallback name="RecoveryFallback">` section with:
  ```xml
  <ReactiveFallback name="RecoveryFallback">
    <GoalUpdated/>
    <RoundRobin name="RecoveryActions">
      
      <Sequence name="ClearLocalSeq">
        <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
        <Wait wait_duration="1.0"/>
      </Sequence>

      <Sequence name="BackupSeq">
        <BackUp backup_dist="0.30" backup_speed="0.05"/>
        <Wait wait_duration="0.5"/>
      </Sequence>

      <Sequence name="SpinSeq">
        <Spin spin_dist="3.14" time_allowance="5.0"/> 
        <Wait wait_duration="1.0"/>
      </Sequence>

      <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>

    </RoundRobin>
  </ReactiveFallback>


# Context: Project Rover Navigation Upgrade (2025 -> 2026)

This document outlines the architectural changes, hardware upgrades, and design constraints for the rover's autonomous navigation stack. Use this context to understand why specific changes are being applied to the `nav2.yaml`, launch files, and behavior trees.

## 1. Hardware Evolution
* **Previous Platform (2025):**
    * **Chassis:** High-friction Skid-Steer.
    * **Issues:** Inability to point turn reliably, high friction causing stalling on tight turns, required wide planning radii (`min_turning_radius: 0.8`), and frequently got stuck.
* **Current Platform (2026):**
    * **Chassis:** 4-Wheel Independent Swerve Drive.
    * **Capabilities:** Full -90° to +90° steering on each wheel. Capable of efficient "Ackermann-style" driving and zero-radius point turns.
    * **Dimensions:** 1.2m x 1.2m (Square).
    * **Sensors:** Front-facing camera, Lidar, RTK GPS (0.2m accuracy).

## 2. Navigation Architecture Strategy
We use a specialized pipeline that differs from standard Nav2 setups:
* **Global Planning:** Custom "Event Horizon Planner" (handles long-range goals >300m) feeding into `SmacPlannerHybrid` (handles kinematic feasibility).
* **Mapping:** Custom `PreserveCostInflationFilter` upstream.
    * *Crucial Detail:* We inflate obstacles *upstream* in the elevation map before Nav2 sees them.
    * *Current Setting:* `coreInflationRadius: 0.8m`. This builds a "Lethal" wall 0.8m around every obstacle.
* **Localization:** EKF fusing IMU, Wheel Encoders, and RTK GPS data.

## 3. Key Configuration Decisions & Rationale

### A. The "Point Robot" Configuration (Costmaps)
**Change:** Removed `footprint` polygon; set `robot_radius` to small values (0.2m Global, 0.1m Local).
**Why:**
* Since obstacles are already inflated by 0.8m upstream, defining a full footprint in Nav2 results in "Double Inflation" (1.3m+ buffer), making valid gaps appear impassable.
* **Logic:**
    * **Planner Safety:** Upstream (0.8m) + Nav2 Global Radius (0.2m) = **1.0m Total Buffer**. (Safe planning).
    * **Controller Limit:** Upstream (0.8m) + Nav2 Local Radius (0.1m) = **0.9m Total Buffer**. (Prevents corners clipping during rotation, since circumscribed radius is ~0.85m).

### B. Swerve Controller Setup
**Change:** Switched to `RegulatedPurePursuit` wrapped in `RotationShimController`.
**Why:**
* The `SmacPlannerHybrid` is configured for **Dubin (Forward-Only)** paths to prevent dangerous reversing with only a front camera.
* The **Rotation Shim** detects when the planner requests a sharp turn (which the Dubin planner allows due to the "Point Robot" config) and automatically **Point Turns** the swerve drive to face the path before driving.
* This mimics "Force Point Turn" behavior without needing a separate planner.

### C. Launch File Fixes
**Change:**
1.  Fixed `nav2_behaviors` plugin namespace typo (Critical crash fix).
2.  Added `namespace` argument to all standard `Node` entries.
**Why:**
* Prevents runtime crashes when Composition is enabled.
* Ensures parameter overrides (like `use_sim_time`) propagate correctly to non-composed nodes.

### D. Recovery Behavior (Forward-Only)
**Change:** Updated Behavior Tree `RecoveryFallback` to "Backup (0.3m) -> Spin (180°)".
**Why:**
* The planner is configured as forward-only (Dubin). If the robot faces a dead end, the planner fails.
* Backing up slightly un-wedges the robot.
* Spinning 180° mechanically puts the empty space *in front* of the sensors and planner, allowing the forward-only logic to find a valid exit path.

## 4. Constraints for Future Code Generation
* **Do not re-enable Nav2 Inflation Layer:** We rely on the upstream `PreserveCostInflationFilter`.
* **Do not increase Update Frequencies arbitrarily:** Global costmap runs at ~2Hz (sufficient for static terrain), but Local Costmap MUST run at ~10Hz+ for collision safety.
* **Do not enable reversing:** Blind spots behind the rover make automatic reversing dangerous.