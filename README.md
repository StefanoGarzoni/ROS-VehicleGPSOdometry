# ROS Vehicle & GPS Odometry with Sector Timing

This project computes odometry from **vehicle status** and **GPS**, publishes the corresponding **TFs**, and estimates **sector split times** on a racetrack. It includes a one-shot ROS launch (nodes + RViz), replays from a **ROS bag**, and lets you set a local ENU frame via GPS reference parameters.

---

🔍 Introduction
This repository enables:

* Computing **vehicle-based odometry** from speed & steering
* Computing **GPS-based odometry** (LLA → ENU) with heading
* Publishing **TF frames** (`odom → base_link`, `odom → gps_link`)
* Calculating **sector split times** and mean speeds
* Visualizing everything in **RViz** and replaying from a **bag**

Everything runs locally in **ROS1 (catkin)** with minimal configuration.

---

🛠️ Features

✔ Dual Odometry – `/odom` from vehicle status, `/gps_odom` from GPS (ENU)
✔ TF Publishing – `odom → base_link` and `odom → gps_link`
✔ Sector Timing – Publishes current sector, time, and mean speed
✔ One-Command Launch – Starts all nodes and RViz with a single file
✔ Bag-Friendly – Works with `rosbag play --clock` and `use_sim_time=true`
✔ Configurable ENU – Set `lat_r`, `lon_r`, `alt_r` as ENU reference

---

⚙️ How It Works?

1️⃣ **Inputs & Preprocessing**
The system subscribes to:

* `/speedsteer` (`geometry_msgs/PointStamped`) → `x: steer [deg]`, `y: speed [km/h]`
* `/swiftnav/front/gps_pose` (`sensor_msgs/NavSatFix`) → GPS LLA

2️⃣ **Vehicle Odometer**
Integrates forward speed (km/h → m/s) and steering into planar pose (`x, y, yaw`) in `odom`, publishing `/odom` and the TF `odom → base_link`.

3️⃣ **GPS Odometer**
Converts LLA to **ECEF** and **ENU** using the reference parameters `lat_r, lon_r, alt_r`. Computes heading from successive positions, publishes `/gps_odom` and TF `odom → gps_link`.

4️⃣ **Sector Times**
Determines the current sector along the track, aggregates **elapsed time** and **mean speed**, and publishes a custom message on `/sector_times`.

5️⃣ **Visualization**
RViz displays both odometries and TF trees. Colors typically distinguish vehicle vs GPS traces for quick comparison.

---

🚀 Quickstart

```bash
# 1) Create workspace & clone
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/StefanoGarzoni/Robotics-Projects.git
cd ..

# 2) Dependencies & build
rosdep update
rosdep install --from-paths src -i -y
catkin_make
source devel/setup.bash

# 3) Replay bag with simulated time
rosparam set use_sim_time true
rosbag play --clock project.bag

# 4) Launch everything (nodes + RViz)
roslaunch first_project launch.launch \
  lat_r:=<lat_ref_deg> lon_r:=<lon_ref_deg> alt_r:=<alt_ref_m>
```

> Tip: set the ENU reference to the **first GPS fix** in your bag for best alignment.

---

📡 ROS Interfaces

**Publishers**

* `/odom` — `nav_msgs/Odometry` (vehicle-based)
* `/gps_odom` — `nav_msgs/Odometry` (GPS ENU)
* `/tf` — `odom → base_link`, `odom → gps_link`
* `/sector_times` — `first_project/SectorTimes`

  ```
  int32  current_sector
  float32 current_sector_time
  float32 current_sector_mean_speed
  ```

**Subscribers**

* `/speedsteer` — `geometry_msgs/PointStamped` (`x: steer_deg`, `y: speed_kmh`)
* `/swiftnav/front/gps_pose` — `sensor_msgs/NavSatFix`

---

🧭 Parameters

* `lat_r` (double) — latitude reference (deg)
* `lon_r` (double) — longitude reference (deg)
* `alt_r` (double) — altitude reference (m)
* (optional) vehicle geometry & scaling (e.g., wheelbase, steering factor, `kmh_to_ms`)

---

🔒 Benefits

✅ Side-by-side odometry comparison (vehicle vs GPS)
✅ Clear TF structure for integration with other stacks
✅ Reproducible replay from bags for debugging & validation

---

NOTE:

* Requires **ROS1** with **catkin**.
* The demo expects a bag file like `project.bag` (not included).
* RViz config and launch file start the full pipeline automatically.

---

📧 Contact
Questions or contributions? Open an **Issue** or reach out on GitHub! 🚀
