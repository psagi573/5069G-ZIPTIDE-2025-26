# VEX V5 Motion Control System - Velocity-Controlled PID with Odometry

This project is a custom motion control system for VEX V5 robots built in VEXcode Pro (C++), designed to support fast, precise autonomous routines using separate PID loops for driving and turning, odometry-based localization, and velocity profiling.

## ✨ Features

- ✅ Velocity-controlled PID (VCPID) system for separate `drive()` and `turn()` control  
- ✅ Odometry-based position tracking using:
  - Dual rotation sensors (X & Y)
  - Inertial sensor for heading
- ✅ Motion control modes:
  - `drive()` – linear movement with heading hold
  - `turn()` – in-place pivot turns
  - `arc()` – curved turns with both sides moving
  - `sweep()` – one side drives, one side stationary
- ✅ Trapezoidal velocity profiling for smooth acceleration
- ✅ Clamp system for speed capping and control stability
- 🔜 Upcoming:
  - `moveTo(x, y, θ)` using 2D position targets
  - Hermite spline path-following system for advanced Skills routines
  - Marker events during paths (e.g., intake toggle at certain points)

## 📂 File Structure

\`\`\`
src/
├── main.cpp            # Main robot code
├── motion.cpp          # Core motion control logic (VCPID, odometry, drive modes)
├── motion.h            # Function declarations
├── odometry.cpp        # Odometry position tracking
├── odometry.h
├── autonSelector.cpp   # Autonomous routine selector (on Brain screen)
├── robot-config.cpp    # VEX device configuration
\`\`\`

## 🧠 Requirements

- VEXcode Pro V5 (VS Code or official IDE)
- VEX V5 Brain, Motors, Inertial Sensor, and 2x Rotation Sensors
- Basic understanding of PID control and VEX device setup

## 🚀 Getting Started

1. Clone this repo:
   \`\`\`bash
   git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
   \`\`\`

2. Open the folder in VEXcode Pro or VS Code with the VEX V5 extension.

3. Ensure your robot config matches the motor/sensor setup:
   - Left motors: L1, L2, L3
   - Right motors: R7, R8
   - Inertial sensor: inertial19
   - Rotation sensors: xTracker, yTracker

4. Build and upload to your robot.

## 📸 Autonomous Modes

Autonomous routines are selected on the Brain using a touchscreen UI. You can customize \`autonSelector.cpp\` to define multiple starting positions and routines.

## 📈 Tuning

PID constants are set in \`motion.cpp\`. You can view tuning values live on the Brain screen or Controller for quick testing. Adjust:
- \`kP\`, \`kI\`, \`kD\` for drive and turn
- Max velocity and acceleration for trapezoidal control
- Odometry scale factors

## 📜 License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## 🙌 Contributions

Pull requests are welcome! If you have improvements to the controller logic, better spline generation, or general optimization tips, feel free to open an issue or PR.

---

*Built by a high school VEX student passionate about clean, efficient robotics code.*
