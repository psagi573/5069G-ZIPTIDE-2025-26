# PID & Motion Profile Tuning Guide

---

## Introduction

This guide helps you tune your robot's PID controllers and motion profiling parameters to achieve smooth, accurate, and fast autonomous driving. It assumes you are using separate PIDs for driving distance (`distPID`) and heading (`headingPID`), along with velocity and acceleration limits for motion profiling.

---

## Part 1: PID Controller Tuning

### What You’re Tuning

- **distPID**: Controls linear distance error
- **headingPID**: Controls angular error (turning)

### Step-by-Step Tuning Procedure

1. **Tune `distPID` first with simple straight drives**  
   Use a command like `drive(24)` or `moveTo(x, y, theta)` straight ahead.

2. **Start with Proportional Gain (kP)**  
   - Begin with a low kP, e.g. 0.5  
   - Increase kP gradually until the robot starts to move smoothly towards the target but **just before oscillation** occurs.

3. **Add Derivative Gain (kD)**  
   - Add small kD to damp oscillations caused by kP  
   - If the robot oscillates (back and forth), increase kD or decrease kP.

4. **Introduce Integral Gain (kI) only if needed**  
   - Add kI if the robot consistently undershoots or stops short  
   - Keep kI small to avoid oscillations and wind-up.

5. **Tune `headingPID` similarly but using turns or heading holds**  
   - Test with simple turns like `turn(90)`  
   - Tune kP for responsiveness, kD to reduce overshoot.

---

### Common Symptoms and Fixes

| Symptom                        | Possible Cause               | Recommended Fix                               |
|--------------------------------|-----------------------------|-----------------------------------------------|
| Robot oscillates around target | kP or kI too high           | Reduce kP or kI, increase kD                  |
| Robot overshoots target        | kP too high                 | Lower kP, increase kD                         |
| Robot moves slowly, undershoots| kP too low                  | Increase kP                                   |
| Robot never reaches target     | No integral term or too low | Add or increase kI slightly                   |
| Robot "stalls" at start        | Output too low or no kick   | Add a minimal voltage kick when output is low |

---

## Part 2: Motion Profile Tuning

### Parameters

- **maxVel**: Maximum velocity the robot can safely reach (in/s or rpm equivalent)  
- **accel**: Maximum acceleration (in/s² or rpm/s equivalent)

### How to Tune

1. Set `maxVel` to your robot’s safe top speed during autonomous.  
   - Start lower to be safe, e.g., 50% of max motor rpm.  
   - Increase gradually to improve speed.

2. Set `accel` to limit acceleration for smooth starts/stops.  
   - Too high causes jerky motions.  
   - Too low causes slow response and inability to reach max speed.

3. Adjust these with real runs on your robot.  
   - If robot “jerks” or wheels slip, reduce acceleration.  
   - If robot is sluggish and slow to reach speeds, increase acceleration.

---

### Behavior Overview

| Scenario                         | What happens                      | What to do                      |
|----------------------------------|-----------------------------------|---------------------------------|
| Acceleration too high            | Robot jerks, wheels slip          | Lower acceleration              |
| Acceleration too low             | Robot slow to reach target speed  | Increase acceleration           |
| Max velocity too high            | Robot overshoots or loses control | Lower maxVel                    |
| Max velocity too low             | Robot is slow overall             | Increase maxVel                 |

---

## Part 3: Putting It All Together - Practical Tips

- Always tune PIDs with motion profile limits in place (maxVel & accel).  
- Start with slow speed, low accel runs to verify PID behavior.  
- Gradually increase maxVel and accel as tuning improves.  
- Use timeout and settle conditions to stop motions gracefully.  
- Implement a small voltage kick when output is near zero but error still large — this overcomes motor static friction.

---

## Bonus: Voltage Kickstart Code Snippet

```cpp
if (fabs(linearOut) < 1.0 && fabs(error) > 1.0)
{
    linearOut = direction * 1.0; // Minimum voltage kick
}
