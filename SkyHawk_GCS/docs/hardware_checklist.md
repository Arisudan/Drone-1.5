# Hardware Checklist

Tick each box as it passes on the real hardware. Do props off first, then tethered, then free flight.

## 1. Communication
- [ ] 1.1 Laptop to Radxa link works
- [ ] 1.2 Radxa to Pixhawk link works
- [ ] 1.3 ROS2 topics publish (camera, odometry, map)
- [ ] 1.4 Telemetry matches the real drone
- [ ] 1.5 NO LINK shows on loss, and it recovers
- [ ] 1.6 GCS and vision bridge don't share sysid 255

## 2. Sensors and SLAM
- [ ] 2.1 D435i streams run (stereo, color, IMU)
- [ ] 2.2 Tracking holds, no drift or jumps
- [ ] 2.3 Heading is stable after a tracking reset
- [ ] 2.4 Map builds and updates
- [ ] 2.5 Pixhawk sensors are healthy
- [ ] 2.6 Calibrations are done

## 3. Flight controller
- [ ] 3.1 Vision position is accepted by EKF2
- [ ] 3.2 OFFBOARD engages with the 20 Hz stream
- [ ] 3.3 Parameters stay after a power cycle
- [ ] 3.4 Motor outputs are mapped correctly

## 4. Commands
- [ ] 4.1 Arm and disarm
- [ ] 4.2 Takeoff reaches the right height
- [ ] 4.3 Mode change
- [ ] 4.4 Hold, land and RTL
- [ ] 4.5 Move and yaw
- [ ] 4.6 Emergency kill stops the motors
- [ ] 4.7 Every command gets a reply

## 5. RC and GCS together
- [ ] 5.1 RC channels show in the GCS
- [ ] 5.2 RC mode switch changes the mode
- [ ] 5.3 RC takes over from the GCS
- [ ] 5.4 RC kill switch works
- [ ] 5.5 Mode box follows the RC, and the override prompt appears

## 6. Navigation
- [ ] 6.1 Position hold is steady
- [ ] 6.2 Route runs, pauses, resumes and aborts
- [ ] 6.3 Path planner avoids obstacles

## 7. Safety
- [ ] 7.1 Vision-loss failsafe
- [ ] 7.2 Link-loss failsafe
- [ ] 7.3 RC-loss failsafe
- [ ] 7.4 Low-battery failsafe
- [ ] 7.5 "Block ARM until ready" works

## 8. Video
- [ ] 8.1 Feed shows with the right state
- [ ] 8.2 Latency and range are acceptable

## 9. Peripherals
- [ ] 9.1 Motor test, props off
- [ ] 9.2 ESP32 servo works

## 10. Power and mechanical
- [ ] 10.1 Battery reading is correct under load
- [ ] 10.2 Motors, ESCs and props: direction and heat
- [ ] 10.3 Camera mount is firm, vibration low
