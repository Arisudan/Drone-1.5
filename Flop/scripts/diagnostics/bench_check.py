#!/usr/bin/env python3
"""Bench check + publisher: the FC's IMU + attitude + body rates, exactly as sent.

Reads the flight controller over USB/UART and PUBLISHES the raw inertial data to
ROS 2 topics *without any frame conversion* -- i.e. exactly what the FC reports,
in PX4's own body-FRD / world-NED convention. This is the un-massaged counterpart
to fc_link.py (which converts NED/FRD -> ENU/FLU for the mrs_lite pipeline).

Each message is parsed with an appropriate header:
  * header.stamp   = the FC packet's OWN timestamp (HIGHRES_IMU.time_usec), so the
                     stamp is the sensor sample time, not the receipt time.
                     (--ros-time uses the local ROS clock instead.)
  * header.frame_id = --frame-id (default base_link_frd; data is FRD/NED).

Publishes:
  /fc/imu   sensor_msgs/Imu            linear_acceleration (m/s^2, FRD, HIGHRES_IMU)
                                       angular_velocity    (rad/s, FRD, raw gyro)
                                       orientation         (quat, body->NED, ATTITUDE)
  /fc/mag   sensor_msgs/MagneticField  magnetic field (Gauss as reported, FRD)

While running it prints the LIVE publish rate of each topic (Hz) once per second,
and a received-vs-published summary on exit.

Run (ROS 2 must be sourced so rclpy/sensor_msgs import):
    source /opt/ros/jazzy/setup.bash
    python3 bench_check.py                        # /dev/ttyACM0, req 100 Hz, forever
    python3 bench_check.py --port /dev/ttyUSB0 --baud 921600 --rate 200
If ROS 2 is not sourced it falls back to console-only diagnostics.
"""
import argparse
import math
import time

from pymavlink import mavutil

DEG = 180.0 / math.pi

# ROS 2 is optional: without it, this stays a console diagnostic.
try:
    import rclpy
    from rclpy.time import Time as RclTime
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, MagneticField
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False


def euler_to_quat(roll, pitch, yaw):
    """RPY -> quaternion (x, y, z, w). No sign flips: exact FC (body->NED) attitude."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def run_scan(m, dur):
    """Listen to the FC's DEFAULT streams for `dur` s; list every msg type + Hz.

    Does not request any custom rates -- shows exactly what the FC emits by default
    on this link, so you see the real, board-specific set of message types.
    """
    seen = {}   # type -> [count, t_first, t_last]
    print(f'\nscanning default streams for {dur:.0f}s (no custom rates requested)...\n')
    t_end = time.time() + dur
    try:
        while time.time() < t_end:
            msg = m.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'BAD_DATA':
                continue
            now = time.time()
            s = seen.get(t)
            if s is None:
                seen[t] = [1, now, now]
            else:
                s[0] += 1
                s[2] = now
    except KeyboardInterrupt:
        pass

    print(f'--- {len(seen)} message types seen (sorted by rate) ---')
    print(f'  {"MESSAGE":26s} {"count":>6s}   {"Hz":>7s}')
    rows = []
    for name, (count, t0, t1) in seen.items():
        hz = (count - 1) / (t1 - t0) if count >= 2 and t1 > t0 else 0.0
        rows.append((hz, count, name))
    for hz, count, name in sorted(rows, reverse=True):
        print(f'  {name:26s} {count:6d}   {hz:7.1f}')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='auto',
                    help="FC serial device, or 'auto' to probe USB-CDC ports (/dev/ttyACM*) "
                         "only. For a ttyUSB UART (TELEM), pass it explicitly. Or a udp url.")
    ap.add_argument('--baud', type=int, default=57600,
                    help='UART baud (e.g. SER_TEL2_BAUD). Ignored for USB CDC; auto tries several.')
    ap.add_argument('--secs', type=float, default=0.0,
                    help='run duration; 0 = until Ctrl-C')
    ap.add_argument('--rate', type=float, default=100.0,
                    help='requested stream rate (Hz) for IMU + attitude')
    ap.add_argument('--max', action='store_true',
                    help='request the FC max rate (smallest interval; the FC caps it)')
    ap.add_argument('--scan', action='store_true',
                    help='listen to the FC default streams and list every message type + Hz')
    ap.add_argument('--frame-id', default='base_link_frd',
                    help='frame_id stamped on published msgs (data is FRD/NED, unconverted)')
    ap.add_argument('--ros-time', action='store_true',
                    help='stamp header with local ROS clock instead of the FC packet time')
    ap.add_argument('--no-ros', action='store_true', help='console only, do not publish')
    args = ap.parse_args()

    publish = HAVE_ROS and not args.no_ros and not args.scan
    node = pub_imu = pub_mag = None
    if publish:
        rclpy.init()
        node = Node('fc_bench_publisher')
        pub_imu = node.create_publisher(Imu, '/fc/imu', 20)
        pub_mag = node.create_publisher(MagneticField, '/fc/mag', 20)
        print('ROS 2: publishing /fc/imu (sensor_msgs/Imu) and /fc/mag (MagneticField)')
    elif not HAVE_ROS:
        print('ROS 2 not available (rclpy import failed) -> console-only diagnostics.')

    def make_stamp(time_usec):
        """Header stamp from the FC packet time (usec), or ROS clock with --ros-time."""
        if args.ros_time or not time_usec:
            return node.get_clock().now().to_msg()
        return RclTime(nanoseconds=int(time_usec) * 1000).to_msg()

    from fc_serial import open_link
    m, port, baud, hb = open_link(args.port, args.baud)
    if hb is None:
        print('NO HEARTBEAT on any candidate port. Checks: cable/port, permissions '
              '(add yourself to "dialout": sudo usermod -aG dialout $USER, then re-login), '
              'the UART baud matches SER_TELx_BAUD, and the port is not held by '
              'QGC/px4_control.py.')
        return 1
    args.baud = baud   # remember what actually worked
    ap_name = mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name
    print(f'HEARTBEAT on {port} @ {baud}: sys {m.target_system} '
          f'comp {m.target_component} autopilot={ap_name}')

    if args.scan:
        return run_scan(m, args.secs if args.secs > 0 else 5.0)

    comp = m.target_component or 1
    req_rate = 2000.0 if args.max else args.rate   # 2 kHz req => FC streams at its ceiling
    interval_us = max(1, int(1e6 / req_rate))
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
                   mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
        m.mav.command_long_send(m.target_system, comp,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, msg_id, interval_us, 0, 0, 0, 0, 0)

    quat = None                      # latest attitude, merged into Imu on each HIGHRES_IMU
    rx = {'HIGHRES_IMU': [0, None, None], 'ATTITUDE': [0, None, None]}   # received: count, t0, t1
    pub_total = {'imu': 0, 'mag': 0}                                     # total published
    win = {'imu': 0, 'mag': 0}                                           # published this 1 s window
    win_t = time.time()
    last_print = 0.0
    t_end = (time.time() + args.secs) if args.secs > 0 else None
    print(f'\nrequested {req_rate:.0f} Hz{" (MAX)" if args.max else ""}; '
          f'{("running " + str(args.secs) + "s") if t_end else "running until Ctrl-C"}...\n')
    try:
        while t_end is None or time.time() < t_end:
            msg = m.recv_match(blocking=True, timeout=1.0)
            now = time.time()

            if msg is not None:
                t = msg.get_type()
                if t in rx:
                    s = rx[t]
                    s[0] += 1
                    if s[1] is None:
                        s[1] = now
                    s[2] = now

                if t == 'ATTITUDE':
                    quat = euler_to_quat(msg.roll, msg.pitch, msg.yaw)
                    if now - last_print > 0.5:
                        last_print = now
                        print(f'  ATT   roll={msg.roll * DEG:7.1f} pitch={msg.pitch * DEG:7.1f} '
                              f'yaw={msg.yaw * DEG:7.1f} deg | rates (rad/s) '
                              f'p={msg.rollspeed:6.3f} q={msg.pitchspeed:6.3f} r={msg.yawspeed:6.3f}')

                elif t == 'HIGHRES_IMU':
                    if publish:
                        stamp = make_stamp(msg.time_usec)
                        imu = Imu()
                        imu.header.stamp = stamp
                        imu.header.frame_id = args.frame_id
                        imu.linear_acceleration.x = float(msg.xacc)   # FRD, m/s^2
                        imu.linear_acceleration.y = float(msg.yacc)
                        imu.linear_acceleration.z = float(msg.zacc)
                        imu.angular_velocity.x = float(msg.xgyro)     # FRD, rad/s (raw gyro)
                        imu.angular_velocity.y = float(msg.ygyro)
                        imu.angular_velocity.z = float(msg.zgyro)
                        if quat is not None:
                            (imu.orientation.x, imu.orientation.y,
                             imu.orientation.z, imu.orientation.w) = quat
                        else:
                            imu.orientation.w = 1.0
                            imu.orientation_covariance[0] = -1.0      # no orientation yet
                        pub_imu.publish(imu)
                        pub_total['imu'] += 1
                        win['imu'] += 1

                        mag = MagneticField()
                        mag.header.stamp = stamp
                        mag.header.frame_id = args.frame_id
                        mag.magnetic_field.x = float(msg.xmag)        # Gauss as reported, FRD
                        mag.magnetic_field.y = float(msg.ymag)
                        mag.magnetic_field.z = float(msg.zmag)
                        pub_mag.publish(mag)
                        pub_total['mag'] += 1
                        win['mag'] += 1

                    if now - last_print > 0.5:
                        last_print = now
                        print(f'  IMU   acc (m/s2) x={msg.xacc:7.3f} y={msg.yacc:7.3f} '
                              f'z={msg.zacc:7.3f} | gyro (rad/s) x={msg.xgyro:6.3f} '
                              f'y={msg.ygyro:6.3f} z={msg.zgyro:6.3f}')
                elif t == 'STATUSTEXT':
                    print(f'  [FC] {msg.text}')

            # once per second, report the live PUBLISH rate of each topic
            dt = now - win_t
            if publish and dt >= 1.0:
                print(f'  [pub] /fc/imu = {win["imu"]/dt:5.1f} Hz   '
                      f'/fc/mag = {win["mag"]/dt:5.1f} Hz')
                win['imu'] = win['mag'] = 0
                win_t = now
    except KeyboardInterrupt:
        pass

    print('\n--- received frequency (from FC) ---')
    for name, (count, t0, t1) in rx.items():
        if count >= 2 and t0 is not None and t1 > t0:
            hz = (count - 1) / (t1 - t0)
            print(f'  {name:12s}: {count:5d} msgs  ->  {hz:6.1f} Hz  (requested {req_rate:.0f} Hz)')
        elif count:
            print(f'  {name:12s}: {count:5d} msgs  (too few to measure rate)')
        else:
            print(f'  {name:12s}:     0 msgs  (!) not received -- is it enabled on this FC?')

    if publish:
        span = max(rx['HIGHRES_IMU'][2] - rx['HIGHRES_IMU'][1], 1e-6) \
            if rx['HIGHRES_IMU'][1] else 0.0
        print('\n--- published frequency (to ROS) ---')
        for topic, key in (('/fc/imu', 'imu'), ('/fc/mag', 'mag')):
            hz = (pub_total[key] / span) if span > 0 else 0.0
            print(f'  {topic:10s}: {pub_total[key]:5d} msgs  ->  {hz:6.1f} Hz (avg over run)')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
