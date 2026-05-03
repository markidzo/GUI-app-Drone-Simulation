from pymavlink import mavutil
import argparse
import time
import math
import json


# lat = -35.3650000
# lon = 149.1660000
# alt = 10


telemetry = {
    "mode": None,
    "armed": None,
    "battery": None,
    "attitude": None,
    "position": None,
    "local": None,
    "imu": None,
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="udpin:0.0.0.0:14550")
    parser.add_argument("--route", required=True, help="Path to route JSON file")
    parser.add_argument("--land", action="store_true", help="Land after route completed")
    return parser.parse_args()


def distance_m(lat1, lon1, lat2, lon2):
    r = 6371000
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def request_message_interval(mav, message_id, hz):
    interval_us = int(1_000_000 / hz)

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


def request_telemetry_streams(mav):
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5)
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 5)


def update_telemetry(msg):
    if msg is None:
        return False

    msg_type = msg.get_type()

    if msg_type == "HEARTBEAT":
        telemetry["mode"] = mavutil.mode_string_v10(msg)
        telemetry["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    elif msg_type == "SYS_STATUS":
        telemetry["battery"] = {
            "voltage_v": msg.voltage_battery / 1000.0,
            "current_a": msg.current_battery / 100.0,
            "remaining_percent": msg.battery_remaining,
        }

    elif msg_type == "ATTITUDE":
        telemetry["attitude"] = {
            "roll": msg.roll,
            "pitch": msg.pitch,
            "yaw": msg.yaw,
            "rollspeed": msg.rollspeed,
            "pitchspeed": msg.pitchspeed,
            "yawspeed": msg.yawspeed,
        }

    elif msg_type == "GLOBAL_POSITION_INT":
        telemetry["position"] = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "relative_alt_m": msg.relative_alt / 1000.0,
            "vx": msg.vx / 100.0,
            "vy": msg.vy / 100.0,
            "vz": msg.vz / 100.0,
        }

    elif msg_type == "LOCAL_POSITION_NED":
        telemetry["local"] = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "vx": msg.vx,
            "vy": msg.vy,
            "vz": msg.vz,
        }

    elif msg_type == "RAW_IMU":
        telemetry["imu"] = {
            "xacc": msg.xacc,
            "yacc": msg.yacc,
            "zacc": msg.zacc,
            "xgyro": msg.xgyro,
            "ygyro": msg.ygyro,
            "zgyro": msg.zgyro,
            "xmag": msg.xmag,
            "ymag": msg.ymag,
            "zmag": msg.zmag,
        }

    return True


def print_telemetry():
    print("\n--- TELEMETRY ---")
    print("Mode:", telemetry["mode"])
    print("Armed:", telemetry["armed"])
    print("Battery:", telemetry["battery"])
    print("Attitude:", telemetry["attitude"])
    print("Position:", telemetry["position"])
    print("Local:", telemetry["local"])
    print("IMU:", telemetry["imu"])


def set_mode(mav, mode):
    mode_id = mav.mode_mapping()[mode]
    mav.set_mode(mode_id)

    while True:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True)
        update_telemetry(msg)

        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode:
            print(f"Mode changed to {mode}")
            break


def load_route(path):
    with open(path, "r", encoding="utf-8") as file:
        route = json.load(file)

    if "points" not in route or len(route["points"]) == 0:
        raise ValueError("JSON route must contain non-empty 'points' list")

    return route


def wait_until_altitude(mav, target_alt, tolerance=0.7, timeout=30):
    print(f"Waiting until altitude {target_alt} m...")

    start = time.time()
    last_print = 0   # ← ОБОВʼЯЗКОВО ТУТ

    while time.time() - start < timeout:
        msg = mav.recv_match(blocking=True, timeout=1)

        if msg is None:
            print("No message while waiting altitude...")
            continue

        update_telemetry(msg)

        if telemetry["position"] is None:
            continue

        current_alt = telemetry["position"]["relative_alt_m"]

        now = time.time()

        if now - last_print >= 1:
            print(f"Current altitude: {current_alt:.2f} m")
            last_print = now

        if abs(current_alt - target_alt) <= tolerance:
            print("Target altitude reached")
            return True

    raise TimeoutError("Drone did not reach target altitude")


def arm(mav, timeout=15):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    print("Arm command sent, waiting...")

    start = time.time()

    while time.time() - start < timeout:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is None:
            continue

        update_telemetry(msg)

        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        if armed:
            print("Drone armed")
            return True

    raise TimeoutError("Drone did not arm. Check pre-arm errors in SITL/MAVProxy.")


def takeoff(mav, alt):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    print(f"Takeoff command sent: {alt} m")


def goto_point(mav, lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    print(f"Target point sent: lat={lat}, lon={lon}, alt={alt}")


def land(mav):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0, 0
    )
    print("Land command sent")


def main():
    args = parse_args()
    route = load_route(args.route)

    points = route["points"]
    takeoff_alt = route.get("takeoff_alt", points[0]["alt"])
    reach_radius_m = route.get("reach_radius_m", 1.5)

    print(f"Connecting to {args.port}...")
    mav = mavutil.mavlink_connection(args.port)

    print("Waiting for heartbeat...")
    mav.wait_heartbeat()
    print(f"Heartbeat OK: system={mav.target_system}, component={mav.target_component}")

    request_telemetry_streams(mav)

    set_mode(mav, "GUIDED")
    arm(mav)

    takeoff(mav, takeoff_alt)
    wait_until_altitude(mav, takeoff_alt)

    last_print = 0
    last_current_print = 0

    for index, point in enumerate(points):
        target_lat = point["lat"]
        target_lon = point["lon"]
        target_alt = point["alt"]

        print(f"\nGoing to point {index + 1}/{len(points)}")
        goto_point(mav, target_lat, target_lon, target_alt)

        point_start = time.time()
        point_timeout = route.get("point_timeout_sec", 60)  # TODO change the logic
        alt_tolerance_m = route.get("alt_tolerance_m", 1.0)

        last_print = 0

        while True:
            msg = mav.recv_match(blocking=True, timeout=2)

            if msg is None:
                print("No message...")
                continue

            update_telemetry(msg)

            now = time.time()

            if now - point_start > point_timeout:  # TODO change the logic
                print(f"Timeout: point {index + 1} was not reached in {point_timeout} seconds")
                break

            if now - last_print >= 1:
                print_telemetry()
                last_print = now

            if telemetry["position"] is not None:
                current_lat = telemetry["position"]["lat"]
                current_lon = telemetry["position"]["lon"]
                current_alt = telemetry["position"]["relative_alt_m"]

                dist = distance_m(
                    current_lat,
                    current_lon,
                    target_lat,
                    target_lon
                )

                alt_error = abs(current_alt - target_alt)   # <-- ВАЖЛИВО: тут

                if now - last_current_print >= 1:
                    print(
                        f"Current: lat={current_lat:.7f}, lon={current_lon:.7f}, "
                        f"alt={current_alt:.2f} m | "
                        f"distance to point {index + 1}/{len(points)}={dist:.2f} m | "
                        f"alt_error={alt_error:.2f} m"
                    )
                    last_current_print = now

                if dist < reach_radius_m and alt_error < alt_tolerance_m:
                    print(f"Point {index + 1} reached")
                    break
                
                current_lat = telemetry["position"]["lat"]
                current_lon = telemetry["position"]["lon"]
                current_alt = telemetry["position"]["relative_alt_m"]

    print("\nRoute completed")

    if args.land:
        land(mav)    
    

if __name__ == "__main__":
    main()