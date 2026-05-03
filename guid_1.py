# Однодроновий CLI-скрипт для тестового виконання маршруту без GUI.
# Він використовує той самий DroneClient, що й GUI, тому корисний для перевірки
# MAVLink-з'єднання, GUIDED mode, waypoint-навігації та посадки окремо від Qt.
import argparse
import time

from drone_client import DroneClient, distance_m, load_route


# Зчитує CLI-аргументи для одного дрона.
# Повертає port, шлях до route JSON і прапорець --land.
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="udpin:0.0.0.0:14550")
    parser.add_argument("--route", required=True, help="Path to route JSON file")
    parser.add_argument("--land", action="store_true", help="Land after route completed")
    return parser.parse_args()


# Основний сценарій польоту: завантажити маршрут, підключитись,
# злетіти, пройти всі точки і опційно сісти.
def main():
    args = parse_args()
    route = load_route(args.route)

    points = route["points"]
    # Якщо takeoff_alt не задано у JSON, беремо висоту першої точки маршруту.
    takeoff_alt = route.get("takeoff_alt", points[0]["alt"])
    reach_radius_m = route.get("reach_radius_m", 1.5)

    client = DroneClient(args.port)
    client.connect()

    # Для керування set_position_target потрібен GUIDED режим і armed стан.
    client.set_mode("GUIDED")
    client.arm()

    client.takeoff(takeoff_alt)
    client.wait_until_altitude(takeoff_alt)

    last_print = 0
    last_current_print = 0

    for index, point in enumerate(points):
        # Кожна точка маршруту містить GPS координати і цільову висоту.
        target_lat = point["lat"]
        target_lon = point["lon"]
        target_alt = point["alt"]

        print(f"\nGoing to point {index + 1}/{len(points)}")
        client.goto_point(target_lat, target_lon, target_alt)

        point_start = time.time()
        point_timeout = route.get("point_timeout_sec", 60)
        alt_tolerance_m = route.get("alt_tolerance_m", 1.0)

        last_print = 0

        while True:
            # У CLI-скрипті читаємо MAVLink блокуюче, бо іншого GUI-потоку немає.
            msg = client.recv_match(blocking=True, timeout=2)

            if msg is None:
                print("No message...")
                continue

            client.update_telemetry(msg)

            now = time.time()

            if now - point_start > point_timeout:
                print(f"Timeout: point {index + 1} was not reached in {point_timeout} seconds")
                break

            # Повний snapshot телеметрії друкується не частіше разу на секунду.
            if now - last_print >= 1:
                client.print_telemetry()
                last_print = now

            if client.telemetry["position"] is not None:
                current_lat = client.telemetry["position"]["lat"]
                current_lon = client.telemetry["position"]["lon"]
                current_alt = client.telemetry["position"]["relative_alt_m"]

                dist = distance_m(
                    current_lat,
                    current_lon,
                    target_lat,
                    target_lon
                )

                alt_error = abs(current_alt - target_alt)

                # Короткий прогрес-лог показує відстань і похибку висоти до waypoint.
                if now - last_current_print >= 1:
                    print(
                        f"Current: lat={current_lat:.7f}, lon={current_lon:.7f}, "
                        f"alt={current_alt:.2f} m | "
                        f"distance to point {index + 1}/{len(points)}={dist:.2f} m | "
                        f"alt_error={alt_error:.2f} m"
                    )
                    last_current_print = now

                # Точка вважається досягнутою, коли виконані умови по відстані і висоті.
                if dist < reach_radius_m and alt_error < alt_tolerance_m:
                    print(f"Point {index + 1} reached")
                    break

    print("\nRoute completed")

    if args.land:
        # У цьому простому CLI-скрипті лише надсилається команда LAND без окремого wait loop.
        client.land()


if __name__ == "__main__":
    main()
