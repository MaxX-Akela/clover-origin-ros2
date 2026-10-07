# Information: https://clover.coex.tech/en/simple_offboard.html#gettelemetry

import rclpy
from clover import srv, service_proxy

rclpy.init()

get_telemetry = service_proxy('get_telemetry', srv.GetTelemetry)

# Print drone's state
print(get_telemetry())
