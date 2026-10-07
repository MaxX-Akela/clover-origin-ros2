# Information: https://clover.coex.tech/en/simple_offboard.html#navigateglobal

import time
import rclpy
from clover import srv, service_proxy
from std_srvs.srv import Trigger
import math

rclpy.init()

get_telemetry = service_proxy('get_telemetry', srv.GetTelemetry)
navigate = service_proxy('navigate', srv.Navigate)
navigate_global = service_proxy('navigate_global', srv.NavigateGlobal)
set_position = service_proxy('set_position', srv.SetPosition)
set_velocity = service_proxy('set_velocity', srv.SetVelocity)
set_attitude = service_proxy('set_attitude', srv.SetAttitude)
set_rates = service_proxy('set_rates', srv.SetRates)
land = service_proxy('land', Trigger)

# https://clover.coex.tech/en/snippets.html#wait_arrival
def wait_arrival(tolerance=0.2):
    while rclpy.ok():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        time.sleep(0.2)

start = get_telemetry()

if math.isnan(start.lat):
    raise Exception('No global position, install and configure GPS sensor: https://clover.coex.tech/gps')

print('Start point global position: lat={}, lon={}'.format(start.lat, start.lon))

print('Take off 3 meters')
navigate(x=0, y=0, z=3, frame_id='body', auto_arm=True)
wait_arrival()

print('Fly 1 arcsecond to the North (approx. 30 meters)')
navigate_global(lat=start.lat+1.0/60/60, lon=start.lon, z=start.z+3, yaw=math.inf, speed=5)
wait_arrival()

print('Fly to home position')
navigate_global(lat=start.lat, lon=start.lon, z=start.z+3, yaw=math.inf, speed=5)
wait_arrival()

print('Land')
land()
