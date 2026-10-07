# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import time
import rclpy
from clover import srv, service_proxy
from std_srvs.srv import Trigger

rclpy.init()

get_telemetry = service_proxy('get_telemetry', srv.GetTelemetry)
navigate = service_proxy('navigate', srv.Navigate)
navigate_global = service_proxy('navigate_global', srv.NavigateGlobal)
set_position = service_proxy('set_position', srv.SetPosition)
set_velocity = service_proxy('set_velocity', srv.SetVelocity)
set_attitude = service_proxy('set_attitude', srv.SetAttitude)
set_rates = service_proxy('set_rates', srv.SetRates)
land = service_proxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while rclpy.ok():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        time.sleep(0.2)

print('Take off 1 meter')
navigate_wait(z=1, frame_id='body', auto_arm=True)

print('Fly forward 1 m')
navigate_wait(x=1, frame_id='body')

print('Land')
land()
