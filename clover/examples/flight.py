# Information: https://clover.coex.tech/programming

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

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
time.sleep(5)

print('Fly forward 1 m')
navigate(x=1, y=0, z=0, frame_id='body')

# Wait for 5 seconds
time.sleep(5)

print('Perform landing')
land()
