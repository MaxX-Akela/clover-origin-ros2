# Information: https://clover.coex.tech/en/laser.html

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

rclpy.init()
node = rclpy.create_node('process_rangefinder')

def range_callback(msg):
    # Process data from the rangefinder
    print('Rangefinder distance:', msg.range)

# Subscribe to laser rangefinder data
node.create_subscription(Range, 'rangefinder/range', range_callback, qos_profile_sensor_data)

rclpy.spin(node)
