import roslibpy
import time
import argparse


client = roslibpy.Ros(host='10.135.4.101', port=9001)
client.run()
bot = 'autobot02'
map_name = 'zone1' # 'zone1' 'zone2' 'zone3' 'full'


axes= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
buttons[7] = 1

map_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/map_state', 'std_msgs/String')
begin_state_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/begin_state', 'std_msgs/Int32')
trajectory_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/trajectory', 'std_msgs/Int32MultiArray')
start_pub = roslibpy.Topic(client, f'/{bot}/lane_controller_node/start', 'std_msgs/Bool')
joy_pub = roslibpy.Topic(client, f'/{bot}/joy', 'sensor_msgs/Joy')

map_publisher.publish(roslibpy.Message({'data': map_name}))
begin_state_publisher.publish(roslibpy.Message({'data': 1}))
trajectory_publisher.publish(roslibpy.Message({'data': [2, 3]}))
start_pub.publish(roslibpy.Message({'data': True}))
joy_pub.publish(roslibpy.Message({'axes': axes, 'buttons':buttons }))
time.sleep(1)

client.terminate()
