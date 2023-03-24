import roslibpy

client = roslibpy.Ros(host='10.135.4.171', port=9001)
client.run()
bot = 'autobot10'
map = 'zone1' # 'zone1' 'zone2' 'zone3' 'full'
map_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/map_state', 'std_msgs/String')
begin_state_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/begin_state', 'std_msgs/Int32')
trajectory_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/trajectory', 'std_msgs/Int32MultiArray')
start_pub = roslibpy.Topic(client, f'/{bot}/lane_controller_node/start', 'std_msgs/Bool')
for i in range(1000):

    map_publisher.publish(roslibpy.Message({'data': map}))
    begin_state_publisher.publish(roslibpy.Message({'data': 2}))
    trajectory_publisher.publish(roslibpy.Message({'data': [3, 4]}))
    start_pub.publish(roslibpy.Message({'data': True}))

client.terminate()
