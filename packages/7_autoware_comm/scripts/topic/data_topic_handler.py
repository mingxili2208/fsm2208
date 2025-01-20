import rosbag2_py
import pandas as pd
from rclpy.serialization import deserialize_message
from autoware_auto_control_msgs.msg import AckermannControlCommand


bag_path = '/home/cityu-fsm-lab-carla/lmx/Data/topic_record/control_cmd_bag_5_1/control_cmd_bag_5_1_0.db3'  # 替换成你的 bag 文件路径
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# 创建一个空的列表来存储数据
data = []

while reader.has_next():
    #print(reader.read_next())
    (topic, msg, t) = reader.read_next()
    if topic == '/control/command/control_cmd':
        # 提取 steering_tire_angle 和 speed
        msg_type = AckermannControlCommand
        msg = deserialize_message(msg, msg_type)
        steering_angle = msg.lateral.steering_tire_angle
        speed = msg.longitudinal.speed

        # 将数据添加到列表中
        data.append({'timestamp': t, 'steering_angle': steering_angle, 'speed': speed})

# 将数据转换为 Pandas DataFrame
df = pd.DataFrame(data)

speed_not_zero = df['speed'] != 0

df = df[speed_not_zero]

df.to_csv('~/lmx/Data/topic_record/control_cmd_bag_5_1/control_cmd_bag_5_1_modified.csv', index=False)

print(df)

#reader.close()