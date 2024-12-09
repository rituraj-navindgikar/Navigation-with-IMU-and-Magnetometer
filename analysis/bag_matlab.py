import os
import rosbag2_py
import rclpy.serialization
from imu_msgs.msg import IMUmsg
from gps_msgs.msg import GPSmsg

import scipy.io as sio

def print_message_data(msg, msg_type, topic, imu_data_list, gps_data_list):
    if msg_type == 'imu_msgs/msg/IMUmsg':  # Custom message type
        imu_msg = IMUmsg()
        imu_msg = rclpy.serialization.deserialize_message(msg, IMUmsg)

        # Collect IMU data
        header_data = {}
        if hasattr(imu_msg.header, 'stamp'):
            header_data['timestamp'] = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        if hasattr(imu_msg.header, 'frame_id'):
            header_data['frame_id'] = imu_msg.header.frame_id

        imu_data = {
            'header': header_data,
            'orientation': {
                'x': imu_msg.imu.orientation.x,
                'y': imu_msg.imu.orientation.y,
                'z': imu_msg.imu.orientation.z,
                'w': imu_msg.imu.orientation.w
            },
            'angular_velocity': {
                'x': imu_msg.imu.angular_velocity.x,
                'y': imu_msg.imu.angular_velocity.y,
                'z': imu_msg.imu.angular_velocity.z
            },
            'linear_acceleration': {
                'x': imu_msg.imu.linear_acceleration.x,
                'y': imu_msg.imu.linear_acceleration.y,
                'z': imu_msg.imu.linear_acceleration.z
            },
            'mag_field' : {
                'x': imu_msg.mag_field.magnetic_field.x,
                'y': imu_msg.mag_field.magnetic_field.y,
                'z': imu_msg.mag_field.magnetic_field.z
            },
            'raw_imu' : imu_msg.raw_imu
        }

        # Append to the IMU list
        imu_data_list.append(imu_data)

    ################################################################################################################ GPS MSG
    if msg_type == 'gps_msgs/msg/GPSmsg':  # Custom message type
        gps_msg = GPSmsg()
        gps_msg = rclpy.serialization.deserialize_message(msg, GPSmsg)

        # Collect GPS data
        header_data = {}
        if hasattr(gps_msg.header, 'stamp'):
            header_data['timestamp'] = gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec * 1e-9
        if hasattr(gps_msg.header, 'frame_id'):
            header_data['frame_id'] = gps_msg.header.frame_id

        gps_data = {
            'header' : header_data,
            'latitude' : gps_msg.latitude,
            'longitude' : gps_msg.longitude,
            'altitude' : gps_msg.altitude,
            'utm_easting' : gps_msg.utm_easting,
            'utm_northing' : gps_msg.utm_northing,
            'zone' : gps_msg.zone,
            'letter' : gps_msg.letter,
        }

        # Append to the GPS list
        gps_data_list.append(gps_data)

def read_rosbag_and_save(bag_file, output_file):
    # Ensure the directory for output_file exists
    output_dir = os.path.dirname(output_file)
    if not os.path.exists(output_dir):
        print(f"Directory {output_dir} does not exist. Creating it.")
        os.makedirs(output_dir)

    # Check if the .mat file already exists
    if not os.path.exists(output_file):
        print(f"{output_file} does not exist. It will be created.")

    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()

    # Print topics found in the bag file
    print("Topics found in bag file:")
    for topic_info in topics:
        print(f"- {topic_info.name}: {topic_info.type}")

    imu_data_list = []
    gps_data_list = []
    count = 0

    # Iterate over all messages in the bag and collect data
    while reader.has_next():
        (topic, msg, t) = reader.read_next()

        # Find the message type for the topic
        msg_type = None
        for topic_info in topics:
            if topic_info.name == topic:
                msg_type = topic_info.type
                break

        # Collect the message data for saving in .mat file
        if msg_type:
            print_message_data(msg, msg_type, topic, imu_data_list, gps_data_list)
        else:
            print(f"Unknown message type for topic: {topic}")

        count += 1

    print(f"Total messages processed: {count}")

    # Save the collected data into a .mat file
    print(f"Saving data to {output_file}")
    sio.savemat(output_file, {'imu_data': imu_data_list, 'gps_data': gps_data_list})

if __name__ == "__main__":
    # Path to your ROS2 bag file
    bag_file = '/home/rituraj/Northeastern_University_Gitlab/lab4_ws/data/data_complete/data_driving/data_driving_0.db3'
    output_file = '/home/rituraj/Northeastern_University_Gitlab/lab4_ws/data/matlab_data/data_driving.mat'

    # Read and save all messages into a .mat file
    read_rosbag_and_save(bag_file, output_file)
