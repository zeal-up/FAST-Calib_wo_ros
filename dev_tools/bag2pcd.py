import rosbag
import sensor_msgs.point_cloud2 as pc2
import os
import argparse
import numpy as np

parser = argparse.ArgumentParser(description='Convert ROS bag to PCD files')
parser.add_argument('--bag_file', type=str, default='your_data.bag', help='Path to the ROS bag file')
parser.add_argument('--pointcloud_topic', type=str, default='/ouster/points',
    help='Topic name of the point cloud, default is "/ouster/points"')
parser.add_argument('--output_dir', type=str, default="",
    help='Output directory for the PCD files (default: current directory)')
parser.add_argument('--output_bin', action='store_true', help='Output binary PCD files instead of text files')

def main():
    args = parser.parse_args()

    # 你的bag文件路径
    bag_file = args.bag_file
    # 你的点云话题名称（可以用 `rosbag info your_data.bag` 查找）
    pointcloud_topic = args.pointcloud_topic

    if args.output_dir == "":
        output_dir = os.path.join(os.path.dirname(bag_file), "..", "pointcloud_pcd")
    else:
        output_dir = args.output_dir

    print(f"Output directory: {output_dir}")
    os.makedirs(output_dir, exist_ok=True)

    # 读取ROS包
    with rosbag.Bag(bag_file, "r") as bag:
        info = bag.get_type_and_topic_info()
        topics = info.topics
        print(f"Topics in '{bag_file}':")
        for topic_name, topic_info in topics.items():
            print(f"- Topic: {topic_name}")
            print(f"  Type: {topic_info.msg_type}")
            print(f"  Count: {topic_info.message_count}")
            print(f"  Frequency: {topic_info.frequency:.2f} Hz")
        count = 0
        msg_count = bag.get_message_count()
        print(msg_count)
        for topic, msg, t in bag.read_messages(topics=[pointcloud_topic]):
            #print(msg.fields)
            # 解析点云数据
            point_data = pc2.read_points(msg, field_names=["x", "y", "z", "intensity", "ring", "t"], skip_nans=True)
            
            # 生成文件名，使用消息的时间戳
            timestamp = str(msg.header.stamp.to_nsec())  # 转换成纳秒级时间戳
            output_file = os.path.join(output_dir, f"{timestamp}.txt")

            # 写入点云数据
            pointcloud = []
            for point in point_data:
                x, y, z, intensity, ring, timestamp_s = point
                timestamp_ns = str(int(timestamp_s * 1e9))  # 转换成纳秒级时间戳
                pointcloud.append((x, y, z, intensity, ring, timestamp_ns))
            
            pointcloud = np.array(pointcloud, dtype=np.float64)
            np.savetxt(output_file, pointcloud)

            count += 1
            progress = f"\rProcessing progress: {count}/{msg_count}"
            print(progress, end="", flush=True)

    print("所有点云数据已保存完成！")

if __name__ == "__main__":
    main()
