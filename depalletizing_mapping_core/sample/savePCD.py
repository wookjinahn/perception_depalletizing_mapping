import rospy
import sys
import datetime

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(data):
    write_pcd(data)


def write_pcd(data):
    pcd_data = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))
    POINT = str(len(pcd_data)) + "\n"

    now = datetime.datetime.now()
    filePath = "/home/wj/Desktop/Data/output_data/" + str(now.strftime("%H%M%S")) + ".pcd"

    f = open(filePath, "w")
    # pcd file format
    f.write("VERSION\n")
    f.write("FIELDS x y z\n")
    f.write("SIZE 4 4 4\n")
    f.write("TYPE F F F\n")
    f.write("COUNT 1 1 1\n")
    f.write("WIDTH 1\n")
    f.write("HEIGHT " + POINT)
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS " + POINT)
    f.write("DATA ascii\n")
    # point data
    for point in pcd_data:    # , skip_nans=True
        f.write(gen)
    f.close()

def main():
    rospy.init_node("get_pointcloud", anonymous=True)

    # rospy.Subscriber("/d435/depth/pointcloud", PointCloud2, callback)
    # rospy.wait_for_message("/d435/depth/pointcloud", PointCloud2)

    rospy.Subscriber("/d435/depth/color/points", PointCloud2, callback)
    rospy.wait_for_message("/d435/depth/color/points", PointCloud2)

if __name__ == '__main__':
    main()