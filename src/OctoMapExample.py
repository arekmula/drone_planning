#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap


# def callback(cloud):
#    assert isinstance(cloud, PointCloud2)
#    gen = point_cloud2.read_points(cloud)
#    print(type(gen))
#    for p in gen:
#       print("Punkt", p)

def callbackOctomap(ocMap):
    assert isinstance(ocMap, Octomap)
    # print(ocMap.data)
    ocDes = ocMap.serialize(ocMap)
    print(ocDes)


def mapListener():
    # create node
    rospy.init_node('octomap_listener', anonymous=True)

    # subscribe /octomap_point_cloud_centers
    # rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback)
    rospy.Subscriber("/octomap_full", Octomap, callbackOctomap)
    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    mapListener()
