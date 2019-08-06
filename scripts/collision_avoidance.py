#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, Quaternion
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from orm import ORM
from polar_histogram import PolarHistogram
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot
import numpy as np

tf_buffer = None
listener = None

latest_scan = None

server = None

robot_frame_id = ""
look_ahead_distance = 2
distance_converged = 0.3
radius = 0.105

orm = None

pub = None


def scan_callback(msg):
    global latest_scan
    latest_scan = msg


def get_obstacles():
    global latest_scan
    scan = latest_scan

    # FIXME: What should last argument be?
    obstacles = PolarHistogram(len(scan.ranges), float("inf"))

    angle = scan.angle_min
    for r in scan.ranges:
        obstacles.set_range(angle, r)
        angle += scan.angle_increment

    return obstacles


def get_next_setpoint(path):
    global tf_buffer, robot_frame_id, look_ahead_distance, distance_converged
    distance_left = look_ahead_distance

    transform = tf_buffer.lookup_transform(
        robot_frame_id, path.header.frame_id, rospy.Time(0), rospy.Duration(1))
    transform_inverse = tf_buffer.lookup_transform(
        path.header.frame_id, robot_frame_id, rospy.Time(0), rospy.Duration(1))

    point = tf2_geometry_msgs.do_transform_point(
        PointStamped(path.header, path.poses[0].pose.position), transform)

    distance = np.linalg.norm(np.array([[point.point.x, point.point.y]]))

    last_point = PointStamped()

    # print(len(path.poses))

    if 1 == len(path.poses):
        point.header = path.header
        point.point = path.poses[0].pose.position
        if distance < distance_converged:
            del path.poses[0]
        return point
    elif distance <= (look_ahead_distance / 2.0):
        del path.poses[0]
        last_point = point
        distance_left -= distance
    elif distance > look_ahead_distance:
        point.header = path.header
        point.point = path.poses[0].pose.position
        return point

    last_position = np.array([[last_point.point.x, last_point.point.y]])
    for pose in path.poses:
        # FIXME: We assume that it is the same transform...
        point = tf2_geometry_msgs.do_transform_point(
            PointStamped(path.header, pose.pose.position), transform)
        position = np.array([[point.point.x, point.point.y]])
        distance = np.linalg.norm(position - last_position)
        if distance > distance_left:
            last_point.header = path.header
            # Return interpolated between point and last_point
            last_point.point = interpolate(
                last_point.point, point.point, distance_left / distance)
            return tf2_geometry_msgs.do_transform_point(last_point, transform_inverse)

        distance_left -= distance
        last_point = point
        last_position = position

    point.header = path.header
    point.point = path.poses[-1].pose.position
    return point  # Return last element


def interpolate(start, end, t):
    return lerp(start, end, t)


def lerp(start, end, t):
    result = Point()
    result.x = (1.0 - t) * start.x + t * end.x
    result.y = (1.0 - t) * start.y + t * end.y
    result.z = (1.0 - t) * start.z + t * end.z
    return result


def avoid_collision(setpoint):
    global radius
    transform = tf_buffer.lookup_transform(
        robot_frame_id, setpoint.header.frame_id, rospy.Time(0), rospy.Duration(1))
    transform_inverse = tf_buffer.lookup_transform(
        setpoint.header.frame_id, robot_frame_id, rospy.Time(0), rospy.Duration(1))

    point = tf2_geometry_msgs.do_transform_point(setpoint, transform)

    goal = [point.point.x, point.point.y]
    obstacles = get_obstacles()

    control = orm.avoid_collision(goal, obstacles)

    point.point.x = control[0]
    point.point.y = control[1]

    return tf2_geometry_msgs.do_transform_point(point, transform_inverse)


def get_setpoint_callback(req):
    res = GetSetpointResponse()

    if 0 == len(req.path.poses):
        res.setpoint.header = req.path.header
        return res

    res.setpoint = get_next_setpoint(req.path)
    res.setpoint = avoid_collision(res.setpoint)
    pub.publish(res.setpoint)
    res.new_path = req.path
    return res


if __name__ == "__main__":
    rospy.init_node("collision_avoidance")

    # Temp for testing
    pub = rospy.Publisher("collision_free_control_point", PointStamped, queue_size=10)

    # Read parameters
    robot_frame_id = rospy.get_param("~robot_frame_id", "base_link")
    look_ahead_distance = rospy.get_param("~look_ahead_distance", 2.0)
    radius = rospy.get_param("~radius", 0.105)
    security_distance = rospy.get_param("~security_distance", 0.1)
    epsilon = rospy.get_param("~epsilon", 0.1)

    # Create TF buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribers
    rospy.Subscriber("scan", LaserScan, scan_callback)

    # Server
    server = rospy.Service("get_setpoint", GetSetpoint, get_setpoint_callback)

    # Init ORM
    orm = ORM(radius, security_distance, epsilon)

    rospy.spin()
