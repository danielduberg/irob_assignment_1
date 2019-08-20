#!/usr/bin/env python2
import rospy
import actionlib
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
import irob_assignment_1.msg
from sensor import Sensor
from rrt import RRT
import numpy as np
import tf2_ros
from math import pi, atan2
from tf.transformations import quaternion_from_euler

action_server = None

tf_buffer = None
listener = None

grid_map = None

robot_frame_id = ""
max_nodes = 1000
extension_range = 1.0
radius = 0.105

sensor = None
l = 0.25

tree_pub = None
best_branch_pub = None


def get_next_goal(goal):
    global action_server, grid_map, sensor
    feedback = irob_assignment_1.msg.GetNextGoalFeedback()

    feedback.gain = 0
    feedback.path = Path()

    while grid_map is None:
        if action_server.is_preempt_requested() or rospy.is_shutdown():
            rospy.loginfo("Explorer: Preempted")
            action_server.set_preempted()
            return
        rospy.loginfo("Explorer: No map recieved yet!")
        rospy.sleep(1)

    bbx_min = [grid_map.info.origin.position.x,
               grid_map.info.origin.position.y]
    bbx_max = [bbx_min[0] + (grid_map.info.width * grid_map.info.resolution),
               bbx_min[1] + (grid_map.info.height * grid_map.info.resolution)]

    tf_buffer.can_transform(grid_map.header.frame_id,
                            robot_frame_id, rospy.Time(0), rospy.Duration(10))

    # Get root position based on TF
    robot_pose = tf_buffer.lookup_transform(
        grid_map.header.frame_id, robot_frame_id, rospy.Time(0))
    root_position = np.array(
        [[robot_pose.transform.translation.x, robot_pose.transform.translation.y]])

    tree = RRT(root_position, bbx_min, bbx_max, radius, extension_range)

    clear_published_tree()

    # Clear path
    path = Path()
    path.header.frame_id = grid_map.header.frame_id
    path.header.stamp = rospy.Time.now()
    best_branch_pub.publish(path)

    success = True
    i = 0
    while i < max_nodes and not rospy.is_shutdown():
        # Check if preempt has been requested by the client
        if action_server.is_preempt_requested() or rospy.is_shutdown():
            rospy.loginfo("Explorer: Preempted")
            action_server.set_preempted()
            success = False
            break

        node = tree.expand_tree(grid_map, sensor, l)
        if node is None:
            continue
        i += 1

        gain = node.gain_along_path(grid_map, sensor)
        if gain > feedback.gain:
            publish_tree(tree)
            # Publish this new better path as feedback
            feedback.gain = gain
            rospy.loginfo("Explorer: Current max gain %s", round(feedback.gain, 2))
            rospy.loginfo("Explorer: %s nodes of %s, %s remaining", i, max_nodes, max_nodes - i)
            feedback.path = node.get_path(grid_map.header.frame_id)
            feedback.path.header.frame_id = grid_map.header.frame_id
            feedback.path.header.stamp = rospy.Time.now()
            best_branch_pub.publish(feedback.path)
            action_server.publish_feedback(feedback)
        elif 0 == i % 100:
            publish_tree(tree)

    if success:
        result = irob_assignment_1.msg.GetNextGoalResult()
        result.gain = feedback.gain
        result.path = feedback.path
        result.path.header.frame_id = grid_map.header.frame_id
        result.path.header.stamp = rospy.Time.now()
        best_branch_pub.publish(result.path)
        action_server.set_succeeded(result)


def clear_published_tree():
    markers = MarkerArray()
    marker = Marker()
    marker.header.frame_id = grid_map.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.DELETEALL
    marker.ns = "edges"
    markers.markers.append(marker)
    tree_pub.publish(markers)


def publish_tree(tree):
    global grid_map, sensor
    markers = MarkerArray()
    current_id = 0
    for node in tree.get_nodes(grid_map):
        if node.has_parent():
            marker = Marker()
            marker.header.frame_id = grid_map.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.ADD
            marker.id = current_id
            current_id += 1
            marker.type = Marker.ARROW
            marker.ns = "edges"
            position = node.get_position()
            parent_position = node.get_parent().get_position()
            marker.pose.position.x = parent_position[0][0]
            marker.pose.position.y = parent_position[0][1]
            marker.pose.position.z = 0
            q = quaternion_from_euler(0, 0, atan2(
                position[0][1] - parent_position[0][1], position[0][0] - parent_position[0][0]))
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.scale.x = np.linalg.norm(position - parent_position)
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.a = 1.0
            marker.color.r = node.gain(grid_map, sensor)
            marker.color.g = 0
            marker.color.b = (1.0 - marker.color.r)
            marker.lifetime = rospy.Duration(0)  # Forever
            marker.frame_locked = False
            markers.markers.append(marker)
    tree_pub.publish(markers)


def map_callback(msg):
    global grid_map
    grid_map = msg


if __name__ == "__main__":
    rospy.init_node("explorer")

    # Temp for testing
    tree_pub = rospy.Publisher("tree", MarkerArray, queue_size=10)
    best_branch_pub = rospy.Publisher("best_branch", Path, queue_size=10)

    # Create TF buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Read parameters
    robot_frame_id = rospy.get_param("~robot_frame_id", "base_link")
    max_nodes = rospy.get_param("~max_nodes", max_nodes)
    extension_range = rospy.get_param("~extension_range", extension_range)
    radius = rospy.get_param("~radius", radius)
    fov = rospy.get_param("~fov", 2 * pi)
    range_min = rospy.get_param("~range_min", 0.12)
    range_max = rospy.get_param("~range_max", 10)  # 3.5
    sensor = Sensor(fov, range_min, range_max)

    # Subscribers
    rospy.Subscriber("costmap_node/costmap/costmap",
                     OccupancyGrid, map_callback)

    # Action servers
    action_server = actionlib.SimpleActionServer(
        "get_next_goal", irob_assignment_1.msg.GetNextGoalAction, execute_cb=get_next_goal, auto_start=False)
    action_server.start()

    rospy.spin()
