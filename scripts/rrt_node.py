#!/usr/bin/env python2
from sensor import Sensor
from math import exp, fabs, atan2, fmod, pi, hypot
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from sets import Set


def raytrace(start, end):
    """Returns all cells in the grid map that has been traversed
    from start to end, including start and excluding end.
    start = (x, y) grid map index
    end = (x, y) grid map index
    """
    (start_x, start_y) = start
    (end_x, end_y) = end
    x = start_x
    y = start_y
    (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
    n = dx + dy
    x_inc = 1
    if end_x <= start_x:
        x_inc = -1
    y_inc = 1
    if end_y <= start_y:
        y_inc = -1
    error = dx - dy
    dx *= 2
    dy *= 2

    traversed = []
    for _ in range(0, int(n)):
        traversed.append((int(x), int(y)))

        if error > 0:
            x += x_inc
            error -= dy
        else:
            if error == 0:
                traversed.append((int(x + x_inc), int(y)))
            y += y_inc
            error += dx

    return traversed


class RRTNode:
    def __init__(self, node_id, position):
        self._node_id = node_id

        self._position = position

        self._parent = None
        self._children = []

        self._cost = None
        self.__cost_parent = None

        self._score = None
        self.__score_parent = None

        self._gain = None

    def __del__(self):
        self.set_parent(None)

        for child in self._children:
            child.set_parent(None)

        del self._children[:]

    def get_id(self):
        return self._node_id

    def set_parent(self, new_parent):
        if self.has_parent():
            self._parent.erase_child(self)

        if new_parent is not None:
            new_parent.add_child(self)

        self._parent = new_parent

    def get_parent(self):
        return self._parent

    def has_parent(self):
        return self._parent is not None

    def add_child(self, new_child):
        self._children.append(new_child)

    def erase_child(self, child):
        self._children.remove(child)

    def get_children(self):
        return self._children

    def get_num_children(self):
        return len(self._children)

    def has_children(self):
        return 0 < len(self._children)

    def cost(self):
        if not self.has_parent():
            self._cost = 0
            return self._cost

        if self.__cost_parent is not None and self.__cost_parent == self._parent:
            return self._cost

        self._cost = self.cost_with_parent(self.get_parent())

        self.__cost_parent = self._parent

        return self._cost

    def cost_with_parent(self, parent):
        return parent.cost() + self.distance(parent)

    def score(self, grid_map, sensor, l):
        if not self.has_parent():
            self._score = 0
            return self._score

        if self.__score_parent is not None and self.__score_parent == self._parent:
            return self._score

        self._score = self.score_with_parent(
            self.get_parent(), grid_map, sensor, l)

        self.__score_parent = self._parent

        return self._score

    def score_with_parent(self, parent, grid_map, sensor, l):
        return parent.score(grid_map, sensor, l) + self.gain(grid_map, sensor) * \
            exp(-l * self.cost_with_parent(parent))

    def constrain_angle(self, angle):
        angle = fmod(angle + pi, 2 * pi)
        if angle < 0:
            angle += 2 * pi
        return angle - pi

    def gain(self, grid_map, sensor):
        if self._gain is not None:
            return self._gain

        res = grid_map.info.resolution

        start_x = int((self._position[0][0] -
                       grid_map.info.origin.position.x) / res)
        start_y = int((self._position[0][1] -
                       grid_map.info.origin.position.y) / res)

        min_x = int(((self._position[0][0] - sensor.get_range_max()) -
                     grid_map.info.origin.position.x) / res)
        max_x = int(((self._position[0][0] + sensor.get_range_max()) -
                     grid_map.info.origin.position.x) / res)

        min_y = int(((self._position[0][1] - sensor.get_range_max()) -
                     grid_map.info.origin.position.y) / res)
        max_y = int(((self._position[0][1] + sensor.get_range_max()) -
                     grid_map.info.origin.position.y) / res)

        min_x = max(0, min(grid_map.info.width-1, min_x))
        max_x = max(0, min(grid_map.info.width-1, max_x))

        min_y = max(0, min(grid_map.info.height-1, min_y))
        max_y = max(0, min(grid_map.info.height-1, max_y))

        unknown_space = Set()
        max_gain = Set()

        for y in [min_y, max_y]:
            for x in range(min_x, max_x, 50):
                # Perform ray tracing
                t = raytrace((start_x, start_y), (x, y))
                max_gain.update(t)

                for (t_x, t_y) in t:
                    value = grid_map.data[t_y * grid_map.info.width + t_x]
                    if -1 == value:
                        unknown_space.add((t_x, t_y))
                    elif 100 == value:
                        break

        for x in [min_x, max_x]:
            for y in range(min_y, max_y, 50):
                # Perform ray tracing
                t = raytrace((start_x, start_y), (x, y))
                max_gain.update(t)

                for (t_x, t_y) in t:
                    value = grid_map.data[t_y * grid_map.info.width + t_x]
                    if -1 == value:
                        unknown_space.add((t_x, t_y))
                    elif 100 == value:
                        break

        self._gain = float(len(unknown_space)) / float(len(max_gain) / 10)

        return self._gain

    def gain_along_path(self, grid_map, sensor):
        total_gain = self.gain(grid_map, sensor)
        current_node = self
        while current_node.has_parent():
            current_node = current_node.get_parent()
            total_gain += current_node.gain(grid_map, sensor)
        return total_gain

    def rad_bounded(self, value):
        value = fmod(value + pi, 2 * pi)
        if 0 > value:
            value += 2 * pi
        return value - pi

    def distance(self, node):
        if not node.has_parent():
            return np.linalg.norm(node.get_position() - self._position)
        else:
            node_position = node.get_position()
            node_parent_position = node.get_parent().get_position()
            node_yaw = atan2(node_position[0][1] - node_parent_position[0]
                             [1], node_position[0][0] - node_parent_position[0][0])
            yaw = atan2(self._position[0][1] - node_position[0]
                        [1], self._position[0][0] - node_position[0][0])
            yaw_diff = self.rad_bounded(yaw - node_yaw)
            return hypot(np.linalg.norm(node.get_position() - self._position), yaw_diff)

    def set_position(self, position):
        self._position = position

    def get_position(self):
        return self._position

    def get_path(self, frame_id):
        path = Path()

        current_node = self
        while current_node is not None:
            if not current_node.has_parent():
                break

            position = current_node.get_position()

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.orientation.w = 1
            pose.pose.position.x = position[0][0]
            pose.pose.position.y = position[0][1]

            parent_position = current_node.get_parent().get_position()

            q = quaternion_from_euler(0, 0, atan2(
                position[0][1] - parent_position[0][1], position[0][0] - parent_position[0][0]))

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.insert(0, pose)
            current_node = current_node.get_parent()
        return path
