#!/usr/bin/env python2

from polar_histogram import PolarHistogram, isfinite, rad_bounded
from math import hypot, atan2, cos, sin, pi, fabs, isnan, isinf, fmod


class ORM:
    def __init__(self, radius, security_distance, epsilon):
        self._radius = radius
        self._security_distance = security_distance
        self._epsilon = epsilon

    def avoid_collision(self, goal, obstacles):
        subgoal = self.subgoal_selector(goal, obstacles)

        if subgoal[0] == 0 and subgoal[1] == 0:
            return subgoal

        return self.motion_computation(subgoal, obstacles)

    def subgoal_selector(self, goal, obstacles):
        if self.is_path_clear(goal, obstacles):
            # We can go straight towards goal
            return goal

        # We have to find a subgoal
        goal_direction = atan2(goal[1], goal[0])

        subgoal = goal
        d = obstacles.bucket_size()
        while pi > d:
            for i in [-1, 1]:
                current_direction = goal_direction + (i * d)
                previous_direction = goal_direction + \
                    (i * (d - obstacles.bucket_size()))
                if self.is_subgoal(obstacles, current_direction, previous_direction, subgoal):
                    return subgoal
            
            d += obstacles.bucket_size()

        return [0, 0]

    def is_subgoal(self, obstacles, direction_1, direction_2, subgoal):
        # TODO: Maybe check for NaN? Or assume that it is never NaN?
        if not obstacles.isfinite(direction_1) and not obstacles.isfinite(direction_2):
            # Not a subgoal because it is only open space
            return False, subgoal

        if not obstacles.isfinite(direction_1):
            # This makes it so direction_1 is always finite
            temp = direction_1
            direction_1 = direction_2
            direction_2 = temp

        temp_subgoal = [0, 0]

        if not obstacles.isfinite(direction_2):
            # Found a potential subgoal at the edge of an obstacle
            temp_subgoal[0] = obstacles.get_range(
                direction_1) * cos(direction_2)
            temp_subgoal[1] = obstacles.get_range(
                direction_1) * sin(direction_2)
        else:
            # We know that both are finite now
            # TODO: Maybe save the points?
            p_1 = obstacles.get_point(direction_1)
            p_2 = obstacles.get_point(direction_2)
            if hypot(p_2[0] - p_1[0], p_2[1] - p_1[1]) > 2 * self._radius:
                # Found a potential subgoal between two obstacles
                temp_subgoal[0] = (p_1[0] + p_2[0]) / 2
                temp_subgoal[1] = (p_1[1] + p_2[1]) / 2
            else:
                return False, subgoal

        if self.is_path_clear(temp_subgoal, obstacles):
            return True, temp_subgoal

        return False, subgoal

    def motion_computation(self, goal, obstacles):
        goal_direction = atan2(goal[1], goal[0])

        (left_bound, right_bound) = self.get_bounds(goal, obstacles)

        diff_left = rad_bounded(left_bound - goal_direction)
        diff_right = rad_bounded(right_bound - goal_direction)

        if diff_left < 0 and diff_right > 0:
            # Move straight towards goal
            return goal

        direction = 0
        if diff_right > 0 and diff_right > diff_left:
            # Move towards left bound
            direction = left_bound
        elif diff_left < 0 and diff_right > diff_left:
            # Move towards right bound
            direction = right_bound
        else:
            # Move to the middle of left and right bound
            left_temp = [cos(left_bound), sin(left_bound)]
            right_temp = [cos(right_bound), sin(right_bound)]
            middle = [(left_temp[0] + right_temp[0]) / 2,
                      (left_temp[1] + right_temp[1]) / 2]
            direction = atan2(middle[1], middle[0])

        distance = hypot(goal[0], goal[1])
        return [distance * cos(direction), distance * sin(direction)]

    def get_bounds(self, goal, obstacles):
        # TODO: Should we only care about the obstacles that are in the way?

        max_distance = hypot(goal[0], goal[1]) + \
            self._radius  # TODO: + self._epsilon also?

        goal_direction = atan2(goal[1], goal[0])

        left_bound = rad_bounded(goal_direction - (pi - 0.001))
        right_bound = rad_bounded(goal_direction + (pi - 0.001))

        for obstacle in obstacles:
            if not obstacle.isfinite() or obstacle.get_range() > max_distance:
                continue

            # TODO: atan or atan2?
            alpha = fabs(
                atan2(self._radius + self._security_distance, obstacle.get_range()))

            beta = 0
            if obstacle.get_range() < self._radius + self._security_distance:
                beta = (pi - alpha) * (1 - ((obstacle.get_range() -
                                             self._radius) / self._security_distance))

            alpha_beta = alpha + beta

            obstacle_direction_goal_origin = rad_bounded(
                obstacle.get_angle() - goal_direction)
            if obstacle_direction_goal_origin > 0:
                if obstacle_direction_goal_origin - alpha_beta < rad_bounded(right_bound - goal_direction):
                    right_bound = obstacle.get_angle() - alpha_beta
            else:
                if obstacle_direction_goal_origin + alpha_beta > rad_bounded(left_bound - goal_direction):
                    left_bound = obstacle.get_angle() + alpha_beta

        return left_bound, right_bound

    def is_path_clear(self, goal, obstacles):
        # A contains points on the left side of goal
        # B contains points on the right side of goal
        (A, B) = self.get_points_of_interest(goal, obstacles)

        # Corners of the rectangle (tunnel)
        rectagle = self.get_rectangle(goal)

        A = self.get_points_in_polygon(rectagle, A)
        B = self.get_points_in_polygon(rectagle, B)

        # Check if path is clear
        diameter = 2 * self._radius  # Use squared diameter instead
        for a in A:
            for b in B:
                if hypot(b[0] - a[0], b[1] - a[1]) < diameter:
                    return False

        return True

    def get_points_of_interest(self, goal, obstacles):
        goal_direction = atan2(goal[1], goal[0])
        max_distance = hypot(goal[0], goal[1]) + self._radius

        left = []
        right = []

        d = 0
        while pi > d:
            direction = goal_direction + d
            if obstacles.isfinite(direction) and obstacles.get_range(direction) <= max_distance:
                left.append(obstacles.get_point(direction))

            direction = goal_direction - d
            if obstacles.isfinite(direction) and obstacles.get_range(direction) <= max_distance:
                right.append(obstacles.get_point(direction))
            
            d += obstacles.bucket_size()

        return left, right

    def get_points_in_polygon(self, polygon, points):
        points_in_polygon = []

        for point in points:
            if not isfinite(point[0]) or not isfinite(point[1]):
                # Not possible to be inside the polygon
                continue

            if self.is_point_in_polygon(polygon, point):
                points_in_polygon.append(point)

        return points_in_polygon

    # Source:
    # https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    def is_point_in_polygon(self, polygon, point):
        # TODO: Check if correct
        c = False
        j = len(polygon) - 1
        for i in range(0, len(polygon)):
            if ((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) and (point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]):
                c = not c

            j = i

        return c

    def get_rectangle(self, goal):
        distance = hypot(goal[0], goal[1]) + self._radius
        direction = atan2(goal[1], goal[0])

        moved_goal = [distance * cos(direction), distance * sin(direction)]

        dx = moved_goal[1] / distance
        dy = -moved_goal[0] / distance

        # 2 * radius becuase then all obstacles that matters will be inside the rectangle
        diameter = 2.0 * self._radius

        rectangle = []
        rectangle.append([diameter * dx, diameter * dy])
        rectangle.append([moved_goal[0] + rectangle[0][0],
                          moved_goal[1] + rectangle[0][1]])
        rectangle.append([moved_goal[0] - rectangle[0][0],
                          moved_goal[1] - rectangle[0][1]])
        rectangle.append([-rectangle[0][0], -rectangle[0][1]])

        return rectangle
