import numpy as np


class RobotPose:

    def __init__(self, x = 0, y = 0, yaw = 0):

        # Current pose of robot in odom frame
        self._x = x
        self._y = y
        self._yaw = yaw

        self._translation_odometry_to_map = [0,0,0] # dx ,dy, dyaw


    def get_pose_odom(self):
        return self._x, self._y, self._yaw


    def get_pose_on_map(self):
        # Calculate transformation from odom to map
        dyaw = self._translation_odometry_to_map[2]
        x_map = (self._x * np.cos(-dyaw) - self._y * np.sin(-dyaw))
        y_map = (self._x * np.sin(-dyaw) + self._y * np.cos(-dyaw))

        x_map += self._translation_odometry_to_map[0]
        y_map += self._translation_odometry_to_map[1]
        yaw_map = dyaw + self._yaw

        return x_map, y_map, yaw_map


    def get_odometry_to_map_translation(self):
        return self._translation_odometry_to_map


    def set_get_odometry_to_map_translation(self, translation):
        self._translation_odometry_to_map = translation


    def set_robot_pose_in_map_frame(self, x_new, y_new, yaw_new):
        # calculates correct offset between map and odom frame

        x_map, y_map, yaw_map = self.get_pose_on_map()
        self._translation_odometry_to_map[0] = -x_map + x_new
        self._translation_odometry_to_map[1] = -y_map + y_new
        self._translation_odometry_to_map[2] = -yaw_map + self._yaw + yaw_new


    def update_odom_pose(self, dx, dy, dyaw):
        # inputs are in robot frame

        # Calculate corresponding translation in osom frame
        dyaw_robot_to_odom = -self._yaw + 3.1415/2
        dy_odom = (dx * np.cos(dyaw_robot_to_odom) - dy * np.sin(dyaw_robot_to_odom))
        dx_odom = -(dx * np.sin(dyaw_robot_to_odom) + dy * np.cos(dyaw_robot_to_odom))
        #print([dx, dy, dyaw, dx_odom,dy_odom])

        self._yaw += dyaw
        self._x += dx_odom
        self._y += dy_odom

        return self._x, self._y, self._yaw
