from __future__ import absolute_import

import threading
import time

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
# from setting import *
from particle_filter import *
from utils import *


# map you want to test
Map_filename = "map_test.json"

# whether enable the GUI
Use_GUI = True


""" Robot Motion parameters
	Feel free to change these param for your debug
"""
# whether move in a circle: There are two robot motion pattern implemented:
# 1. Robot move forward, if hit an obstacle, robot bounces to a random direction
# 2. Robot move as a circle (This is the motion autograder uses)
# This is the flag to enable circle motion or not
Move_circular = False

# robot moving speed (grid per move)
Robot_speed = 0.5
# initial robot transformation (X, Y, yaw in deg)
Robot_init_pose = (6, 3, 0)
# Angle (in degree) to turn per run in circle motion mode
Dh_circular = 10



# Forward motion mode: move robot just forward
# if in collsion, bouncing to a random direction which is collision free
def move_robot_forward(robot, speed, grid):
    old_x, old_y = robot.x, robot.y
    old_heading = robot.h
    while True:
        dh = diff_heading_deg(robot.h, old_heading)
        if not robot.check_collsion((speed, 0, dh), grid):
            robot.move((speed, 0, dh))
            break
        # Bumped into something, chose random new direction
        robot.h = robot.chose_random_heading()
    # calc odom
    dx, dy = rotate_point(robot.x-old_x, robot.y-old_y, -old_heading)
    return (dx, dy, diff_heading_deg(robot.h, old_heading))


# Circular motion mode:
# if in collsion throw error
# This is the motion mode autograder will use
def move_robot_circular(robot, dh, speed, grid):
    old_x, old_y = robot.x, robot.y
    old_heading = robot.h
    if robot.check_collsion((speed, 0, dh), grid):
        raise ValueError('Robot in collision')
    else:
        robot.move((speed, 0, dh))
    # calc odom
    dx, dy = rotate_point(robot.x-old_x, robot.y-old_y, -old_heading)
    return (dx, dy, dh)


# particle filter class
class ParticleFilter:

    def __init__(self, particles, robbie, grid):
        self.particles = particles
        self.robbie = robbie
        self.grid = grid

    def update(self):

        # ---------- Move Robot and get odometry ----------
        if Move_circular:
            odom = add_odometry_noise(move_robot_circular(self.robbie, Dh_circular, Robot_speed, self.grid), \
                heading_sigma=setting.ODOM_HEAD_SIGMA, trans_sigma=setting.ODOM_TRANS_SIGMA)
        else:
            odom = add_odometry_noise(move_robot_forward(self.robbie, Robot_speed, self.grid), \
                heading_sigma=setting.ODOM_HEAD_SIGMA, trans_sigma=setting.ODOM_TRANS_SIGMA)

        print('\nrobot :', self.robbie)
        print('odometry measured :', odom)


        # ---------- PF: Motion model update ----------
        self.particles = motion_update(self.particles, odom)


        # ---------- Find markers in camera ----------
        # read markers
        r_marker_list_raw = self.robbie.read_markers(self.grid)
        #print("r_marker_list :", r_marker_list)

        # add noise to marker list
        r_marker_list = []
        for m in r_marker_list_raw:
            r_marker_list.append(add_marker_measurement_noise(m, \
                trans_sigma=setting.MARKER_TRANS_SIGMA, rot_sigma=setting.MARKER_ROT_SIGMA))


        # ---------- PF: Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)


        # ---------- Display current state in GUI ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


# thread to run particle filter when GUI is on
class ParticleFilterThread(threading.Thread):

    def __init__(self, particle_filter, gui):
        threading.Thread.__init__(self, daemon=True)
        self.filter = particle_filter
        self.gui = gui

    def run(self):
        while True:
            estimated = self.filter.update()
            self.gui.show_particles(self.filter.particles)
            self.gui.show_mean(estimated[0], estimated[1], estimated[2], estimated[3])
            self.gui.show_robot(self.filter.robbie)
            self.gui.updated.set()


if __name__ == "__main__":
    grid = CozGrid(Map_filename)

    # initial distribution assigns each particle an equal probability
    particles = Particle.create_random(setting.PARTICLE_COUNT, grid)
    robbie = Robot(Robot_init_pose[0], Robot_init_pose[1], Robot_init_pose[2])
    particlefilter = ParticleFilter(particles, robbie, grid)

    if Use_GUI:
        gui = GUIWindow(grid)
        filter_thread = ParticleFilterThread(particlefilter, gui)
        filter_thread.start()
        gui.start()
    else:
        while True:
            particlefilter.update()

