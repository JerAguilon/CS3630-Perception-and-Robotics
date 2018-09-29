from __future__ import absolute_import

import threading
import time

from grid import CozGrid
from particle import Particle, Robot
import setting
from particle_filter import *
from utils import *


# map file
Map_filename = "map_test.json"


""" Autograder rubric
    Total score = 90 (DETECTION_FAILURE_RATE, SPURIOUS_DETECTION_RATE = 0)
                + 10 (DETECTION_FAILURE_RATE, SPURIOUS_DETECTION_RATE = 0.1)

    Each part corresponds to two stages:
    1.  Build tracking:
        if the filter can build tracking and output correct robot transformation
        in error tolarance anytime in 100 steps, give 50 points
    2.  Maintain tracking:
        let the filter run 100 steps, give score
        score = correct pose percentage / 100 * 50
              = #correct pose / #total pose * 50
"""
# number of steps to allow build tracking
Steps_build_tracking = 95
# number of steps must have stable tracking
Steps_stable_tracking = 100
# translational error allow
Err_trans = 1.0
# orientation erro allow in degree
Err_rot = 10


""" Robot motion: Robot move as a circle, and the circle params are defined as:
        Robot_speed: robot moving speed (grid per move)
        Robot_init_pose: initial robot transformation (X, Y, yaw in deg)
        Dh_circular: Angle (in degree) to turn per run in circle motion mode
    Here we give 5 example circles you can test, just uncomment the one you like
    But note that in real grading we will use *another* 5 different circles,
    so don't try to hack anything!
"""
# example circle 1
Robot_init_pose = (6, 3, 0)
Dh_circular = 10
Robot_speed = 0.5

# # example circle 2
# Robot_init_pose = (5, 1, 0)
# Dh_circular = 6
# Robot_speed = 0.5

# # example circle 3
# Robot_init_pose = (5, 4, 0)
# Dh_circular = 20
# Robot_speed = 0.3

# # example circle 4
# Robot_init_pose = (3, 2, 0)
# Dh_circular = 20
# Robot_speed = 0.3

# # example circle 5
# Robot_init_pose = (9, 9, 180)
# Dh_circular = 15
# Robot_speed = 0.5


# move robot circular
# if in collsion throw error
def move_robot_circular(robot, dh, speed, grid):
    old_x, old_y = robot.x, robot.y
    old_heading = robot.h
    if robot.check_collsion((speed, 0, dh), grid):
        raise ValueError('Robot collision')
    else:
        robot.move((speed, 0, dh))
    # calc odom
    dx, dy = rotate_point(robot.x-old_x, robot.y-old_y, -old_heading)
    return (dx, dy, dh)



class ParticleFilter:

    def __init__(self, particles, robbie, grid):
        self.particles = particles
        self.robbie = robbie
        self.grid = grid

    def update(self):

        # ---------- Move Robot ----------
        odom = add_odometry_noise(move_robot_circular(self.robbie, Dh_circular, Robot_speed, self.grid), \
            heading_sigma=setting.ODOM_HEAD_SIGMA, trans_sigma=setting.ODOM_TRANS_SIGMA)


        # ---------- Motion model update ----------
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


        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)


        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


def run_test_case(Robot_init_pose, Dh_circular, Robot_speed):
    grid = CozGrid(Map_filename)

    # initial distribution assigns each particle an equal probability
    particles = Particle.create_random(setting.PARTICLE_COUNT, grid)
    robbie = Robot(Robot_init_pose[0], Robot_init_pose[1], Robot_init_pose[2])
    particlefilter = ParticleFilter(particles, robbie, grid)

    score = 0

    # 1. steps to build tracking
    steps_built_track = 9999
    for i in range(0, Steps_build_tracking):

        est_pose = particlefilter.update()

        if grid_distance(est_pose[0], est_pose[1], robbie.x, robbie.y) < Err_trans \
                and math.fabs(diff_heading_deg(est_pose[2], robbie.h)) < Err_rot \
                and i+1 < steps_built_track:
            steps_built_track = i+1

    #print("steps_built_track =", steps_built_track)
    if steps_built_track < 50:
        score = 45
    elif steps_built_track < Steps_build_tracking:
        score = Steps_build_tracking-steps_built_track
    else:
        score = 0

    print("\nPhase 1")
    print("Number of steps to build track :", steps_built_track, "/", Steps_build_tracking)
    acc_err_trans, acc_err_rot = 0.0, 0.0
    max_err_trans, max_err_rot = 0.0, 0.0
    step_track = 0

    # 2. test tracking
    score_per_track = 45.0 / Steps_stable_tracking

    for i in range(0, Steps_stable_tracking):

        est_pose = particlefilter.update()

        err_trans = grid_distance(est_pose[0], est_pose[1], robbie.x, robbie.y)
        acc_err_trans += err_trans
        if max_err_trans < err_trans:
            max_err_trans = err_trans

        err_rot = math.fabs(diff_heading_deg(est_pose[2], robbie.h))
        acc_err_rot += err_rot
        if max_err_rot < err_rot:
            max_err_rot = err_rot

        if grid_distance(est_pose[0], est_pose[1], robbie.x, robbie.y) < Err_trans \
                and math.fabs(diff_heading_deg(est_pose[2], robbie.h)) < Err_rot:
            step_track += 1
            score += score_per_track

    print("\nPhase 2")
    print("Number of steps error in threshold :", step_track, "/", Steps_stable_tracking)
    print("Average translational error :", acc_err_trans / Steps_stable_tracking)
    print("Average rotational error :", acc_err_rot / Steps_stable_tracking, "deg")
    print("Max translational error :", max_err_trans)
    print("Max rotational error :", max_err_rot, "deg")

    return score

# Scores from this stress test are simply for reference, nothing to do with the final grade
def stress_test(num_iter=5):
    r = random.random

    print("-----------STRESS TEST--------------")
    i = 0
    scores = []
    while i < num_iter:
        try:
            a1 = (r()*8+1, r()*8+1, r()*360)
            a2 = r()*15+5
            a3 = r()*0.4+0.2
            score = run_test_case(a1,a2,a3)
            scores.append(score)
            print("\nscore =", score)
            print('mean score: ', np.mean(scores))
            i += 1
        except Exception as e:
            print(e, ', invalid track. Trying another track.')
            continue


if __name__ == "__main__":

    score_without_noise = run_test_case(Robot_init_pose, Dh_circular, Robot_speed)
    setting.DETECTION_FAILURE_RATE = 0.1
    setting.SPURIOUS_DETECTION_RATE = 0.1
    score_with_noise = run_test_case(Robot_init_pose, Dh_circular, Robot_speed)
    score_with_noise = (5.0/45.0) * score_with_noise

    if score_with_noise > 9:
        score_with_noise = 10

    score = score_without_noise + score_with_noise
    print("\nscore without noise =", score_without_noise)
    print("\nscore with noise =", score_with_noise)
    print("\nTotal Score =", round(score,2))
    # stress_test()
