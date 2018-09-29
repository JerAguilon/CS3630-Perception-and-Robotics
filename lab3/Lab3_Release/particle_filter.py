from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    for p in particles:
        transf_x, transf_y = rotate_point(odom[0], odom[1], p.h)
        p.x += transf_x
        p.x = add_gaussian_noise(p.x, ODOM_TRANS_SIGMA)

        p.y += transf_y
        p.y = add_gaussian_noise(p.y, ODOM_TRANS_SIGMA)

        p.h += odom[2]
        p.h = add_gaussian_noise(p.h, ODOM_HEAD_SIGMA)

        motion_particles.append(p)
    return motion_particles

class ParticleWeight(object):
    def __init__(particle, weight):
        self.particle = particle
        self.weight = weight

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    particle_weights = []
    particle_count = 0

    if len(measured_marker_list) == 0:
        for p in particles:
            particle_weights.append(
                ParticleWeight(particle=p, weight=1/len(particles))
            )
    else:
        for p in particles:
            if (not (0 <= p.x < grid.width)) or (not (0 <= p.y < grid.height)):
                weight.append(ParticleWeight(particle=p, weight=0))
                continue
            # TODO(do we care about occupied?)

            curr_markers = p.read_markers(grid)
            diff = abs(len(curr_markers) - len(measured_marker_list))
            matches = recurse(curr_markers, measured_marker_list)

def recurse(curr_list, measured_marker_list, i=0, matches={}):
    if len(curr_list) == 0:
        return matches

    curr_marker = measured_marker_list[i]

    # Tuple of (x, y, h)
    marker_measurement = add_marker_measurement_noise(
        curr_marker,
        MARKER_TRANS_SIGMA,
        MARKER_ROT_SIGMA,
    )

    # Compute cloesst particle
    def distance_cmp(particle):
        return grid_distance(
            marker_measurement[0], marker_measurement[1], particle[0], particle[1]
        )
    closest_particle = min(curr_list, key=lambda x: distance_cmp(x))
    curr_list.remove(closet_particle)

    matches[closest_particle] = curr_marker
    return recurse(curr_list, measured_marker_list, i + 1, matches)
