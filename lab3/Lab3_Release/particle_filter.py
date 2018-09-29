from grid import *
from particle import Particle
from utils import *
from setting import *

from bisect import bisect_left

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
    def __init__(self, particle, weight):
        self.particle = particle
        self.weight = weight

    def __str__(self):
        return "Particle: ({}, {}, {}), Weight: {}".format(
            self.particle.x,
            self.particle.y,
            self.particle.z,
            self.weight,
        )

    def __repr__(self):
        return "Particle: ({}, {}, {}), Weight: {}".format(
            self.particle.x,
            self.particle.y,
            self.particle.h,
            self.weight,
        )

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
    if len(measured_marker_list) == 0:
        return particles

    particle_weights = []

    for p in particles:
        if grid.is_free(p.x, p.y):
            if len(measured_marker_list) > 0:
                particle_markers = p.read_markers(grid)
                particle_weights.append(
                    get_particle_weight(
                        measured_marker_list=measured_marker_list,
                        particle_marker_list=particle_markers,
                        particle=p,
                    )
                )
            else:
                particle_weights.append(
                    ParticleWeight(particle=p, weight=1)
                )
        else:
            particle_weights.append(
                ParticleWeight(particle=p, weight=0)
            )

    total_weight = sum([p.weight for p in particle_weights])
    mu = total_weight / len(particle_weights)
    count_positive = sum(1 for p in particle_weights if p.weight > 0)
    if count_positive > 0:
        mu = (mu / count_positive) * len(particles)

    if mu > 0:
        for pw in particle_weights:
            pw.weight = pw.weight / mu

    particle_weight_cdf = []
    curr_total = 0
    for pw in particle_weights:
        curr_total += pw.weight
        particle_weight_cdf.append(curr_total)

    measured_particles = []
    for i in range(len(particles)):
        curr_particle = generate_particle_from_dist(particle_weights, particle_weight_cdf, grid)
        if curr_particle:
            measured_particles.append(Particle(curr_particle[0], curr_particle[1], curr_particle[2]))
    return measured_particles

def generate_particle_from_dist(particle_weights, particle_weight_cdf, grid):
    bound = np.random.uniform() # [0, 1) uniform dist
    rand_index = bisect_left(particle_weight_cdf, bound)
    if 0 <= rand_index < len(particle_weights):
        return particle_weights[rand_index].particle
    else:
        rand_particle = Particle.create_random(1, grid)[0]
        return (rand_particle.x, rand_particle.y, rand_particle.h)

def get_particle_weight(measured_marker_list, particle_marker_list, particle):
    if len(measured_marker_list) == 0 or len(particle_marker_list) == 0:
        return ParticleWeight(particle, 0)

    i_measured_list = 0

    matched = {}
    while i_measured_list < len(measured_marker_list) and len(particle_marker_list) > 0:

        marker_measurement = measured_marker_list[i_measured_list]

        def distance_cmp(particle):
            return grid_distance(
                marker_measurement[0], marker_measurement[1], particle[0], particle[1]
            )
        closest_particle = min(particle_marker_list, key=lambda x: distance_cmp(x))

        matched[marker_measurement] = closest_particle
        particle_marker_list.remove(closest_particle)

        i_measured_list += 1

    weight = 1
    for marker, particle in matched.items():
        angle = diff_heading_deg(marker[2], particle[2])
        dist = grid_distance(marker[0], marker[1], particle[0], particle[1])

        a = (dist**2 + angle**2) / (-2 * MARKER_TRANS_SIGMA ** 2)
        weight *= np.exp(a)
    return ParticleWeight(particle, weight)
