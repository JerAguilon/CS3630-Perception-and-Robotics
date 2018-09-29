from grid import *
from particle import Particle
from utils import *
from setting import *

import numpy as np


class ParticleWeight(object):
    def __init__(self, particle, weight):
        self.particle = particle
        self.weight = weight

    def __str__(self):
        print(type(self.particle))
        return "Particle: ({}, {}, {}), Weight: {}".format(
            self.particle.x,
            self.particle.y,
            self.particle.h,
            self.weight,
        )

    def __repr__(self):
        return "Particle: ({}, {}, {}), Weight: {}".format(
            self.particle.x,
            self.particle.y,
            self.particle.h,
            self.weight,
        )

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
            pw.weight = pw.weight / total_weight

    particle_weight_cdf = []
    curr_total = 0
    for pw in particle_weights:
        curr_total += pw.weight
        particle_weight_cdf.append(curr_total)


    count_zero = len(particle_weights) - count_positive
    measured_particles = Particle.create_random(count_zero, grid)
    if particle_weights != []:
        pdf = [p.weight for p in particle_weights]
        sample = np.random.choice(particle_weights, size=count_positive, p=pdf, replace=True)
        for s in sample:
            p = s.particle
            measured_particles.append(
                Particle(p.x, p.y, p.h)
            )

    assert len(measured_particles) == len(particles)
    return measured_particles

def get_particle_weight(measured_marker_list, particle_marker_list, particle):
    if len(measured_marker_list) == 0 or len(particle_marker_list) == 0:
        return ParticleWeight(particle, 0)

    i_measured_list = 0

    matched = []

    particle_set = set(particle_marker_list)

    while i_measured_list < len(measured_marker_list) and len(particle_set) > 0:

        marker_measurement = measured_marker_list[i_measured_list]

        def distance_cmp(particle):
            return grid_distance(
                marker_measurement[0], marker_measurement[1], particle[0], particle[1]
            )
        closest_particle = min(particle_marker_list, key=lambda x: distance_cmp(x))

        matched.append( (marker_measurement, closest_particle) )
        if closest_particle in particle_set:
            particle_set.remove(closest_particle)

        i_measured_list += 1

    weight = 1
    for marker, closest_particle in matched:
        angle = diff_heading_deg(marker[2], closest_particle[2])
        dist = grid_distance(marker[0], marker[1], closest_particle[0], closest_particle[1])

        a = -((dist**2 / (2 * MARKER_TRANS_SIGMA ** 2)) + \
              (angle**2 / (2 * MARKER_ROT_SIGMA ** 2)))
        weight *= np.exp(a)
    return ParticleWeight(particle, weight)
