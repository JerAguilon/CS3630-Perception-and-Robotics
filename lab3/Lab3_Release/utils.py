# from setting import *
import setting
import random
random.seed(setting.RANDOM_SEED)
import math

""" Some math utilies, feel free to use any of these!!!
"""

# euclian distance in grid world
def grid_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# utils for 2d rotation, given frame \theta
def rotate_point(x, y, heading_deg):
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr


# heading angle difference = heading1 - heading2
# return value always in range (-180, 180] in deg
def diff_heading_deg(heading1, heading2):
	dh = heading1 - heading2
	while dh > 180:
		dh -= 360
	while dh <= -180:
		dh += 360
	return dh


def compute_mean_pose(particles, confident_dist=1):
    """ Compute the mean pose for all particles
    	This is not part of the particle filter algorithm but rather an
    	addition to show the "best belief" for current pose
    """
    m_x, m_y, m_count = 0, 0, 0
    # for rotation average
    m_hx, m_hy = 0, 0
    for p in particles:
        m_count += 1
        m_x += p.x
        m_y += p.y
        m_hx += math.sin(math.radians(p.h))
        m_hy += math.cos(math.radians(p.h))

    if m_count == 0:
        return -1, -1, 0, False

    m_x /= m_count
    m_y /= m_count

    # average rotation
    m_hx /= m_count
    m_hy /= m_count
    m_h = math.degrees(math.atan2(m_hx, m_hy));

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if grid_distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_h, m_count > len(particles) * 0.95


# utils to add gaussian noise

def add_gaussian_noise(data, sigma):
    return data + random.gauss(0.0, sigma)

def add_odometry_noise(odom_act, heading_sigma, trans_sigma):
    return (add_gaussian_noise(odom_act[0], trans_sigma), \
        add_gaussian_noise(odom_act[1], trans_sigma), \
        add_gaussian_noise(odom_act[2], heading_sigma))

def add_marker_measurement_noise(marker_measured, trans_sigma, rot_sigma):
    return (add_gaussian_noise(marker_measured[0], trans_sigma), \
        add_gaussian_noise(marker_measured[1], trans_sigma), \
        add_gaussian_noise(marker_measured[2], rot_sigma))
