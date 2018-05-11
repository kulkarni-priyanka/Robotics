from lab8.grid import *
from lab8.particle import Particle
from lab8.utils import *
from lab8.setting import *
import scipy.stats
import numpy as np
import copy


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
        #add odometry noise to the sample
        odom_sample = add_odometry_noise(odom, heading_sigma=ODOM_HEAD_SIGMA,
                                         trans_sigma=ODOM_TRANS_SIGMA)
        dx, dy = rotate_point(odom_sample[0], odom_sample[1], p.h)
        dh = odom_sample[2]
        #update particles so as to translate and rotate by delta
        p.x += dx
        p.y += dy
        p.h += dh
        motion_particles.append(p)
    return motion_particles

def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map, which contains the marker information,
                see grid.h and CozGrid for definition

        Returns: the list of particle represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weightArr = []

    if len(measured_marker_list) != 0:
        for particle in particles:
            # Obtain list of localization markers
            visibleMarkers = particle.read_markers(grid)

            if not (grid.is_in(particle.x, particle.y)or (particle.x,particle.y) in grid.occupied):
                weightArr.append(0)
            else:


                mmlLength = len(measured_marker_list)
                vmLength = len(visibleMarkers)
                pairs = []
                for measuredMarker in measured_marker_list:
                    if len(visibleMarkers) != 0:
                        # find closest marker
                        nearestMarker = findNearestMarker(measuredMarker, visibleMarkers)
                        # remove from possible future pairings
                        visibleMarkers.remove(nearestMarker)
                        # store pairings
                        pairs.append((nearestMarker, measuredMarker))

                weightArr.append( getProbability(pairs, mmlLength, vmLength))



    else:
        weightArr = [1 / PARTICLE_COUNT for x in particles]

    if(len(weightArr) != len(particles)):
        print("Enter here")

    w = weightArr
    norm_const = sum(w)
    if norm_const ==0:
        weight_list = [1 /PARTICLE_COUNT for x in w]
    else:
        weight_list = [x / float(norm_const) for x in w]

    #Eliminate 2% particles from existing resampling list and replace them with random particles
    extra = int(0.02 * PARTICLE_COUNT)
    measured_particles = np.random.choice(particles, PARTICLE_COUNT - extra,replace=True, p=weight_list)
    measured_particles = [copy.deepcopy(m) for m in measured_particles]
    additional_particles = np.asarray(Particle.create_random(extra, grid))
    measured_particles = np.concatenate((measured_particles, additional_particles), axis=0)

    return measured_particles





def findNearestMarker(measuredMarker, visibleMarkers):
    measuredMarkerX, measuredMarkerY, _ = add_marker_measurement_noise(measuredMarker, MARKER_TRANS_SIGMA,
                                                                       MARKER_ROT_SIGMA)
    nearestMarker = visibleMarkers[0]
    nearestDistance = grid_distance(measuredMarkerX, measuredMarkerY, nearestMarker[0], nearestMarker[1])
    for visibleMarker in visibleMarkers:
        visibleMarkerX = visibleMarker[0]
        visibleMarkerY = visibleMarker[1]
        distance = grid_distance(measuredMarkerX, measuredMarkerY, visibleMarkerX, visibleMarkerY)
        if distance < nearestDistance:
            nearestMarker = visibleMarker
            nearestDistance = distance

    return nearestMarker


def getProbability(pairs, mmlLength, vmLength):
    probability = 1
    transConstantMax = 0
    for p1, p2 in pairs:
        # euclidean distance of each pair
        markerDistance = grid_distance(p1[0], p1[1], p2[0], p2[1])
        markerAngle = diff_heading_deg(p1[2], p2[2])

        # a: distBetweenMarkers^2 / (2 * (standard deviation of the gaussian model for translation)^2)
        transConstantMax = max(transConstantMax, (markerDistance ** 2) / (2 * (MARKER_TRANS_SIGMA ** 2)))

        # b: angleBetweenMarkers^2 / (2 * (standard deviation for rotation measurements)^2)
        newRotConstant = (markerAngle ** 2) / (2 * (MARKER_ROT_SIGMA ** 2))

        # p = e^-(a + b)
        power = ((markerDistance ** 2) / (2 * (MARKER_TRANS_SIGMA ** 2))) + newRotConstant
        probability = probability * np.exp(-power)

    # handle measured marker and visible marker difference
    rotConstantMax = (45 ** 2) / (2 * (MARKER_ROT_SIGMA ** 2))
    difference = math.fabs(mmlLength - vmLength)
    count = 0
    while count < int(difference):
        probability = probability * np.exp(-transConstantMax - rotConstantMax)
        count = count + 1

    return probability


def normpdf(x, mean, sd):
    var = float(sd) ** 2
    pi = math.pi
    denom = (2 * pi * var) ** .5
    num = math.exp(-(float(x) - float(mean)) ** 2 / (2 * var))
    return (num / denom)
