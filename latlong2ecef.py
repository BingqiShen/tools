# from curses.ascii import FF
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from gps_common.msg import GPSFix


def latLonToEcef(lat, lon, alt):

    DEGREES_TO_RADIANS = math.pi / 180
    WGS84_A = 6378137.
    f = 1. / 298.257223563
    WGS84_B = WGS84_A * (1. - f)
    WGS84_E = math.sqrt(WGS84_A * WGS84_A - WGS84_B * WGS84_B) / WGS84_A

    clat = math.cos(lat * DEGREES_TO_RADIANS)
    slat = math.sin(lat * DEGREES_TO_RADIANS)
    clon = math.cos(lon * DEGREES_TO_RADIANS)
    slon = math.sin(lon * DEGREES_TO_RADIANS)

    N = WGS84_A / math.sqrt(1.0 - WGS84_E * WGS84_E * slat * slat)

    x = (N + alt) * clat * clon
    y = (N + alt) * clat * slon
    z = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat

    return x, y, z

def ecefTomap(ecef_x, ecef_y, ecef_z):
    t_base2ecef = np.array([ecef_x, ecef_y, ecef_z, 1]).T
    q_ecef2map = [0.431550779388071, -0.248567031217781, -0.75143217078753, 0.432814103867547]
    R_ecef2map_ = R.from_quat(q_ecef2map)
    R_ecef2map = R_ecef2map_.as_matrix()

    t_ecef2map = np.array([[-18601.3472204838, 0, -6372712.67892339]]).T


    T_ecef2map = np.vstack([
        np.hstack([R_ecef2map, t_ecef2map]),
        np.array([[0.0, 0.0, 0.0, 1.0]]),
    ])

    t_base2map = T_ecef2map @ t_base2ecef

    return t_base2map[0], t_base2map[1], t_base2map[2]

def gps_callback(msg):
    ecef_x, ecef_y, ecef_z = latLonToEcef(msg.latitude, msg.longitude, 0)
    print(ecef_x, ecef_y, ecef_z)

    map_x, map_y, map_z = ecefTomap(ecef_x, ecef_y, ecef_z)
    print(map_x, map_y, map_z)


def rtk_subscriber():
    rospy.init_node("read_gps", anonymous=True)
    rospy.Subscriber("/jzhw/gps/fix", gps_callback)
    rospy.spin()




if __name__ == '__main__':
    rtk_subscriber()