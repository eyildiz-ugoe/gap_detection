"""
Helper methods
==============
Methods for gap_detector.py to convert or construct different ROS message
formats, evaluate the detector as well as calculating the convex hull of a gap.

"""

import numpy as np
import rospy
import rospkg
import geometry_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from scipy.spatial import ConvexHull


def PC2_to_numpy_array(pointcloud):
    """
    Converts a sensor_msgs.pointcloud2 ROS message to a numpy array containing
    all points by their x,y,z coordinate values.

    Parameters
    ----------
    pointcloud : sensor_msgs.pointcloud2
        Pointcloud2 ROS message.

    Returns
    -------
    numpy.array
        Numpy array of all points of the pointcloud given in order:
        [[x1,y1,z1], [x2,y2,z2], ....

    Notes
    -----
    http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html

    """

    dimensions = ("x", "y", "z")
    points = pc2.read_points(
        pointcloud, field_names=dimensions, skip_nans=True)

    point_list = []
    for p in points:
        point_list.append([p[0], p[1], p[2]])

    return np.array(point_list)


def numpy_array_to_PC2(numpy_array, frame_id):
    """
    Converts a numpy array containing all points by their x,y,z coordinate
    values to a sensor_msgs.pointcloud2 ROS message.

    Parameters
    ----------
    numpy_array : numpy.array
        Numpy array containing points in order:
        [[x1,y1,z1], [x2,y2,z2], ....

    frame_id : String
        Frame id of the Pointcloud2 header.

    Returns
    -------
    sensor_msgs.pointcloud2
        Pointcloud2 ROS message with pointfields x,y and z.

    Notes
    -----
    http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html

    """

    points = numpy_array.tolist()

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = frame_id

    return pc2.create_cloud(header, fields, points)


def calculate_convex_hulls_and_centers(gaps):
    """
    Calculates the convex hull for the given gap points including the
    simplices, vertices, volume, number of points in the gap and the center
    point(center of the hull).

    Parameters
    ----------
    gaps : list
        List of gaps with the points of the gap in order:
        [[[x1,y1,z1], [x2,y2,z2], ... ],[[x11,y11,z11], [x12,y12,z12], ...]...]
                      GAP1                              GAP2            ...

    Returns
    -------
    list
        List of convex hull info for each gap:
        [[center, vertices, simplices, hull.volume, sizes, num_of_pts],      GAP1
         [center, vertices, simplices, hull.volume, sizes, num_of_pts],      GAP2
                            ....                                      ....

    """

    convex_hull = []

    for gap in gaps:
        hull = ConvexHull(gap, qhull_options="QJ")

        # x, y, z components of the hull vertices
        vertices_x = hull.points[hull.vertices, 0]
        vertices_y = hull.points[hull.vertices, 1]
        vertices_z = hull.points[hull.vertices, 2]

        # calculate center of hull by taking middle of extremas of axes
        cx = (np.max(vertices_x)+np.min(vertices_x))/2
        cy = (np.max(vertices_y)+np.min(vertices_y))/2
        cz = (np.max(vertices_z)+np.min(vertices_z))/2
        center = [cx, cy, cz]

        # calculate the sizes
        sx = (np.max(vertices_x)-np.min(vertices_x))/2
        sy = (np.max(vertices_y)-np.min(vertices_y))/2
        sz = (np.max(vertices_x)-np.min(vertices_x))/2
        size = [sx, sy, sz]


        # map vertices and simplices to gap points
        vertices = []
        for vertex in hull.vertices:
            x, y, z = hull.points[vertex]
            vertices.append([x, y, z])

        simplices = []
        for simplex in hull.simplices:
            x, y, z = hull.points[simplex]
            simplices.append([x, y, z])

        # number of points(for evaluation)
        num_of_points, dim = gap.shape

        info = [center, vertices, simplices, hull.volume, size, num_of_points]
        convex_hull.append(info)

    return convex_hull


def evaluate_detector(num_of_points):
    """
    Evaluate the detector by writing infos about the found gaps to
    detector_evaluation.txt.

    Parameters
    ----------
    num_of_points : tuple
        Tuple of the number of points for each gap.

    """

    rospack = rospkg.RosPack()
    src_dir = rospack.get_path('ugoe_gap_detection_ros')
    file_path = src_dir + "/evaluation/detector_evaluation.txt"

    try:
        file = open(file_path, "w")
    except (IOError, OSError):
        rospy.logwarn("Could not open/read file:" + file_path)
        return

    for gap, number_of_points in enumerate(num_of_points):
        file.write("Gap " + str(int(gap)+1) + " - " +
                   str(number_of_points) + " points\n")

    rospy.loginfo("Saved evaluation report of detector to " + src_dir + "/evaluation/detector_evaluation.txt.")

    file.close()
