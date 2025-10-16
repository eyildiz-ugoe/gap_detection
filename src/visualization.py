"""
Visualization
=============
Methods for gap_detector.py to visualize outputs of the gap detector in RVIZ
or Matplotlib.

"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


def draw_convex_hull(gap_simplexes, frame_id):
    """
    Construct a RVIZ marker for the convex hull(s) given by the gap simplexes.

    Parameters
    ----------
    gap_simplexes : (ndarray of ints, shape (nfacet, ndim))
        List of simplexes of each gap computed by scipy.spatial.ConvexHull.

    frame_id : String
        Frame id of the outgoing marker.

    Returns
    -------
    visualization_msgs.msg.Marker
        Marker of type LINE_LIST showing the boundaries of the convex hulls.

    """

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.type = Marker.LINE_LIST
    marker.scale.x = 0.0004
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(0.) # 0 means forever
    marker.points = []
    marker.header.stamp = rospy.Time.now()

    for gap in gap_simplexes:
        for simplex in gap:
            x, y, z = simplex
            marker.points.append(Point(x[0], x[1], x[2]))
            marker.points.append(Point(y[0], y[1], y[2]))

            marker.points.append(Point(y[0], y[1], y[2]))
            marker.points.append(Point(z[0], z[1], z[2]))

            marker.points.append(Point(x[0], x[1], x[2]))
            marker.points.append(Point(z[0], z[1], z[2]))

    return marker


def draw_centers(center_list, frame_id):
    """
    Draws a red sphere for each gap center in RVIZ.

    Parameters
    ----------
    center_list : list
        List of center for each gap.

    frame_id : String
        Frame id of the outgoing marker.

    Returns
    -------
    visualization_msgs.msg.MarkerArray
        MarkerArray of Marker.SPHERE for each gap center.

    """

    markerArray = MarkerArray()

    for center in center_list:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.lifetime = rospy.Duration(0.)

        markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    return markerArray


def draw_volume_text(volume_list, center_list, frame_id):
    """
    Draw a gaps volume in RVIZ at the center point of the gap.

    Parameters
    ----------
    volume_list : list
        List of volumes for each gap. Should be the same order as center_list.

    center_list : list
        List of center for each gap. Should be the same order as volume_list.

    frame_id : String
        Frame id of the outgoing marker.

    Returns
    -------
    visualization_msgs.msg.MarkerArray
        MarkerArray of Marker.TEXT_VIEW_FACING showing the volume of the gaps.

    """
    markerArray = MarkerArray()

    for center, volume in zip(center_list, volume_list):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = str(round(volume*1000000, 4)) + "cm^3"
        marker.action = marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.lifetime = rospy.Duration(0.)

        markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    return markerArray


def plot_convex_hull(clusters):
    """
    Calculates the convex hull of each cluster and plots it in matplotlib.

    Parameters
    ----------
    clusters : list
        List of clusters in order:
        [[[x1,y1,z1], [x2,y2,z2], ... ],[[x11,y11,z11], [x12,y12,z12], ...]...]
                CLUSTER1                          CLUSTER2              ...

    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for cluster in clusters:
        points = cluster
        hull = ConvexHull(cluster)

        edges = zip(*points)

        for s in hull.simplices:
            s = np.append(s, s[0])
            plt.plot(points[s, 0], points[s, 1], points[s, 2], 'r-')
        ax.plot(edges[0], edges[1], edges[2], 'b.')

    plt.show()
