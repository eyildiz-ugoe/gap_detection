#!/usr/bin/env python

import sys
import visualization
import helpers

# ROS
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from ugoe_gap_detection_ros.srv import GapArray, GapArrayResponse
from imagine_common.msg import PartInfo, PartOutline
from geometry_msgs.msg import Point
import geometry_msgs.msg
import imagine_common.msg
import sensor_msgs.msg
import visualization_msgs.msg

import threading

debug_lock = threading.RLock()

# Imports for dynamic reconfigure
from dynamic_reconfigure.server import Server
from ugoe_gap_detection_ros.cfg import GapDetectionConfig

# Numpy and scikit-learn
import numpy as np
from skimage.filters import threshold_otsu, threshold_minimum, threshold_li
from skimage.filters import threshold_yen
from scipy.spatial import ConvexHull

import warnings
with warnings.catch_warnings():
    # filter sklearn\externals\joblib\parallel.py:268:
    # DeprecationWarning: check_pickle is deprecated
    warnings.simplefilter("ignore", category=DeprecationWarning)
    import sklearn.cluster as cluster
    import hdbscan

NERIAN_FRAME_PERIOD = 1. / 1.5  # 1.5 Hz

class GapDetector:
    def __init__(self):
        self.detected_gap = threading.Event()  # event variable to signal gap detection
        self.image_received = threading.Event()  # event variable to signal gap detection

        self.potential_gaps_pub = rospy.Publisher('potential_gap_cloud', PointCloud2, queue_size=10)
        self.convex_hull_marker_pub = rospy.Publisher('convex_hull_marker', Marker, queue_size=1)
        self.centers_marker_pub = rospy.Publisher('center_marker', MarkerArray, queue_size=1)
        self.volume_text_marker_pub = rospy.Publisher('volume_text', MarkerArray, queue_size=1)

        # dynamic reconfigure server
        self.dyn_reconf_server = Server(GapDetectionConfig, self.callback_reconf)

        # gap detection service
        self.gap_detection_srv = rospy.Service('ugoe_gap_detection_ros/detect_gaps', GapArray, self.gap_detection_call)

        self.gaps = []

    # =============== MAIN DETECTION LOOP ===============
    def detector_callback(self, preprocessed_pc):
        self.image_received.set()
        points = helpers.PC2_to_numpy_array(preprocessed_pc)
        self.frame_id = preprocessed_pc.header.frame_id

        # ----- AUTOMATIC THRESHOLDING TO FIND GAPS -----
        depth_axis_pts = points[:, self.depth_axis]

        if(self.automatic_thresholding == 0):
            try:
                threshold = threshold_minimum(depth_axis_pts)
            except RuntimeError:
                rospy.logwarn('Threshold_minimum was unable to find two maxima in histogram!')
                return
        elif(self.automatic_thresholding == 1):
            threshold = threshold_li(depth_axis_pts)
        elif(self.automatic_thresholding == 2):
            threshold = threshold_yen(depth_axis_pts)
        elif(self.automatic_thresholding == 3):
            threshold = threshold_otsu(depth_axis_pts, self.otsu_bins)
        else:
            raise Exception('Automatic threshold value out of bounds!')

        device_surface_pts = points[depth_axis_pts <= threshold]
        self.surface_height = np.median(device_surface_pts[:, self.depth_axis])
        points = points[depth_axis_pts > threshold]

        # ----- PUBLISHING OF THE POTENTIAL GAPS -----
        potential_gaps = helpers.numpy_array_to_PC2(points, self.frame_id)
        self.potential_gaps_pub.publish(potential_gaps)

        # ----- CLUSTERING THE GAPS -----
        clustering_switch = {
            0: self.kmeans,
            1: self.birch,
            2: self.dbscan,
            3: self.hdbscan
        }

        cluster_algorithm = clustering_switch[self.clustering]
        labels = cluster_algorithm(points)

        labels = np.array(labels)
        labels_T = np.array([labels]).T
        clustered_points = np.append(points, labels_T, axis=1)


        clusters = []
        for i in set(labels):
            cluster = clustered_points[clustered_points[:, 3] == float(i)]
            cluster = cluster[:, [0, 1, 2]]

            # To construct a convex hull a minimum of 4 points is needed
            num_of_points, dim = cluster.shape
            if(num_of_points >= 4):
                clusters.append(cluster)

        # ----- VOLUME CORRECTION -----
        volume_corrected_clusters = []
        num_vol_corrected_pts = 0
        volume_corrected_pts_tuple = ()
        for cluster in clusters:
            hull = ConvexHull(cluster, qhull_options="QJ")

            # Map from vertex to point in cluster
            vertices = []
            for vertex in hull.vertices:
                x, y, z = cluster[vertex]
                vertices.append([x, y, z])

            gap = cluster.tolist()
            for vertex in vertices:
                # For each vertex, add a new point to the gap with the height
                # of the surface and the other axes corresponding to the vertex
                if(self.depth_axis == 0):
                    volume_point = [self.surface_height, vertex[1], vertex[2]]
                elif(self.depth_axis == 1):
                    volume_point = [vertex[0], self.surface_height, vertex[2]]
                elif(self.depth_axis == 2):
                    volume_point = [vertex[0], vertex[1], self.surface_height]

                gap.append(volume_point)
                num_vol_corrected_pts = num_vol_corrected_pts + 1

            volume_corrected_clusters.append(np.array(gap))
            volume_corrected_pts_tuple = volume_corrected_pts_tuple + \
                (num_vol_corrected_pts,)
            num_vol_corrected_pts = 0

        # ---- CALCULATING CONVEX HULLS OF GAPS AND THEIR CENTER -----
        self.convex_hulls_and_info = helpers.calculate_convex_hulls_and_centers(volume_corrected_clusters)

        # ---- FILTER BASED ON VOLUME -----
        self.gaps = []
        for gap_info in self.convex_hulls_and_info:
            gap_volume = gap_info[3]

            # convert cm^3 to m^3
            gap_volume = gap_volume * 1000000.0

            if(self.min_gap_volume <= gap_volume <= self.max_gap_volume):
                self.gaps.append(gap_info)

        
        # ----- EVALUATION -----
        if(self.create_evaluation):
            num_of_points = np.subtract(num_of_points, num_vol_corrected_pts)
            helpers.evaluate_detector(num_of_points)
            self.create_evaluation = False

        self.detected_gap.set()  # signal succesful gap detection


    # =============== DYNAMIC RECONFIGURE CALLBACK ===============
    def callback_reconf(self, config, level):
        self.depth_axis = config.Depthaxis
        self.min_gap_volume = config.Minimum_gap_volume
        self.max_gap_volume = config.Maximum_gap_volume

        # ----- evaluation -----
        self.create_evaluation = config.create_evaluation
        # directly uncheck checkbox to use it similar to a button
        config.create_evaluation = False

        # ----- automatic thresholding -----
        self.automatic_thresholding = config.Automatic_Thresholding
        self.otsu_bins = config.Bins

        config.groups.groups.Otsus_Method.state = False
        config.groups.groups.Yens_Method.state = False

        if(config.Automatic_Thresholding == 2):
            config.groups.groups.Yens_Method.state = True
        if(config.Automatic_Thresholding == 3):
            config.groups.groups.Otsus_Method.state = True

        # ----- clustering algorithms -----
        self.clustering = config.Clustering

        self.KM_number_of_clusters = config.number_of_clusters
        self.B_branching_factor = config.branching_factor
        self.B_threshold = config.threshold
        self.DB_eps = config.eps
        self.HDB_min_cluster_size = config.minimum_cluster_size

        config.groups.groups.K_Means.state = False
        config.groups.groups.Birch.state = False
        config.groups.groups.DBScan.state = False
        config.groups.groups.HDBSCAN.state = False

        if(config.Clustering == 0):
            config.groups.groups.K_Means.state = True
        if(config.Clustering == 1):
            config.groups.groups.Birch.state = True
        if(config.Clustering == 2):
            config.groups.groups.DBScan.state = True
        if(config.Clustering == 3):
            config.groups.groups.HDBSCAN.state = True

        rospy.loginfo('Parameters reconfigured.')
        return config

    # =============== OUTLINE CALCULATION ================
    def calculate_outline(self, gap):
        # extract information from the gaps
        center, vertices, simplices, volumes, size, num_of_points = gap


        # create a part outline message and fill it in with gap outline
        part_outline = imagine_common.msg.PartOutline()

        # fill in the bounding box, which is a pose
        part_outline.part_bounding_box.center.position.x = center[0]
        part_outline.part_bounding_box.center.position.y = center[1]
        part_outline.part_bounding_box.center.position.z = center[2]
        part_outline.part_bounding_box.center.orientation.x = float(0)
        part_outline.part_bounding_box.center.orientation.y = float(0)
        part_outline.part_bounding_box.center.orientation.z = float(0)
        part_outline.part_bounding_box.center.orientation.w = float(0)

        # now fill in the size in all dimensions
        part_outline.part_bounding_box.size.x = size[0]
        part_outline.part_bounding_box.size.y = size[1]
        part_outline.part_bounding_box.size.z = size[2]

        for vertex in vertices:
            outline_pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(vertex[0], vertex[1], vertex[2]), geometry_msgs.msg.Quaternion(*((float(0),)*4)))
            part_outline.outline_poses.poses.append(outline_pose)

        return part_outline

    # =============== GAP DETECTION SERVICE ===============
    def gap_detection_call(self, req):
        current_time = rospy.get_rostime().to_sec()
        pc = rospy.wait_for_message('denoised_cloud', PointCloud2)
        # try for ten frames to get an image that is more recent than the call (based on the frame time)
        for i in range(10):
            if current_time + 2 * NERIAN_FRAME_PERIOD < pc.header.stamp.to_sec():
                break
            pc = rospy.wait_for_message('denoised_cloud', PointCloud2)
        self.detected_gap.clear()
        self.detector_callback(pc)
        self.detected_gap.wait(3.)  # wait for 3 seconds

        if self.gaps is None:
            rospy.loginfo('No gaps yet, skipping.')
            return []

        # ----- PUBLISHING THE MARKERS TO RVIZ -----
        if not len(self.gaps) == 0:
            centers, vertices, simplices, volumes, sizes, num_of_points = zip(*self.gaps)

            convex_hull_marker = visualization.draw_convex_hull(simplices, self.frame_id)
            self.convex_hull_marker_pub.publish(convex_hull_marker)

            centers_marker = visualization.draw_centers(centers, self.frame_id)
            self.centers_marker_pub.publish(centers_marker)

            volume_text_marker = visualization.draw_volume_text(volumes, centers, self.frame_id)
            self.volume_text_marker_pub.publish(volume_text_marker)


        temporaryList = []

        for idx, gap in enumerate(self.gaps):
            center, vertices, simplices, volumes, sizes, num_of_points = gap

            # for every instance, create a gap object
            gapInfo = PartInfo()

            # fill in the message
            gapInfo.pose.position = geometry_msgs.msg.Point(center[0], center[1], center[2])
            gapInfo.pose.orientation = geometry_msgs.msg.Quaternion(*((float(0),)*4))
            gapInfo.part_type = "standard"
            gapInfo.part_id = "gap" + str(idx)
            gapInfo.part_outline = self.calculate_outline(gap)
            # we have the part mask as none, because we will conduct backprojection in the perception wrapper
            part_mask = sensor_msgs.msg.Image()
            gapInfo.part_outline.part_mask = part_mask
            gapInfo.part_type_confidence = 100.0
            
            # add it to the list
            temporaryList.append(gapInfo)

        response = GapArrayResponse()
        response.gap_array = temporaryList

        rospy.loginfo('Service response sent.')
        return response

    # =============== CLUSTER ALGORITHM WRAPPERS ===============
    def kmeans(self, data):
        return cluster.KMeans(n_clusters=self.KM_number_of_clusters).fit_predict(data)

    def birch(self, data):
        params = {'branching_factor': self.B_branching_factor,
                  'threshold': self.B_threshold,
                  'n_clusters': None,
                  'compute_labels': True}

        return cluster.Birch(**params).fit_predict(data)

    def dbscan(self, data):
        return cluster.DBSCAN(eps=self.DB_eps).fit_predict(data)

    def hdbscan(self, data):
        return hdbscan.HDBSCAN(min_cluster_size=self.HDB_min_cluster_size).fit_predict(data)


if __name__ == '__main__':
    rospy.init_node('ugoe_gap_detection_node', anonymous=True)
    rospy.loginfo('Detecting Gaps.')
    GapDetector()
    rospy.spin()
