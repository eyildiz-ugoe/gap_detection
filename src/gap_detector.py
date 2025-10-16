#!/usr/bin/env python

import visualization
import helpers
import gap_detection_core

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
        self._gap_point_counts = []
        self._volume_correction_counts = []

    # =============== MAIN DETECTION LOOP ===============
    def detector_callback(self, preprocessed_pc):
        self.image_received.set()
        cloud_points = helpers.PC2_to_numpy_array(preprocessed_pc)
        self.frame_id = preprocessed_pc.header.frame_id

        detector_params = gap_detection_core.DetectorParameters(
            depth_axis=self.depth_axis,
            automatic_thresholding=self.automatic_thresholding,
            otsu_bins=self.otsu_bins)

        clustering_params = gap_detection_core.ClusteringParameters(
            method=gap_detection_core.ClusteringMethod.from_value(self.clustering),
            kmeans_clusters=self.KM_number_of_clusters,
            birch_branching_factor=self.B_branching_factor,
            birch_threshold=self.B_threshold,
            dbscan_eps=self.DB_eps,
            hdbscan_min_cluster_size=self.HDB_min_cluster_size)

        try:
            detection_result = gap_detection_core.detect_gaps(
                cloud_points, detector_params, clustering_params)
        except gap_detection_core.GapDetectionError as exc:
            rospy.logwarn(str(exc))
            self.gaps = []
            self._gap_point_counts = []
            self._volume_correction_counts = []
            self.detected_gap.set()
            return

        self.surface_height = detection_result.surface_height

        potential_gaps = helpers.numpy_array_to_PC2(
            detection_result.potential_gap_points, self.frame_id)
        self.potential_gaps_pub.publish(potential_gaps)

        self.gaps = []
        self._gap_point_counts = []
        self._volume_correction_counts = []

        for gap in detection_result.gaps:
            gap_info = [gap.center, gap.vertices, gap.simplices,
                        gap.volume, gap.size, gap.total_points]
            gap_volume_cm3 = gap.volume_cm3

            if(self.min_gap_volume <= gap_volume_cm3 <= self.max_gap_volume):
                self.gaps.append(gap_info)
                self._gap_point_counts.append(gap.original_points)
                self._volume_correction_counts.append(gap.volume_correction_points)

        # ----- EVALUATION -----
        if(self.create_evaluation and self.gaps):
            corrected_counts = [original - correction for original, correction
                                in zip(self._gap_point_counts,
                                       self._volume_correction_counts)]
            try:
                helpers.evaluate_detector(corrected_counts)
            except RuntimeError as exc:
                rospy.logwarn(str(exc))
            self.create_evaluation = False

        self.detected_gap.set()  # signal successful gap detection


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

if __name__ == '__main__':
    rospy.init_node('ugoe_gap_detection_node', anonymous=True)
    rospy.loginfo('Detecting Gaps.')
    GapDetector()
    rospy.spin()
