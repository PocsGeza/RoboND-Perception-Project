#!/usr/bin/env python

# region Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
# endregion

#Made this global, not sure why.
detected_objects = {}

# region Helper functions
# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    #Some casting here to get read of numpy versions.
    yaml_dict["test_scene_num"] = int(test_scene_num.data)
    yaml_dict["arm_name"]  = str(arm_name.data)
    yaml_dict["object_name"] = str(object_name.data)
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)

    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# endregion


# region Functions to implement
# Callback function for your Point Cloud Subscriber
# I will focus on getting this working first
def pcl_callback(pcl_msg_ros):

# region Exercise-2 TODOs:

    # region  TODO: Convert ROS msg to PCL data
    pcl_msg = ros_to_pcl(pcl_msg_ros)
    # endregion

    # region TODO: Statistical Outlier Filtering
    outlier_filter = pcl_msg.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.5
    outlier_filter.set_std_dev_mul_thresh(x)
    pcl_msg = outlier_filter.filter()
    # endregion

    # region TODO: Voxel Grid Downsampling
    vox = pcl_msg.make_voxel_grid_filter()
    LEAF_SIZE =.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    # endregion

    # region TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.3
    axis_max = 5.0
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.34
    axis_max = 1.0
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    # endregion

    # region TODO: RANSAC Plane Segmentation
    segmenter = cloud_filtered.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.015
    segmenter.set_distance_threshold(max_distance)
    inliers, coefficients = segmenter.segment()
    # endregion

    # region TODO: Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    # endregion

    # region TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    #Cluster extraction object
    eu_clust = white_cloud.make_EuclideanClusterExtraction()

    # Set parameters
    eu_clust.set_ClusterTolerance(0.01)
    eu_clust.set_MinClusterSize(50)
    eu_clust.set_MaxClusterSize(15000)

    # Search the k-d tree for clusters
    eu_clust.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = eu_clust.Extract()
    # endregion

    # region TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []
    # endregion

    # region TODO: Convert PCL data to ROS messages
    table_pcl_msg = pcl_to_ros(extracted_inliers)
    objects_pcl_msg = pcl_to_ros(extracted_outliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # endregion

    # region TODO: Publish ROS messages
    pcl_objects_pub.publish(objects_pcl_msg)
    pcl_table_pub.publish(table_pcl_msg)
    pcl_clusters_pub.publish(ros_cluster_cloud)
    # endregion

# endregion

# region Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    lastone = 0


    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = extracted_outliers.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .25

        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        det_obj = DetectedObject()
        det_obj.label = label
        det_obj.cloud = ros_cluster
        detected_objects.append(det_obj)
    # endregion

        # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)

        # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
        # Could add some logic to determine whether or not your object detections are robust
        # before calling pr2_mover()
        try:
            pr2_mover(detected_objects_list)
        except rospy.ROSInterruptException:
            pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    test_sceen_number = std_msgs.msg.Int32()
    test_sceen_number.data = 2
    object_name = std_msgs.msg.String()
    arm_selected = std_msgs.msg.String() # green = right, red = left
    pick_pose = geometry_msgs.msg.Pose()
    place_pose = geometry_msgs.msg.Pose()
    centroid = []
    counter = 0
    output_yaml = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    rospy.loginfo('Starting pr2_mover with {} objects'.format(len(object_list)))

    # Loop through the pick list
    for target in object_list_param:
        object_name.data = object_list_param[counter]['name']
        found = False
        # Get the PointCloud for a given object and obtain it's centroid
        for detected in object_list:
             if (object_name.data == detected.label):

                  points_arr = ros_to_pcl(detected.cloud).to_array()
                  centroids = (np.mean(points_arr,axis=0)[:3])
                  found = True

        if (found):

             # Create 'place_pose' for the object
             pick_pose.position.x = float(centroids[0])
             pick_pose.position.y = float(centroids[1])
             pick_pose.position.z = float(centroids[2])

             # Assign the arm to be used for pick_place
             place_pose.position.x = 0.0
             place_pose.position.z = 0.8

             if (object_list_param[counter]['group'] == "red"):
                 arm_selected.data = "left"
                 place_pose.position.y = 0.71
             else:
                 arm_selected.data = "right"
                 place_pose.position.y = -0.71

             # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
             yaml_dict = make_yaml_dict(test_sceen_number,arm_selected,object_name,pick_pose,place_pose)
             output_yaml.append(yaml_dict)

             # Wait for 'pick_place_routine' service to come up
             rospy.wait_for_service('pick_place_routine')

             try:
                 pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                 # Insert your message variables to be sent as a service request
                 #resp = pick_place_routine(test_sceen_number, object_name, arm_selected, pick_pose, place_pose)

                 #print ("Response: ",resp.success)

             except rospy.ServiceException, e:
                 print "Service call failed: %s"%e
        else:
             rospy.loginfo('Can\'t Find Object: {}'.format(object_list_param[counter]['name'] ))

        counter += 1

    # Output your request parameters into output yaml file
    send_to_yaml("output_" + str(test_sceen_number.data) + ".yaml",output_yaml)
# endregion


if __name__ == '__main__':

    # region ROS node initialization
    rospy.init_node('clustering', anonymous=False)
    # endregion

    # region Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # endregion

    # region Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers",Marker,queue_size =10)
    detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size =10)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    # endregion

    # region Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # endregion

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
