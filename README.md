## Project: Perception Pick & Place
---
# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

[image1]: Images/table.PNG
[image2]: Images/items_without_table.PNG
[image4]: Images/training.PNG
[image5]: Images/figure_1-RGB.png
[image6]: Images/figure_1-HSV.png
[image7]: Images/figure_1-1.png
[image8]: Images/figure_22.png
[image9]: Images/figure_3.png
[image10]: Images/items_1.PNG
[image11]: Images/items_2.PNG
[image12]: Images/items_3_2.PNG


---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.


![alt text][image1]
![alt text][image2]

In Exercise 1 the goal is to filter a point cloud data based on known information. To do it first downsample the data so it becomes easier to compute.Then, spasify the range from the view that the objects are in. Here the range in z axis is between (0.6,1.1) and y axis (-0.45,0.45).Lastly, Random Sample Consensus (RANSAC) is used to saparate the items from the table. The comments on the fallowing code explain the steps:

```python
#note: this code from the project code from callback method:
# TODO: Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)

# Voxel Grid Downsampling
vox = cloud.make_voxel_grid_filter()
LEAF_SIZE = 0.005 #0.01   
# Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
cloud_filtered = vox.filter()

# PassThrough Filter
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6 
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()

#To remove the two boxes for the sides from the view of the robot
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
axis_min = -0.45 
axis_max = 0.45 
passthrough.set_filter_limits(axis_min, axis_max)

# Finally use the filter function to obtain the resultant point cloud.
cloud_filtered = passthrough.filter()

outlier_filter = cloud_filtered.make_statistical_outlier_filter()
# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(10)

# Set threshold scale factor
x = .005 #1.0

# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()

# RANSAC Plane Segmentation
# Create the segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance 
# for segmenting the table
max_distance = 0.006 #was 0.01
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()

# Extract inliers and outliers
# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
```


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

In Exercise 2 the filterd point cloud data used to cluster each item data togother. First, the point cloud data is converted from RGB XYZ to XYZ because the data is clusterd based on postion. By using Euclidean Clustering algorithm the data is clustered with a specific minimam points per cluster and a maximam. There is tolerances for distance threshold. the fallowing code is presented with comments for explanation:

```python

# Euclidean Clustering
white_cloud = XYZRGB_to_XYZ(extracted_outliers)# Apply function to convert XYZRGB to XYZ
tree = white_cloud.make_kdtree()

# Create Cluster-Mask Point Cloud to visualize each cluster separately
# Create a cluster extraction object
ec = white_cloud.make_EuclideanClusterExtraction()
# Set tolerances for distance threshold 
ec.set_ClusterTolerance(0.01) #0.04
# Minimum and maximum cluster size (in points)
ec.set_MinClusterSize(10) 
ec.set_MaxClusterSize(9500)
# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()

cluster_color = get_color_list(len(cluster_indices))

color_cluster_point_list = []

for j, indices in enumerate(cluster_indices):
for i, indice in enumerate(indices):
    color_cluster_point_list.append([white_cloud[indice][0],
                                white_cloud[indice][1],
                                white_cloud[indice][2],
                                 rgb_to_float(cluster_color[j])])

# Create new cloud containing all clusters, each with unique color
cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)

# Convert PCL data to ROS messages
ros_cloud_objects = pcl_to_ros(extracted_outliers)
ros_cloud_table = pcl_to_ros(extracted_inliers)
ros_cluster_cloud = pcl_to_ros(cluster_cloud)

# Publish ROS messages
pcl_cluster_pub.publish(ros_cluster_cloud)
pcl_objects_pub.publish(ros_cloud_objects)
pcl_table_pub.publish(ros_cloud_table)

```

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

The point cloud data is filterd , and grouped into clusters now its time to recognize each cluster. to do Classification there first must be a feature data, and a teained classifire. to generate features for each item the sensor stick world is launched, then each item appers with rondomized angles. as shown in the image below:

![alt text][image4]

the generated features data is used to train a classfire. SVM is the classifire used in this program. I have tested traing SVM using RGB and HSV and with diffrant number of iterations, here are the results: 

![alt text][image5]
![alt text][image6]

>> Object recognition steps have been implemented in the pcl_callback() function within template Python script. 

After that using the genarated model from SVM, the code go throuh each cluster and and add them to detected objects list. here is the code for that: 

```python

# Classify the clusters! (loop through each detected cluster one at a time)
detected_objects_labels = []
detected_objects = []
for index, pts_list in enumerate(cluster_indices):
    # Grab the points for the cluster from the extracted outliers (cloud_objects)
    pcl_cluster = extracted_outliers.extract(pts_list)
    # Convert the cluster from pcl to ROS using helper function
    ros_cluster = pcl_to_ros(pcl_cluster)
    
    # Extract histogram features
    #change sample_cloud to ros_cluster
    chists = compute_color_histograms(ros_cluster, using_hsv=True)
    normals = get_normals(ros_cluster)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists, nhists))
    # comm ---- labeled_features.append([feature, model_name])
    # Make the prediction, retrieve the label for the result
    # and add it to detected_objects_labels list
    prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
    label = encoder.inverse_transform(prediction)[0]
    detected_objects_labels.append(label)

    # Publish a label into RViz
    label_pos = list(white_cloud[pts_list[0]])
    label_pos[2] += .25 #was 0.4
    object_markers_pub.publish(make_label(label,label_pos, index))

    # Add the detected object to the list of detected objects.
    do = DetectedObject()
    do.label = label
    do.cloud = ros_cluster
    detected_objects.append(do)

rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

# Publish the list of detected objects
# This is the output you'll need to complete the upcoming project!
detected_objects_pub.publish(detected_objects)
```
Here are the SVM results of each items list for the project:

### list 1:
![alt text][image7]
### list 2:
![alt text][image8]
### list 3:
![alt text][image9]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Here are images of each items set that shows how the robots classefied them. first set (3/3) secound set (4/5) last set (8/8):

![alt text][image10]
![alt text][image11]
![alt text][image12]

Here is the code for generating the `.yaml` files which are uploded in this robository. 

```python
def pr2_mover(object_list):

# Initialize variables
TEST_SCENE_NUM = Int32()
TEST_SCENE_NUM.data = 3 
OBJECT_NAME = String()
WHICH_ARM = String() 
PICK_POSE = Pose()
PLACE_POSE = Pose()

labels = []
centroids = [] # to be list of tuples (x, y, z)
output_yaml = []

# Get/Read parameters
# get parameters
object_list_param = rospy.get_param('/object_list')

#  Parse parameters into individual variables
#object_name = object_list_param[i]['name']
#object_group = object_list_param[i]['group']

# Rotate PR2 in place to capture side tables for the collision map

# Loop through the pick list
for object in object_list:
    labels.append(object.label)
    # TODO: Get the PointCloud for a given object and obtain it's centroid
    points_arr = ros_to_pcl(object.cloud).to_array()
    centroids.append(np.mean(points_arr, axis=0)[:3])

for i in range(0,len(object_list_param)):

    OBJECT_NAME.data = object_list_param[i]['name']
    #  Create 'place_pose' for the object
    PICK_POSE.position.x = np.asscalar(centroids[i][0])
    PICK_POSE.position.y = np.asscalar(centroids[i][1])
    PICK_POSE.position.z = np.asscalar(centroids[i][2])

    #  Assign the arm to be used for pick_place
    PLACE_POSE.position.x = 0.0
    PLACE_POSE.position.z = 0.605

    if(object_list_param[i]['group'] == 'red'):
        PLACE_POSE.position.y = 0.71
        WHICH_ARM.data = "left"
    else:
        PLACE_POSE.position.y = -0.71
        WHICH_ARM.data = "right"


    #  Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
    yaml_dict = make_yaml_dict(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
    output_yaml.append(yaml_dict)

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        #  Insert your message variables to be sent as a service request
        resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Output your request parameters into output yaml file
    yaml_filename = 'output_'+str(TEST_SCENE_NUM.data)+'.yaml'
    send_to_yaml(yaml_filename,output_yaml)
```
