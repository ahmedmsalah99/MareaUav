# AprilTag 2 ROS2 Node

This ROS2 node uses the AprilTag 2 library to detect AprilTags in images and publish their pose, id and additional metadata.

For more information on AprilTag 2, the paper and the reference implementation: https://april.eecs.umich.edu/software/apriltag.html

In addition, this ros2 package depends on `apriltag` package. For download please refer to `git clone https://NicoNelli@bitbucket.org/isme_robotics/mar_frankenrotor_ros.git`
README.md.


The source file as reference is `AprilTag2Node2_0.cpp`.

## Topics

### Subscriptions:
- `/apriltag/image/compressed` (type: `sensor_msgs/CompressedImage`, format: `"jpeg"` or `"jpg"`)

### Publisher:
- `/platform_center` (type: `geometry_msgs/TransformStamped`)

The camera intrinsics `K` is used to compute the marker tag pose `T` from the homography `H`. 

The relative pose between platform center and the drone is published on the topic `/platform_center`.
The platform center is estimated knowing the transformation matrices between each tag respect to the tag in the center. 
Subsequently, the pose respect to the camera is transferred on drone frame (ENU).

These information are stored in : `include/LandingPlatform.h`

## Configuration

The node is configured via a yaml configurations file. For the complete ROS yaml parameter file syntax, see: https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser.

The file has the format:
```YAML
apriltag:                           # namespace
    apriltag2:                      # node name
        ros__parameters:
            # required
            family: <tag family>    # tag family name
            size: <tag edge size>   # tag edge size in meter
            z_up: true              # rotate about x-axis to have Z pointing upwards

            # (optional) tuning of detection
            max_hamming: 0          # maximum allowed hamming distance (corrected bits)
            decimate: 1.0           # decimate resolution for quad detection
            blur: 0.0               # sigma of Gaussian blur for quad detection
            refine-edges: 1         # snap to strong gradients
            threads: 1              # number of threads
            debug: 0                # write additional debugging images to current working directory

            # (optional) list of tags
            tag_lists:
                1: 0.195
                2: 0.195
                3: 0.195
                4: 0.195
                5: 0.025
                6: 0.035
                7: 0.035
                8: 0.035
                9: 0.035
```

The parameters `family` and `size` are required as default dimension. 
`family` (string) defines the tag family for the detector and must be one of 
`16h5`, `25h7`, `25h9`, `36h10`, `36h11`, `36artoolkit`. `size` (float) is the tag edge size in meters, assuming square markers.

`tag_lists` is list containing the  IDs' tags on the platform and their dimensions in meters, assuming square markers.
The dimension is related to the black square.

The remaining parameters are set to the their default values from the library. 
See `apriltag.h` for a more detailed description of their function.

See [tags_16h5_all.yaml](node/cfg/tags_16h5_all.yaml) for 
an example configuration that publishes all markers in the 16h5 family and [tags_16h5_filtered.yaml](node/cfg/tags_16h5_filtered.yaml)
for filtering tags.

To run the node and load the configuration, pass the configuration file to `__params`:
```bash
ros2 run apriltag2_node apriltag2_node \
    /apriltag/image/compressed:=/camera/image/compressed \
    __params:=`ros2 pkg prefix apriltag2_node`/share/apriltag2_node/cfg/platform.yaml
```