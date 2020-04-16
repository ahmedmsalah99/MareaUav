;; Auto-generated. Do not edit!


(when (boundp 'apriltag_ros::AprilTagDetection)
  (if (not (find-package "APRILTAG_ROS"))
    (make-package "APRILTAG_ROS"))
  (shadow 'AprilTagDetection (find-package "APRILTAG_ROS")))
(unless (find-package "APRILTAG_ROS::APRILTAGDETECTION")
  (make-package "APRILTAG_ROS::APRILTAGDETECTION"))

(in-package "ROS")
;;//! \htmlinclude AprilTagDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass apriltag_ros::AprilTagDetection
  :super ros::object
  :slots (_id _size _pose _corners2d ))

(defmethod apriltag_ros::AprilTagDetection
  (:init
   (&key
    ((:id __id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:size __size) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pose __pose) (instance geometry_msgs::PoseWithCovarianceStamped :init))
    ((:corners2d __corners2d) (let (r) (dotimes (i 4) (push (instance geometry_msgs::Point32 :init) r)) r))
    )
   (send-super :init)
   (setq _id __id)
   (setq _size __size)
   (setq _pose __pose)
   (setq _corners2d __corners2d)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:corners2d
   (&rest __corners2d)
   (if (keywordp (car __corners2d))
       (send* _corners2d __corners2d)
     (progn
       (if __corners2d (setq _corners2d (car __corners2d)))
       _corners2d)))
  (:serialization-length
   ()
   (+
    ;; int32[] _id
    (* 4    (length _id)) 4
    ;; float64[] _size
    (* 8    (length _size)) 4
    ;; geometry_msgs/PoseWithCovarianceStamped _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Point32[4] _corners2d
    (apply #'+ (send-all _corners2d :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _id
     (write-long (length _id) s)
     (dotimes (i (length _id))
       (write-long (elt _id i) s)
       )
     ;; float64[] _size
     (write-long (length _size) s)
     (dotimes (i (length _size))
       (sys::poke (elt _size i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/PoseWithCovarianceStamped _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Point32[4] _corners2d
     (dolist (elem _corners2d)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float64[] _size
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _size (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _size i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/PoseWithCovarianceStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Point32[4] _corners2d
   (dotimes (i 4)
     (send (elt _corners2d i) :deserialize buf ptr-) (incf ptr- (send (elt _corners2d i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get apriltag_ros::AprilTagDetection :md5sum-) "a56cd17428cca3ad284869c18e65bd8c")
(setf (get apriltag_ros::AprilTagDetection :datatype-) "apriltag_ros/AprilTagDetection")
(setf (get apriltag_ros::AprilTagDetection :definition-)
      "# Tag ID(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the IDs of each tag in the bundle.
int32[] id

# Tag size(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the sizes of each tag in the bundle, in the same
# order as the IDs above.
float64[] size

# Pose in the camera frame, obtained from homography transform. If a standalone
# tag, the homography is from the four tag corners. If a tag bundle, the
# homography is from at least the four corners of one member tag and at most the
# four corners of all member tags.
geometry_msgs/PoseWithCovarianceStamped pose

geometry_msgs/Point32[4] corners2d  # AprilTag corners in image

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
")



(provide :apriltag_ros/AprilTagDetection "a56cd17428cca3ad284869c18e65bd8c")


