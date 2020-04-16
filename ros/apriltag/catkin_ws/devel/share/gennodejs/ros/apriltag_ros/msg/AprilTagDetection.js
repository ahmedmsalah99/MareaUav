// Auto-generated. Do not edit!

// (in-package apriltag_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class AprilTagDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.size = null;
      this.pose = null;
      this.corners2d = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = [];
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = [];
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovarianceStamped();
      }
      if (initObj.hasOwnProperty('corners2d')) {
        this.corners2d = initObj.corners2d
      }
      else {
        this.corners2d = new Array(4).fill(new geometry_msgs.msg.Point32());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AprilTagDetection
    // Serialize message field [id]
    bufferOffset = _arraySerializer.int32(obj.id, buffer, bufferOffset, null);
    // Serialize message field [size]
    bufferOffset = _arraySerializer.float64(obj.size, buffer, bufferOffset, null);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovarianceStamped.serialize(obj.pose, buffer, bufferOffset);
    // Check that the constant length array field [corners2d] has the right length
    if (obj.corners2d.length !== 4) {
      throw new Error('Unable to serialize array field corners2d - length must be 4')
    }
    // Serialize message field [corners2d]
    obj.corners2d.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AprilTagDetection
    let len;
    let data = new AprilTagDetection(null);
    // Deserialize message field [id]
    data.id = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [size]
    data.size = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovarianceStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [corners2d]
    len = 4;
    data.corners2d = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.corners2d[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.id.length;
    length += 8 * object.size.length;
    length += geometry_msgs.msg.PoseWithCovarianceStamped.getMessageSize(object.pose);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'apriltag_ros/AprilTagDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a56cd17428cca3ad284869c18e65bd8c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Tag ID(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AprilTagDetection(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = []
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovarianceStamped.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovarianceStamped()
    }

    if (msg.corners2d !== undefined) {
      resolved.corners2d = new Array(4)
      for (let i = 0; i < resolved.corners2d.length; ++i) {
        if (msg.corners2d.length > i) {
          resolved.corners2d[i] = geometry_msgs.msg.Point32.Resolve(msg.corners2d[i]);
        }
        else {
          resolved.corners2d[i] = new geometry_msgs.msg.Point32();
        }
      }
    }
    else {
      resolved.corners2d = new Array(4).fill(new geometry_msgs.msg.Point32())
    }

    return resolved;
    }
};

module.exports = AprilTagDetection;
