; Auto-generated. Do not edit!


(cl:in-package map_manager-msg)


;//! \htmlinclude StaticObstacleArray.msg.html

(cl:defclass <StaticObstacleArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector map_manager-msg:StaticObstacle)
   :initform (cl:make-array 0 :element-type 'map_manager-msg:StaticObstacle :initial-element (cl:make-instance 'map_manager-msg:StaticObstacle))))
)

(cl:defclass StaticObstacleArray (<StaticObstacleArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StaticObstacleArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StaticObstacleArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name map_manager-msg:<StaticObstacleArray> is deprecated: use map_manager-msg:StaticObstacleArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StaticObstacleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_manager-msg:header-val is deprecated.  Use map_manager-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <StaticObstacleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_manager-msg:obstacles-val is deprecated.  Use map_manager-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StaticObstacleArray>) ostream)
  "Serializes a message object of type '<StaticObstacleArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StaticObstacleArray>) istream)
  "Deserializes a message object of type '<StaticObstacleArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'map_manager-msg:StaticObstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StaticObstacleArray>)))
  "Returns string type for a message object of type '<StaticObstacleArray>"
  "map_manager/StaticObstacleArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StaticObstacleArray)))
  "Returns string type for a message object of type 'StaticObstacleArray"
  "map_manager/StaticObstacleArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StaticObstacleArray>)))
  "Returns md5sum for a message object of type '<StaticObstacleArray>"
  "61c9bb1e78d8bdd31690ec29054c93df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StaticObstacleArray)))
  "Returns md5sum for a message object of type 'StaticObstacleArray"
  "61c9bb1e78d8bdd31690ec29054c93df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StaticObstacleArray>)))
  "Returns full string definition for message of type '<StaticObstacleArray>"
  (cl:format cl:nil "std_msgs/Header header~%StaticObstacle[] obstacles ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: map_manager/StaticObstacle~%std_msgs/Header header~%string name~%uint32 id~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 size~%float64 yaw~%string semantic_class~%float64 cost_weight~%float64 safety_distance~%std_msgs/ColorRGBA color ~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StaticObstacleArray)))
  "Returns full string definition for message of type 'StaticObstacleArray"
  (cl:format cl:nil "std_msgs/Header header~%StaticObstacle[] obstacles ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: map_manager/StaticObstacle~%std_msgs/Header header~%string name~%uint32 id~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 size~%float64 yaw~%string semantic_class~%float64 cost_weight~%float64 safety_distance~%std_msgs/ColorRGBA color ~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StaticObstacleArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StaticObstacleArray>))
  "Converts a ROS message object to a list"
  (cl:list 'StaticObstacleArray
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
