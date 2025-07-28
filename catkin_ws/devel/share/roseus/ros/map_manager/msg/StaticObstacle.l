;; Auto-generated. Do not edit!


(when (boundp 'map_manager::StaticObstacle)
  (if (not (find-package "MAP_MANAGER"))
    (make-package "MAP_MANAGER"))
  (shadow 'StaticObstacle (find-package "MAP_MANAGER")))
(unless (find-package "MAP_MANAGER::STATICOBSTACLE")
  (make-package "MAP_MANAGER::STATICOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude StaticObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass map_manager::StaticObstacle
  :super ros::object
  :slots (_header _name _id _position _size _yaw _semantic_class _cost_weight _safety_distance _color ))

(defmethod map_manager::StaticObstacle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:id __id) 0)
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    ((:yaw __yaw) 0.0)
    ((:semantic_class __semantic_class) "")
    ((:cost_weight __cost_weight) 0.0)
    ((:safety_distance __safety_distance) 0.0)
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _id (round __id))
   (setq _position __position)
   (setq _size __size)
   (setq _yaw (float __yaw))
   (setq _semantic_class (string __semantic_class))
   (setq _cost_weight (float __cost_weight))
   (setq _safety_distance (float __safety_distance))
   (setq _color __color)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:semantic_class
   (&optional __semantic_class)
   (if __semantic_class (setq _semantic_class __semantic_class)) _semantic_class)
  (:cost_weight
   (&optional __cost_weight)
   (if __cost_weight (setq _cost_weight __cost_weight)) _cost_weight)
  (:safety_distance
   (&optional __safety_distance)
   (if __safety_distance (setq _safety_distance __safety_distance)) _safety_distance)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; uint32 _id
    4
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _size
    (send _size :serialization-length)
    ;; float64 _yaw
    8
    ;; string _semantic_class
    4 (length _semantic_class)
    ;; float64 _cost_weight
    8
    ;; float64 _safety_distance
    8
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint32 _id
       (write-long _id s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _semantic_class
       (write-long (length _semantic_class) s) (princ _semantic_class s)
     ;; float64 _cost_weight
       (sys::poke _cost_weight (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _safety_distance
       (sys::poke _safety_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _semantic_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _semantic_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _cost_weight
     (setq _cost_weight (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _safety_distance
     (setq _safety_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;;
   self)
  )

(setf (get map_manager::StaticObstacle :md5sum-) "2617d1f512b745b066593a9f8010262e")
(setf (get map_manager::StaticObstacle :datatype-) "map_manager/StaticObstacle")
(setf (get map_manager::StaticObstacle :definition-)
      "std_msgs/Header header
string name
uint32 id
geometry_msgs/Vector3 position
geometry_msgs/Vector3 size
float64 yaw
string semantic_class
float64 cost_weight
float64 safety_distance
std_msgs/ColorRGBA color 
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
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

")



(provide :map_manager/StaticObstacle "2617d1f512b745b066593a9f8010262e")


