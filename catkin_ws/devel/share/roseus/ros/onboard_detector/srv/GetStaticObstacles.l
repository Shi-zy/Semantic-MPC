;; Auto-generated. Do not edit!


(when (boundp 'onboard_detector::GetStaticObstacles)
  (if (not (find-package "ONBOARD_DETECTOR"))
    (make-package "ONBOARD_DETECTOR"))
  (shadow 'GetStaticObstacles (find-package "ONBOARD_DETECTOR")))
(unless (find-package "ONBOARD_DETECTOR::GETSTATICOBSTACLES")
  (make-package "ONBOARD_DETECTOR::GETSTATICOBSTACLES"))
(unless (find-package "ONBOARD_DETECTOR::GETSTATICOBSTACLESREQUEST")
  (make-package "ONBOARD_DETECTOR::GETSTATICOBSTACLESREQUEST"))
(unless (find-package "ONBOARD_DETECTOR::GETSTATICOBSTACLESRESPONSE")
  (make-package "ONBOARD_DETECTOR::GETSTATICOBSTACLESRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass onboard_detector::GetStaticObstaclesRequest
  :super ros::object
  :slots (_current_position _range _semantic_filter ))

(defmethod onboard_detector::GetStaticObstaclesRequest
  (:init
   (&key
    ((:current_position __current_position) (instance geometry_msgs::Point :init))
    ((:range __range) 0.0)
    ((:semantic_filter __semantic_filter) "")
    )
   (send-super :init)
   (setq _current_position __current_position)
   (setq _range (float __range))
   (setq _semantic_filter (string __semantic_filter))
   self)
  (:current_position
   (&rest __current_position)
   (if (keywordp (car __current_position))
       (send* _current_position __current_position)
     (progn
       (if __current_position (setq _current_position (car __current_position)))
       _current_position)))
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:semantic_filter
   (&optional __semantic_filter)
   (if __semantic_filter (setq _semantic_filter __semantic_filter)) _semantic_filter)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _current_position
    (send _current_position :serialization-length)
    ;; float64 _range
    8
    ;; string _semantic_filter
    4 (length _semantic_filter)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _current_position
       (send _current_position :serialize s)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _semantic_filter
       (write-long (length _semantic_filter) s) (princ _semantic_filter s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _current_position
     (send _current_position :deserialize buf ptr-) (incf ptr- (send _current_position :serialization-length))
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _semantic_filter
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _semantic_filter (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass onboard_detector::GetStaticObstaclesResponse
  :super ros::object
  :slots (_names _ids _positions _sizes _yaws _semantic_classes _cost_weights _safety_distances ))

(defmethod onboard_detector::GetStaticObstaclesResponse
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:ids __ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:positions __positions) ())
    ((:sizes __sizes) ())
    ((:yaws __yaws) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:semantic_classes __semantic_classes) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:cost_weights __cost_weights) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:safety_distances __safety_distances) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _names __names)
   (setq _ids __ids)
   (setq _positions __positions)
   (setq _sizes __sizes)
   (setq _yaws __yaws)
   (setq _semantic_classes __semantic_classes)
   (setq _cost_weights __cost_weights)
   (setq _safety_distances __safety_distances)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:ids
   (&optional __ids)
   (if __ids (setq _ids __ids)) _ids)
  (:positions
   (&rest __positions)
   (if (keywordp (car __positions))
       (send* _positions __positions)
     (progn
       (if __positions (setq _positions (car __positions)))
       _positions)))
  (:sizes
   (&rest __sizes)
   (if (keywordp (car __sizes))
       (send* _sizes __sizes)
     (progn
       (if __sizes (setq _sizes (car __sizes)))
       _sizes)))
  (:yaws
   (&optional __yaws)
   (if __yaws (setq _yaws __yaws)) _yaws)
  (:semantic_classes
   (&optional __semantic_classes)
   (if __semantic_classes (setq _semantic_classes __semantic_classes)) _semantic_classes)
  (:cost_weights
   (&optional __cost_weights)
   (if __cost_weights (setq _cost_weights __cost_weights)) _cost_weights)
  (:safety_distances
   (&optional __safety_distances)
   (if __safety_distances (setq _safety_distances __safety_distances)) _safety_distances)
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; uint32[] _ids
    (* 4    (length _ids)) 4
    ;; geometry_msgs/Vector3[] _positions
    (apply #'+ (send-all _positions :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _sizes
    (apply #'+ (send-all _sizes :serialization-length)) 4
    ;; float64[] _yaws
    (* 8    (length _yaws)) 4
    ;; string[] _semantic_classes
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _semantic_classes)) 4
    ;; float64[] _cost_weights
    (* 8    (length _cost_weights)) 4
    ;; float64[] _safety_distances
    (* 8    (length _safety_distances)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; uint32[] _ids
     (write-long (length _ids) s)
     (dotimes (i (length _ids))
       (write-long (elt _ids i) s)
       )
     ;; geometry_msgs/Vector3[] _positions
     (write-long (length _positions) s)
     (dolist (elem _positions)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _sizes
     (write-long (length _sizes) s)
     (dolist (elem _sizes)
       (send elem :serialize s)
       )
     ;; float64[] _yaws
     (write-long (length _yaws) s)
     (dotimes (i (length _yaws))
       (sys::poke (elt _yaws i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string[] _semantic_classes
     (write-long (length _semantic_classes) s)
     (dolist (elem _semantic_classes)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _cost_weights
     (write-long (length _cost_weights) s)
     (dotimes (i (length _cost_weights))
       (sys::poke (elt _cost_weights i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _safety_distances
     (write-long (length _safety_distances) s)
     (dotimes (i (length _safety_distances))
       (sys::poke (elt _safety_distances i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; uint32[] _ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Vector3[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _positions)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _sizes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sizes (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _sizes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _yaws
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yaws (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _yaws i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string[] _semantic_classes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _semantic_classes (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _semantic_classes i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _cost_weights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cost_weights (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cost_weights i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _safety_distances
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _safety_distances (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _safety_distances i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass onboard_detector::GetStaticObstacles
  :super ros::object
  :slots ())

(setf (get onboard_detector::GetStaticObstacles :md5sum-) "720433b0b9d7c4c8bbc6acba25184779")
(setf (get onboard_detector::GetStaticObstacles :datatype-) "onboard_detector/GetStaticObstacles")
(setf (get onboard_detector::GetStaticObstacles :request) onboard_detector::GetStaticObstaclesRequest)
(setf (get onboard_detector::GetStaticObstacles :response) onboard_detector::GetStaticObstaclesResponse)

(defmethod onboard_detector::GetStaticObstaclesRequest
  (:response () (instance onboard_detector::GetStaticObstaclesResponse :init)))

(setf (get onboard_detector::GetStaticObstaclesRequest :md5sum-) "720433b0b9d7c4c8bbc6acba25184779")
(setf (get onboard_detector::GetStaticObstaclesRequest :datatype-) "onboard_detector/GetStaticObstaclesRequest")
(setf (get onboard_detector::GetStaticObstaclesRequest :definition-)
      "# Request
geometry_msgs/Point current_position
float64 range
string semantic_filter  # Optional filter by semantic class (e.g., \"WALL\", \"FURNITURE\", empty for all)

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
# Response
# Array of static obstacles using standard ROS messages
string[] names
uint32[] ids
geometry_msgs/Vector3[] positions
geometry_msgs/Vector3[] sizes
float64[] yaws
string[] semantic_classes
float64[] cost_weights
float64[] safety_distances 

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
")

(setf (get onboard_detector::GetStaticObstaclesResponse :md5sum-) "720433b0b9d7c4c8bbc6acba25184779")
(setf (get onboard_detector::GetStaticObstaclesResponse :datatype-) "onboard_detector/GetStaticObstaclesResponse")
(setf (get onboard_detector::GetStaticObstaclesResponse :definition-)
      "# Request
geometry_msgs/Point current_position
float64 range
string semantic_filter  # Optional filter by semantic class (e.g., \"WALL\", \"FURNITURE\", empty for all)

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
# Response
# Array of static obstacles using standard ROS messages
string[] names
uint32[] ids
geometry_msgs/Vector3[] positions
geometry_msgs/Vector3[] sizes
float64[] yaws
string[] semantic_classes
float64[] cost_weights
float64[] safety_distances 

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
")



(provide :onboard_detector/GetStaticObstacles "720433b0b9d7c4c8bbc6acba25184779")


