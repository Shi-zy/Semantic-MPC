
(cl:in-package :asdf)

(defsystem "map_manager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "StaticObstacle" :depends-on ("_package_StaticObstacle"))
    (:file "_package_StaticObstacle" :depends-on ("_package"))
    (:file "StaticObstacleArray" :depends-on ("_package_StaticObstacleArray"))
    (:file "_package_StaticObstacleArray" :depends-on ("_package"))
  ))