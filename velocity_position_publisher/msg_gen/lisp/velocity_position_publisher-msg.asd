
(cl:in-package :asdf)

(defsystem "velocity_position_publisher-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "velocity_position_info" :depends-on ("_package_velocity_position_info"))
    (:file "_package_velocity_position_info" :depends-on ("_package"))
  ))