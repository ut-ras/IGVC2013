
(cl:in-package :asdf)

(defsystem "ocean_server_imu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "custom_cool_msg" :depends-on ("_package_custom_cool_msg"))
    (:file "_package_custom_cool_msg" :depends-on ("_package"))
    (:file "RawData" :depends-on ("_package_RawData"))
    (:file "_package_RawData" :depends-on ("_package"))
  ))