
(cl:in-package :asdf)

(defsystem "ocean_server_imu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RawData" :depends-on ("_package_RawData"))
    (:file "_package_RawData" :depends-on ("_package"))
  ))