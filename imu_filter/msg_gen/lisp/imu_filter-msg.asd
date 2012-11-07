
(cl:in-package :asdf)

(defsystem "imu_filter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FilteredIMUData" :depends-on ("_package_FilteredIMUData"))
    (:file "_package_FilteredIMUData" :depends-on ("_package"))
  ))