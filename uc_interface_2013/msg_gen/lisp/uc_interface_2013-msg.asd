
(cl:in-package :asdf)

(defsystem "uc_interface_2013-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Telemetry" :depends-on ("_package_Telemetry"))
    (:file "_package_Telemetry" :depends-on ("_package"))
  ))