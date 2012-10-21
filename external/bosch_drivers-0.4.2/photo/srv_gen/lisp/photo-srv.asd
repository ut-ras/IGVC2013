
(cl:in-package :asdf)

(defsystem "photo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetConfig" :depends-on ("_package_GetConfig"))
    (:file "_package_GetConfig" :depends-on ("_package"))
    (:file "SetConfig" :depends-on ("_package_SetConfig"))
    (:file "_package_SetConfig" :depends-on ("_package"))
    (:file "Capture" :depends-on ("_package_Capture"))
    (:file "_package_Capture" :depends-on ("_package"))
  ))