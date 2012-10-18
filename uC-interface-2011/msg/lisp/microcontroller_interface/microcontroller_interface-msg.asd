
(in-package :asdf)

(defsystem "microcontroller_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Telemetry" :depends-on ("_package"))
    (:file "_package_Telemetry" :depends-on ("_package"))
    ))
