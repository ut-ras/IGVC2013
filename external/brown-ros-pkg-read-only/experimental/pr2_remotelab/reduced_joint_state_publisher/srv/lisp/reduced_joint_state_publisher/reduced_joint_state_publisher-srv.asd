
(in-package :asdf)

(defsystem "reduced_joint_state_publisher-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "RequestedJointStates" :depends-on ("_package"))
    (:file "_package_RequestedJointStates" :depends-on ("_package"))
    ))
