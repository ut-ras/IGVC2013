
(cl:in-package :asdf)

(defsystem "smi540-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "smi540meas" :depends-on ("_package_smi540meas"))
    (:file "_package_smi540meas" :depends-on ("_package"))
  ))