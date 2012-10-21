
(cl:in-package :asdf)

(defsystem "gumstix_memread_bma180-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "bma180meas" :depends-on ("_package_bma180meas"))
    (:file "_package_bma180meas" :depends-on ("_package"))
  ))