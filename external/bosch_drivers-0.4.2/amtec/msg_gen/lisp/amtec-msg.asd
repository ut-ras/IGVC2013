
(cl:in-package :asdf)

(defsystem "amtec-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AmtecState" :depends-on ("_package_AmtecState"))
    (:file "_package_AmtecState" :depends-on ("_package"))
  ))