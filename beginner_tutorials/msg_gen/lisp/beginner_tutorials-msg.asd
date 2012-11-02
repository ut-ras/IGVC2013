
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "custom_cool_msg" :depends-on ("_package_custom_cool_msg"))
    (:file "_package_custom_cool_msg" :depends-on ("_package"))
  ))