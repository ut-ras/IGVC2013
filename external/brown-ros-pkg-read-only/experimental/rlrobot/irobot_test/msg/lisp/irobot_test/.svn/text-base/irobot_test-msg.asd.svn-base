
(in-package :asdf)

(defsystem "irobot_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "LocalizableObject" :depends-on ("_package"))
    (:file "_package_LocalizableObject" :depends-on ("_package"))
    (:file "Locations" :depends-on ("_package"))
    (:file "_package_Locations" :depends-on ("_package"))
    ))
