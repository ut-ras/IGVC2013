
(in-package :asdf)

(defsystem "sogp_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sogp_node-msg)
  :components ((:file "_package")
    (:file "Reset" :depends-on ("_package"))
    (:file "_package_Reset" :depends-on ("_package"))
    (:file "PredictMatrix" :depends-on ("_package"))
    (:file "_package_PredictMatrix" :depends-on ("_package"))
    (:file "SetParameters" :depends-on ("_package"))
    (:file "_package_SetParameters" :depends-on ("_package"))
    (:file "PredictVector" :depends-on ("_package"))
    (:file "_package_PredictVector" :depends-on ("_package"))
    ))
