
(cl:in-package :asdf)

(defsystem "gumstix_memwrite_bma180-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "writermsg" :depends-on ("_package_writermsg"))
    (:file "_package_writermsg" :depends-on ("_package"))
  ))