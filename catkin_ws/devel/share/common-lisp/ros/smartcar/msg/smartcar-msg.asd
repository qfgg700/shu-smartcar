
(cl:in-package :asdf)

(defsystem "smartcar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "aimPoint" :depends-on ("_package_aimPoint"))
    (:file "_package_aimPoint" :depends-on ("_package"))
  ))