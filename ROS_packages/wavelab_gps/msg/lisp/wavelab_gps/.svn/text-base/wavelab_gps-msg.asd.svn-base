
(in-package :asdf)

(defsystem "wavelab_gps-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "gps_msg" :depends-on ("_package"))
    (:file "_package_gps_msg" :depends-on ("_package"))
    ))
