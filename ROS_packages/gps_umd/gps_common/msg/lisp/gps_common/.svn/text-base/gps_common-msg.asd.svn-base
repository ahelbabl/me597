
(in-package :asdf)

(defsystem "gps_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "GPSFix" :depends-on ("_package"))
    (:file "_package_GPSFix" :depends-on ("_package"))
    (:file "GPSStatus" :depends-on ("_package"))
    (:file "_package_GPSStatus" :depends-on ("_package"))
    ))
