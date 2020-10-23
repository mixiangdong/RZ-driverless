
(cl:in-package :asdf)

(defsystem "rfans_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Point" :depends-on ("_package_Point"))
    (:file "_package_Point" :depends-on ("_package"))
    (:file "gps_data" :depends-on ("_package_gps_data"))
    (:file "_package_gps_data" :depends-on ("_package"))
    (:file "lidarpoint" :depends-on ("_package_lidarpoint"))
    (:file "_package_lidarpoint" :depends-on ("_package"))
    (:file "point_zx" :depends-on ("_package_point_zx"))
    (:file "_package_point_zx" :depends-on ("_package"))
    (:file "steering_wheel_angle" :depends-on ("_package_steering_wheel_angle"))
    (:file "_package_steering_wheel_angle" :depends-on ("_package"))
  ))