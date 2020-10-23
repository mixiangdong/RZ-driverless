;; Auto-generated. Do not edit!


(when (boundp 'rfans_driver::steering_wheel_angle)
  (if (not (find-package "RFANS_DRIVER"))
    (make-package "RFANS_DRIVER"))
  (shadow 'steering_wheel_angle (find-package "RFANS_DRIVER")))
(unless (find-package "RFANS_DRIVER::STEERING_WHEEL_ANGLE")
  (make-package "RFANS_DRIVER::STEERING_WHEEL_ANGLE"))

(in-package "ROS")
;;//! \htmlinclude steering_wheel_angle.msg.html


(defclass rfans_driver::steering_wheel_angle
  :super ros::object
  :slots (_data ))

(defmethod rfans_driver::steering_wheel_angle
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64 _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rfans_driver::steering_wheel_angle :md5sum-) "fdb28210bfa9d7c91146260178d9a584")
(setf (get rfans_driver::steering_wheel_angle :datatype-) "rfans_driver/steering_wheel_angle")
(setf (get rfans_driver::steering_wheel_angle :definition-)
      "float64 data
")



(provide :rfans_driver/steering_wheel_angle "fdb28210bfa9d7c91146260178d9a584")


