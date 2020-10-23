;; Auto-generated. Do not edit!


(when (boundp 'rfans_driver::point_zx)
  (if (not (find-package "RFANS_DRIVER"))
    (make-package "RFANS_DRIVER"))
  (shadow 'point_zx (find-package "RFANS_DRIVER")))
(unless (find-package "RFANS_DRIVER::POINT_ZX")
  (make-package "RFANS_DRIVER::POINT_ZX"))

(in-package "ROS")
;;//! \htmlinclude point_zx.msg.html


(defclass rfans_driver::point_zx
  :super ros::object
  :slots (_x1 _y1 _x2 _y2 _x3 _y3 _x4 _y4 _x5 _y5 _x6 _y6 ))

(defmethod rfans_driver::point_zx
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:y1 __y1) 0.0)
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    ((:x3 __x3) 0.0)
    ((:y3 __y3) 0.0)
    ((:x4 __x4) 0.0)
    ((:y4 __y4) 0.0)
    ((:x5 __x5) 0.0)
    ((:y5 __y5) 0.0)
    ((:x6 __x6) 0.0)
    ((:y6 __y6) 0.0)
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _y1 (float __y1))
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   (setq _x3 (float __x3))
   (setq _y3 (float __y3))
   (setq _x4 (float __x4))
   (setq _y4 (float __y4))
   (setq _x5 (float __x5))
   (setq _y5 (float __y5))
   (setq _x6 (float __x6))
   (setq _y6 (float __y6))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:y3
   (&optional __y3)
   (if __y3 (setq _y3 __y3)) _y3)
  (:x4
   (&optional __x4)
   (if __x4 (setq _x4 __x4)) _x4)
  (:y4
   (&optional __y4)
   (if __y4 (setq _y4 __y4)) _y4)
  (:x5
   (&optional __x5)
   (if __x5 (setq _x5 __x5)) _x5)
  (:y5
   (&optional __y5)
   (if __y5 (setq _y5 __y5)) _y5)
  (:x6
   (&optional __x6)
   (if __x6 (setq _x6 __x6)) _x6)
  (:y6
   (&optional __y6)
   (if __y6 (setq _y6 __y6)) _y6)
  (:serialization-length
   ()
   (+
    ;; float64 _x1
    8
    ;; float64 _y1
    8
    ;; float64 _x2
    8
    ;; float64 _y2
    8
    ;; float64 _x3
    8
    ;; float64 _y3
    8
    ;; float64 _x4
    8
    ;; float64 _y4
    8
    ;; float64 _x5
    8
    ;; float64 _y5
    8
    ;; float64 _x6
    8
    ;; float64 _y6
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x1
       (sys::poke _x1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y1
       (sys::poke _y1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y2
       (sys::poke _y2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x3
       (sys::poke _x3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y3
       (sys::poke _y3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x4
       (sys::poke _x4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y4
       (sys::poke _y4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x5
       (sys::poke _x5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y5
       (sys::poke _y5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x6
       (sys::poke _x6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y6
       (sys::poke _y6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x1
     (setq _x1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y1
     (setq _y1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x2
     (setq _x2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y2
     (setq _y2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x3
     (setq _x3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y3
     (setq _y3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x4
     (setq _x4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y4
     (setq _y4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x5
     (setq _x5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y5
     (setq _y5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x6
     (setq _x6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y6
     (setq _y6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rfans_driver::point_zx :md5sum-) "954e85bbf6ced39ad18ebcb8b2458d7e")
(setf (get rfans_driver::point_zx :datatype-) "rfans_driver/point_zx")
(setf (get rfans_driver::point_zx :definition-)
      "#存储锥桶xy信息，无顺序

float64 x1
float64 y1
float64 x2
float64 y2
float64 x3
float64 y3
float64 x4
float64 y4
float64 x5
float64 y5
float64 x6
float64 y6

")



(provide :rfans_driver/point_zx "954e85bbf6ced39ad18ebcb8b2458d7e")


