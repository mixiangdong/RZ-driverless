;; Auto-generated. Do not edit!


(when (boundp 'rfans_driver::Point)
  (if (not (find-package "RFANS_DRIVER"))
    (make-package "RFANS_DRIVER"))
  (shadow 'Point (find-package "RFANS_DRIVER")))
(unless (find-package "RFANS_DRIVER::POINT")
  (make-package "RFANS_DRIVER::POINT"))

(in-package "ROS")
;;//! \htmlinclude Point.msg.html


(defclass rfans_driver::Point
  :super ros::object
  :slots (_x1 _y1 _x2 _y2 _a1 _a2 _a3 _a4 _b1 _b2 _b3 _b4 ))

(defmethod rfans_driver::Point
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:y1 __y1) 0.0)
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    ((:a1 __a1) 0.0)
    ((:a2 __a2) 0.0)
    ((:a3 __a3) 0.0)
    ((:a4 __a4) 0.0)
    ((:b1 __b1) 0.0)
    ((:b2 __b2) 0.0)
    ((:b3 __b3) 0.0)
    ((:b4 __b4) 0.0)
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _y1 (float __y1))
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   (setq _a1 (float __a1))
   (setq _a2 (float __a2))
   (setq _a3 (float __a3))
   (setq _a4 (float __a4))
   (setq _b1 (float __b1))
   (setq _b2 (float __b2))
   (setq _b3 (float __b3))
   (setq _b4 (float __b4))
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
  (:a1
   (&optional __a1)
   (if __a1 (setq _a1 __a1)) _a1)
  (:a2
   (&optional __a2)
   (if __a2 (setq _a2 __a2)) _a2)
  (:a3
   (&optional __a3)
   (if __a3 (setq _a3 __a3)) _a3)
  (:a4
   (&optional __a4)
   (if __a4 (setq _a4 __a4)) _a4)
  (:b1
   (&optional __b1)
   (if __b1 (setq _b1 __b1)) _b1)
  (:b2
   (&optional __b2)
   (if __b2 (setq _b2 __b2)) _b2)
  (:b3
   (&optional __b3)
   (if __b3 (setq _b3 __b3)) _b3)
  (:b4
   (&optional __b4)
   (if __b4 (setq _b4 __b4)) _b4)
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
    ;; float64 _a1
    8
    ;; float64 _a2
    8
    ;; float64 _a3
    8
    ;; float64 _a4
    8
    ;; float64 _b1
    8
    ;; float64 _b2
    8
    ;; float64 _b3
    8
    ;; float64 _b4
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
     ;; float64 _a1
       (sys::poke _a1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a2
       (sys::poke _a2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a3
       (sys::poke _a3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a4
       (sys::poke _a4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b1
       (sys::poke _b1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b2
       (sys::poke _b2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b3
       (sys::poke _b3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b4
       (sys::poke _b4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64 _a1
     (setq _a1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a2
     (setq _a2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a3
     (setq _a3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a4
     (setq _a4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b1
     (setq _b1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b2
     (setq _b2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b3
     (setq _b3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b4
     (setq _b4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rfans_driver::Point :md5sum-) "c81b1cf288dd128754d09c9e4bbb83cf")
(setf (get rfans_driver::Point :datatype-) "rfans_driver/Point")
(setf (get rfans_driver::Point :definition-)
      "#存储距离雷达最近的两对锥桶及其中心点的坐标

float64 x1
float64 y1
float64 x2
float64 y2
float64 a1
float64 a2
float64 a3
float64 a4
float64 b1
float64 b2
float64 b3
float64 b4







")



(provide :rfans_driver/Point "c81b1cf288dd128754d09c9e4bbb83cf")


