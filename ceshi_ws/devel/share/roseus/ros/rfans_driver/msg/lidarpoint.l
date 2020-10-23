;; Auto-generated. Do not edit!


(when (boundp 'rfans_driver::lidarpoint)
  (if (not (find-package "RFANS_DRIVER"))
    (make-package "RFANS_DRIVER"))
  (shadow 'lidarpoint (find-package "RFANS_DRIVER")))
(unless (find-package "RFANS_DRIVER::LIDARPOINT")
  (make-package "RFANS_DRIVER::LIDARPOINT"))

(in-package "ROS")
;;//! \htmlinclude lidarpoint.msg.html


(defclass rfans_driver::lidarpoint
  :super ros::object
  :slots (_A_X _A_Y _B_X _B_Y _a_x _a_y _b_x _b_y _Aa_Xx _Aa_Yy _Bb_Xx _Bb_Yy ))

(defmethod rfans_driver::lidarpoint
  (:init
   (&key
    ((:A_X __A_X) 0.0)
    ((:A_Y __A_Y) 0.0)
    ((:B_X __B_X) 0.0)
    ((:B_Y __B_Y) 0.0)
    ((:a_x __a_x) 0.0)
    ((:a_y __a_y) 0.0)
    ((:b_x __b_x) 0.0)
    ((:b_y __b_y) 0.0)
    ((:Aa_Xx __Aa_Xx) 0.0)
    ((:Aa_Yy __Aa_Yy) 0.0)
    ((:Bb_Xx __Bb_Xx) 0.0)
    ((:Bb_Yy __Bb_Yy) 0.0)
    )
   (send-super :init)
   (setq _A_X (float __A_X))
   (setq _A_Y (float __A_Y))
   (setq _B_X (float __B_X))
   (setq _B_Y (float __B_Y))
   (setq _a_x (float __a_x))
   (setq _a_y (float __a_y))
   (setq _b_x (float __b_x))
   (setq _b_y (float __b_y))
   (setq _Aa_Xx (float __Aa_Xx))
   (setq _Aa_Yy (float __Aa_Yy))
   (setq _Bb_Xx (float __Bb_Xx))
   (setq _Bb_Yy (float __Bb_Yy))
   self)
  (:A_X
   (&optional __A_X)
   (if __A_X (setq _A_X __A_X)) _A_X)
  (:A_Y
   (&optional __A_Y)
   (if __A_Y (setq _A_Y __A_Y)) _A_Y)
  (:B_X
   (&optional __B_X)
   (if __B_X (setq _B_X __B_X)) _B_X)
  (:B_Y
   (&optional __B_Y)
   (if __B_Y (setq _B_Y __B_Y)) _B_Y)
  (:a_x
   (&optional __a_x)
   (if __a_x (setq _a_x __a_x)) _a_x)
  (:a_y
   (&optional __a_y)
   (if __a_y (setq _a_y __a_y)) _a_y)
  (:b_x
   (&optional __b_x)
   (if __b_x (setq _b_x __b_x)) _b_x)
  (:b_y
   (&optional __b_y)
   (if __b_y (setq _b_y __b_y)) _b_y)
  (:Aa_Xx
   (&optional __Aa_Xx)
   (if __Aa_Xx (setq _Aa_Xx __Aa_Xx)) _Aa_Xx)
  (:Aa_Yy
   (&optional __Aa_Yy)
   (if __Aa_Yy (setq _Aa_Yy __Aa_Yy)) _Aa_Yy)
  (:Bb_Xx
   (&optional __Bb_Xx)
   (if __Bb_Xx (setq _Bb_Xx __Bb_Xx)) _Bb_Xx)
  (:Bb_Yy
   (&optional __Bb_Yy)
   (if __Bb_Yy (setq _Bb_Yy __Bb_Yy)) _Bb_Yy)
  (:serialization-length
   ()
   (+
    ;; float64 _A_X
    8
    ;; float64 _A_Y
    8
    ;; float64 _B_X
    8
    ;; float64 _B_Y
    8
    ;; float64 _a_x
    8
    ;; float64 _a_y
    8
    ;; float64 _b_x
    8
    ;; float64 _b_y
    8
    ;; float64 _Aa_Xx
    8
    ;; float64 _Aa_Yy
    8
    ;; float64 _Bb_Xx
    8
    ;; float64 _Bb_Yy
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _A_X
       (sys::poke _A_X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _A_Y
       (sys::poke _A_Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _B_X
       (sys::poke _B_X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _B_Y
       (sys::poke _B_Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_x
       (sys::poke _a_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_y
       (sys::poke _a_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b_x
       (sys::poke _b_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b_y
       (sys::poke _b_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Aa_Xx
       (sys::poke _Aa_Xx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Aa_Yy
       (sys::poke _Aa_Yy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Bb_Xx
       (sys::poke _Bb_Xx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Bb_Yy
       (sys::poke _Bb_Yy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _A_X
     (setq _A_X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _A_Y
     (setq _A_Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _B_X
     (setq _B_X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _B_Y
     (setq _B_Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_x
     (setq _a_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_y
     (setq _a_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b_x
     (setq _b_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b_y
     (setq _b_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Aa_Xx
     (setq _Aa_Xx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Aa_Yy
     (setq _Aa_Yy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Bb_Xx
     (setq _Bb_Xx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Bb_Yy
     (setq _Bb_Yy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rfans_driver::lidarpoint :md5sum-) "a84347ea08c5142003aeb6f79e914d56")
(setf (get rfans_driver::lidarpoint :datatype-) "rfans_driver/lidarpoint")
(setf (get rfans_driver::lidarpoint :definition-)
      "#大写表示左侧红锥桶，小写表示右侧蓝色锥桶;双写字母例如Aa表示黄色起始锥桶
float64 A_X
float64 A_Y
float64 B_X
float64 B_Y
float64 a_x
float64 a_y
float64 b_x
float64 b_y
float64 Aa_Xx
float64 Aa_Yy
float64 Bb_Xx
float64 Bb_Yy
")



(provide :rfans_driver/lidarpoint "a84347ea08c5142003aeb6f79e914d56")


