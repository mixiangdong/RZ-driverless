;; Auto-generated. Do not edit!


(when (boundp 'rfans_driver::null)
  (if (not (find-package "RFANS_DRIVER"))
    (make-package "RFANS_DRIVER"))
  (shadow 'null (find-package "RFANS_DRIVER")))
(unless (find-package "RFANS_DRIVER::NULL")
  (make-package "RFANS_DRIVER::NULL"))
(unless (find-package "RFANS_DRIVER::NULLREQUEST")
  (make-package "RFANS_DRIVER::NULLREQUEST"))
(unless (find-package "RFANS_DRIVER::NULLRESPONSE")
  (make-package "RFANS_DRIVER::NULLRESPONSE"))

(in-package "ROS")





(defclass rfans_driver::nullRequest
  :super ros::object
  :slots (_state ))

(defmethod rfans_driver::nullRequest
  (:init
   (&key
    ((:state __state) nil)
    )
   (send-super :init)
   (setq _state __state)
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; bool _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rfans_driver::nullResponse
  :super ros::object
  :slots (_work ))

(defmethod rfans_driver::nullResponse
  (:init
   (&key
    ((:work __work) nil)
    )
   (send-super :init)
   (setq _work __work)
   self)
  (:work
   (&optional __work)
   (if __work (setq _work __work)) _work)
  (:serialization-length
   ()
   (+
    ;; bool _work
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _work
       (if _work (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _work
     (setq _work (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rfans_driver::null
  :super ros::object
  :slots ())

(setf (get rfans_driver::null :md5sum-) "0ec340068269569bdcfcbf79cb9781d5")
(setf (get rfans_driver::null :datatype-) "rfans_driver/null")
(setf (get rfans_driver::null :request) rfans_driver::nullRequest)
(setf (get rfans_driver::null :response) rfans_driver::nullResponse)

(defmethod rfans_driver::nullRequest
  (:response () (instance rfans_driver::nullResponse :init)))

(setf (get rfans_driver::nullRequest :md5sum-) "0ec340068269569bdcfcbf79cb9781d5")
(setf (get rfans_driver::nullRequest :datatype-) "rfans_driver/nullRequest")
(setf (get rfans_driver::nullRequest :definition-)
      "
bool state
---
bool work
")

(setf (get rfans_driver::nullResponse :md5sum-) "0ec340068269569bdcfcbf79cb9781d5")
(setf (get rfans_driver::nullResponse :datatype-) "rfans_driver/nullResponse")
(setf (get rfans_driver::nullResponse :definition-)
      "
bool state
---
bool work
")



(provide :rfans_driver/null "0ec340068269569bdcfcbf79cb9781d5")


