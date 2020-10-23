;; Auto-generated. Do not edit!


(when (boundp 'rfans_driver::RfansCommand)
  (if (not (find-package "RFANS_DRIVER"))
    (make-package "RFANS_DRIVER"))
  (shadow 'RfansCommand (find-package "RFANS_DRIVER")))
(unless (find-package "RFANS_DRIVER::RFANSCOMMAND")
  (make-package "RFANS_DRIVER::RFANSCOMMAND"))
(unless (find-package "RFANS_DRIVER::RFANSCOMMANDREQUEST")
  (make-package "RFANS_DRIVER::RFANSCOMMANDREQUEST"))
(unless (find-package "RFANS_DRIVER::RFANSCOMMANDRESPONSE")
  (make-package "RFANS_DRIVER::RFANSCOMMANDRESPONSE"))

(in-package "ROS")





(defclass rfans_driver::RfansCommandRequest
  :super ros::object
  :slots (_cmd _speed ))

(defmethod rfans_driver::RfansCommandRequest
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _speed (round __speed))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _cmd
    4
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _cmd
       (write-long _cmd s)
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _cmd
     (setq _cmd (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rfans_driver::RfansCommandResponse
  :super ros::object
  :slots (_status ))

(defmethod rfans_driver::RfansCommandResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int32 _status
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _status
       (write-long _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rfans_driver::RfansCommand
  :super ros::object
  :slots ())

(setf (get rfans_driver::RfansCommand :md5sum-) "2c14a5137ad0eede7d4a988cde7612e0")
(setf (get rfans_driver::RfansCommand :datatype-) "rfans_driver/RfansCommand")
(setf (get rfans_driver::RfansCommand :request) rfans_driver::RfansCommandRequest)
(setf (get rfans_driver::RfansCommand :response) rfans_driver::RfansCommandResponse)

(defmethod rfans_driver::RfansCommandRequest
  (:response () (instance rfans_driver::RfansCommandResponse :init)))

(setf (get rfans_driver::RfansCommandRequest :md5sum-) "2c14a5137ad0eede7d4a988cde7612e0")
(setf (get rfans_driver::RfansCommandRequest :datatype-) "rfans_driver/RfansCommandRequest")
(setf (get rfans_driver::RfansCommandRequest :definition-)
      "int32 cmd
int32 speed
---
int32 status

")

(setf (get rfans_driver::RfansCommandResponse :md5sum-) "2c14a5137ad0eede7d4a988cde7612e0")
(setf (get rfans_driver::RfansCommandResponse :datatype-) "rfans_driver/RfansCommandResponse")
(setf (get rfans_driver::RfansCommandResponse :definition-)
      "int32 cmd
int32 speed
---
int32 status

")



(provide :rfans_driver/RfansCommand "2c14a5137ad0eede7d4a988cde7612e0")


