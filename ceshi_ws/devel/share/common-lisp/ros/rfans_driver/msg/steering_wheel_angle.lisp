; Auto-generated. Do not edit!


(cl:in-package rfans_driver-msg)


;//! \htmlinclude steering_wheel_angle.msg.html

(cl:defclass <steering_wheel_angle> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass steering_wheel_angle (<steering_wheel_angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <steering_wheel_angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'steering_wheel_angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rfans_driver-msg:<steering_wheel_angle> is deprecated: use rfans_driver-msg:steering_wheel_angle instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <steering_wheel_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rfans_driver-msg:data-val is deprecated.  Use rfans_driver-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <steering_wheel_angle>) ostream)
  "Serializes a message object of type '<steering_wheel_angle>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <steering_wheel_angle>) istream)
  "Deserializes a message object of type '<steering_wheel_angle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<steering_wheel_angle>)))
  "Returns string type for a message object of type '<steering_wheel_angle>"
  "rfans_driver/steering_wheel_angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'steering_wheel_angle)))
  "Returns string type for a message object of type 'steering_wheel_angle"
  "rfans_driver/steering_wheel_angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<steering_wheel_angle>)))
  "Returns md5sum for a message object of type '<steering_wheel_angle>"
  "fdb28210bfa9d7c91146260178d9a584")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'steering_wheel_angle)))
  "Returns md5sum for a message object of type 'steering_wheel_angle"
  "fdb28210bfa9d7c91146260178d9a584")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<steering_wheel_angle>)))
  "Returns full string definition for message of type '<steering_wheel_angle>"
  (cl:format cl:nil "float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'steering_wheel_angle)))
  "Returns full string definition for message of type 'steering_wheel_angle"
  (cl:format cl:nil "float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <steering_wheel_angle>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <steering_wheel_angle>))
  "Converts a ROS message object to a list"
  (cl:list 'steering_wheel_angle
    (cl:cons ':data (data msg))
))
