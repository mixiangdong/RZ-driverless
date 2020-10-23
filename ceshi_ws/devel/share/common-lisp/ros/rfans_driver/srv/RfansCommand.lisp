; Auto-generated. Do not edit!


(cl:in-package rfans_driver-srv)


;//! \htmlinclude RfansCommand-request.msg.html

(cl:defclass <RfansCommand-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass RfansCommand-request (<RfansCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RfansCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RfansCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rfans_driver-srv:<RfansCommand-request> is deprecated: use rfans_driver-srv:RfansCommand-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <RfansCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rfans_driver-srv:cmd-val is deprecated.  Use rfans_driver-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <RfansCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rfans_driver-srv:speed-val is deprecated.  Use rfans_driver-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RfansCommand-request>) ostream)
  "Serializes a message object of type '<RfansCommand-request>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RfansCommand-request>) istream)
  "Deserializes a message object of type '<RfansCommand-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RfansCommand-request>)))
  "Returns string type for a service object of type '<RfansCommand-request>"
  "rfans_driver/RfansCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RfansCommand-request)))
  "Returns string type for a service object of type 'RfansCommand-request"
  "rfans_driver/RfansCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RfansCommand-request>)))
  "Returns md5sum for a message object of type '<RfansCommand-request>"
  "2c14a5137ad0eede7d4a988cde7612e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RfansCommand-request)))
  "Returns md5sum for a message object of type 'RfansCommand-request"
  "2c14a5137ad0eede7d4a988cde7612e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RfansCommand-request>)))
  "Returns full string definition for message of type '<RfansCommand-request>"
  (cl:format cl:nil "int32 cmd~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RfansCommand-request)))
  "Returns full string definition for message of type 'RfansCommand-request"
  (cl:format cl:nil "int32 cmd~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RfansCommand-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RfansCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RfansCommand-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude RfansCommand-response.msg.html

(cl:defclass <RfansCommand-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass RfansCommand-response (<RfansCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RfansCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RfansCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rfans_driver-srv:<RfansCommand-response> is deprecated: use rfans_driver-srv:RfansCommand-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <RfansCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rfans_driver-srv:status-val is deprecated.  Use rfans_driver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RfansCommand-response>) ostream)
  "Serializes a message object of type '<RfansCommand-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RfansCommand-response>) istream)
  "Deserializes a message object of type '<RfansCommand-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RfansCommand-response>)))
  "Returns string type for a service object of type '<RfansCommand-response>"
  "rfans_driver/RfansCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RfansCommand-response)))
  "Returns string type for a service object of type 'RfansCommand-response"
  "rfans_driver/RfansCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RfansCommand-response>)))
  "Returns md5sum for a message object of type '<RfansCommand-response>"
  "2c14a5137ad0eede7d4a988cde7612e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RfansCommand-response)))
  "Returns md5sum for a message object of type 'RfansCommand-response"
  "2c14a5137ad0eede7d4a988cde7612e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RfansCommand-response>)))
  "Returns full string definition for message of type '<RfansCommand-response>"
  (cl:format cl:nil "int32 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RfansCommand-response)))
  "Returns full string definition for message of type 'RfansCommand-response"
  (cl:format cl:nil "int32 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RfansCommand-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RfansCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RfansCommand-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RfansCommand)))
  'RfansCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RfansCommand)))
  'RfansCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RfansCommand)))
  "Returns string type for a service object of type '<RfansCommand>"
  "rfans_driver/RfansCommand")