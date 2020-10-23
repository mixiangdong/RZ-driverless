; Auto-generated. Do not edit!


(cl:in-package rfans_driver-srv)


;//! \htmlinclude null-request.msg.html

(cl:defclass <null-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass null-request (<null-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <null-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'null-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rfans_driver-srv:<null-request> is deprecated: use rfans_driver-srv:null-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <null-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rfans_driver-srv:state-val is deprecated.  Use rfans_driver-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <null-request>) ostream)
  "Serializes a message object of type '<null-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <null-request>) istream)
  "Deserializes a message object of type '<null-request>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<null-request>)))
  "Returns string type for a service object of type '<null-request>"
  "rfans_driver/nullRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'null-request)))
  "Returns string type for a service object of type 'null-request"
  "rfans_driver/nullRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<null-request>)))
  "Returns md5sum for a message object of type '<null-request>"
  "0ec340068269569bdcfcbf79cb9781d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'null-request)))
  "Returns md5sum for a message object of type 'null-request"
  "0ec340068269569bdcfcbf79cb9781d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<null-request>)))
  "Returns full string definition for message of type '<null-request>"
  (cl:format cl:nil "~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'null-request)))
  "Returns full string definition for message of type 'null-request"
  (cl:format cl:nil "~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <null-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <null-request>))
  "Converts a ROS message object to a list"
  (cl:list 'null-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude null-response.msg.html

(cl:defclass <null-response> (roslisp-msg-protocol:ros-message)
  ((work
    :reader work
    :initarg :work
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass null-response (<null-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <null-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'null-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rfans_driver-srv:<null-response> is deprecated: use rfans_driver-srv:null-response instead.")))

(cl:ensure-generic-function 'work-val :lambda-list '(m))
(cl:defmethod work-val ((m <null-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rfans_driver-srv:work-val is deprecated.  Use rfans_driver-srv:work instead.")
  (work m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <null-response>) ostream)
  "Serializes a message object of type '<null-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'work) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <null-response>) istream)
  "Deserializes a message object of type '<null-response>"
    (cl:setf (cl:slot-value msg 'work) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<null-response>)))
  "Returns string type for a service object of type '<null-response>"
  "rfans_driver/nullResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'null-response)))
  "Returns string type for a service object of type 'null-response"
  "rfans_driver/nullResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<null-response>)))
  "Returns md5sum for a message object of type '<null-response>"
  "0ec340068269569bdcfcbf79cb9781d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'null-response)))
  "Returns md5sum for a message object of type 'null-response"
  "0ec340068269569bdcfcbf79cb9781d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<null-response>)))
  "Returns full string definition for message of type '<null-response>"
  (cl:format cl:nil "bool work~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'null-response)))
  "Returns full string definition for message of type 'null-response"
  (cl:format cl:nil "bool work~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <null-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <null-response>))
  "Converts a ROS message object to a list"
  (cl:list 'null-response
    (cl:cons ':work (work msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'null)))
  'null-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'null)))
  'null-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'null)))
  "Returns string type for a service object of type '<null>"
  "rfans_driver/null")