; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude SafetyStatus.msg.html

(defclass <SafetyStatus> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (flags
    :reader flags-val
    :initarg :flags
    :type fixnum
    :initform 0)
   (estop
    :reader estop-val
    :initarg :estop
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <SafetyStatus>) ostream)
  "Serializes a message object of type '<SafetyStatus>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'flags)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'flags)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'estop) 1 0)) ostream)
)
(defmethod deserialize ((msg <SafetyStatus>) istream)
  "Deserializes a message object of type '<SafetyStatus>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'flags)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'flags)) (read-byte istream))
  (setf (slot-value msg 'estop) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<SafetyStatus>)))
  "Returns string type for a message object of type '<SafetyStatus>"
  "clearpath_horizon/SafetyStatus")
(defmethod md5sum ((type (eql '<SafetyStatus>)))
  "Returns md5sum for a message object of type '<SafetyStatus>"
  "cf78d6042b92d64ebda55641e06d66fa")
(defmethod message-definition ((type (eql '<SafetyStatus>)))
  "Returns full string definition for message of type '<SafetyStatus>"
  (format nil "Header header~%uint16 flags~%bool estop~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <SafetyStatus>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     2
     1
))
(defmethod ros-message-to-list ((msg <SafetyStatus>))
  "Converts a ROS message object to a list"
  (list '<SafetyStatus>
    (cons ':header (header-val msg))
    (cons ':flags (flags-val msg))
    (cons ':estop (estop-val msg))
))
