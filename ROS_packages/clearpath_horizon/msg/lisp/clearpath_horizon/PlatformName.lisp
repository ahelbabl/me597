; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude PlatformName.msg.html

(defclass <PlatformName> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (name
    :reader name-val
    :initarg :name
    :type string
    :initform ""))
)
(defmethod serialize ((msg <PlatformName>) ostream)
  "Serializes a message object of type '<PlatformName>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'name))
)
(defmethod deserialize ((msg <PlatformName>) istream)
  "Deserializes a message object of type '<PlatformName>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'name) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<PlatformName>)))
  "Returns string type for a message object of type '<PlatformName>"
  "clearpath_horizon/PlatformName")
(defmethod md5sum ((type (eql '<PlatformName>)))
  "Returns md5sum for a message object of type '<PlatformName>"
  "e541bb76a27a3e03e7987c32ec4fd724")
(defmethod message-definition ((type (eql '<PlatformName>)))
  "Returns full string definition for message of type '<PlatformName>"
  (format nil "Header header~%string name~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <PlatformName>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'name))
))
(defmethod ros-message-to-list ((msg <PlatformName>))
  "Converts a ROS message object to a list"
  (list '<PlatformName>
    (cons ':header (header-val msg))
    (cons ':name (name-val msg))
))
