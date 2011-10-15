; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude PlatformInfo.msg.html

(defclass <PlatformInfo> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (model
    :reader model-val
    :initarg :model
    :type string
    :initform "")
   (revision
    :reader revision-val
    :initarg :revision
    :type fixnum
    :initform 0)
   (serial
    :reader serial-val
    :initarg :serial
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <PlatformInfo>) ostream)
  "Serializes a message object of type '<PlatformInfo>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'model))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'model))
    (write-byte (ldb (byte 8 0) (slot-value msg 'revision)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'serial)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'serial)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'serial)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'serial)) ostream)
)
(defmethod deserialize ((msg <PlatformInfo>) istream)
  "Deserializes a message object of type '<PlatformInfo>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'model) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'model) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'revision)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'serial)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'serial)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'serial)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'serial)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PlatformInfo>)))
  "Returns string type for a message object of type '<PlatformInfo>"
  "clearpath_horizon/PlatformInfo")
(defmethod md5sum ((type (eql '<PlatformInfo>)))
  "Returns md5sum for a message object of type '<PlatformInfo>"
  "ff95c25c6ef78f06bbb7ef85aad5735e")
(defmethod message-definition ((type (eql '<PlatformInfo>)))
  "Returns full string definition for message of type '<PlatformInfo>"
  (format nil "Header header~%string model~%int8 revision~%uint32 serial~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <PlatformInfo>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'model))
     1
     4
))
(defmethod ros-message-to-list ((msg <PlatformInfo>))
  "Converts a ROS message object to a list"
  (list '<PlatformInfo>
    (cons ':header (header-val msg))
    (cons ':model (model-val msg))
    (cons ':revision (revision-val msg))
    (cons ':serial (serial-val msg))
))
