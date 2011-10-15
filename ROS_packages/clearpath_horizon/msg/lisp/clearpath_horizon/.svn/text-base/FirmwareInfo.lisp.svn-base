; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude FirmwareInfo.msg.html

(defclass <FirmwareInfo> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (firmware_major
    :reader firmware_major-val
    :initarg :firmware_major
    :type fixnum
    :initform 0)
   (firmware_minor
    :reader firmware_minor-val
    :initarg :firmware_minor
    :type fixnum
    :initform 0)
   (protocol_major
    :reader protocol_major-val
    :initarg :protocol_major
    :type fixnum
    :initform 0)
   (protocol_minor
    :reader protocol_minor-val
    :initarg :protocol_minor
    :type fixnum
    :initform 0)
   (firmware_write_time
    :reader firmware_write_time-val
    :initarg :firmware_write_time
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <FirmwareInfo>) ostream)
  "Serializes a message object of type '<FirmwareInfo>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'firmware_major)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'firmware_minor)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'protocol_major)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'protocol_minor)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'firmware_write_time)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'firmware_write_time)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'firmware_write_time)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'firmware_write_time)) ostream)
)
(defmethod deserialize ((msg <FirmwareInfo>) istream)
  "Deserializes a message object of type '<FirmwareInfo>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'firmware_major)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'firmware_minor)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'protocol_major)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'protocol_minor)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'firmware_write_time)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'firmware_write_time)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'firmware_write_time)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'firmware_write_time)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<FirmwareInfo>)))
  "Returns string type for a message object of type '<FirmwareInfo>"
  "clearpath_horizon/FirmwareInfo")
(defmethod md5sum ((type (eql '<FirmwareInfo>)))
  "Returns md5sum for a message object of type '<FirmwareInfo>"
  "dd399eb9c7b3816e8bea664a45a7e9ea")
(defmethod message-definition ((type (eql '<FirmwareInfo>)))
  "Returns full string definition for message of type '<FirmwareInfo>"
  (format nil "Header header~%int8 firmware_major~%int8 firmware_minor~%int8 protocol_major~%int8 protocol_minor~%uint32 firmware_write_time~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <FirmwareInfo>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     1
     1
     1
     4
))
(defmethod ros-message-to-list ((msg <FirmwareInfo>))
  "Converts a ROS message object to a list"
  (list '<FirmwareInfo>
    (cons ':header (header-val msg))
    (cons ':firmware_major (firmware_major-val msg))
    (cons ':firmware_minor (firmware_minor-val msg))
    (cons ':protocol_major (protocol_major-val msg))
    (cons ':protocol_minor (protocol_minor-val msg))
    (cons ':firmware_write_time (firmware_write_time-val msg))
))
