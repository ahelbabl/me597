; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude Announce.msg.html

(defclass <Announce> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (action
    :reader action-val
    :initarg :action
    :type string
    :initform "")
   (topic
    :reader topic-val
    :initarg :topic
    :type string
    :initform ""))
)
(defmethod serialize ((msg <Announce>) ostream)
  "Serializes a message object of type '<Announce>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'action))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'action))
  (let ((__ros_str_len (length (slot-value msg 'topic))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'topic))
)
(defmethod deserialize ((msg <Announce>) istream)
  "Deserializes a message object of type '<Announce>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'action) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'action) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'topic) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'topic) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Announce>)))
  "Returns string type for a message object of type '<Announce>"
  "clearpath_horizon/Announce")
(defmethod md5sum ((type (eql '<Announce>)))
  "Returns md5sum for a message object of type '<Announce>"
  "6767c1dfe3757cc07658c1e09e10b1ba")
(defmethod message-definition ((type (eql '<Announce>)))
  "Returns full string definition for message of type '<Announce>"
  (format nil "Header header~%string action~%string topic~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Announce>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'action))
     4 (length (slot-value msg 'topic))
))
(defmethod ros-message-to-list ((msg <Announce>))
  "Converts a ROS message object to a list"
  (list '<Announce>
    (cons ':header (header-val msg))
    (cons ':action (action-val msg))
    (cons ':topic (topic-val msg))
))
