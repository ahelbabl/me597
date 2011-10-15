; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude PowerStatus.msg.html

(defclass <PowerStatus> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (sources
    :reader sources-val
    :initarg :sources
    :type (vector <PowerSource>)
   :initform (make-array 0 :element-type '<PowerSource> :initial-element (make-instance '<PowerSource>))))
)
(defmethod serialize ((msg <PowerStatus>) ostream)
  "Serializes a message object of type '<PowerStatus>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'sources))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'sources))
)
(defmethod deserialize ((msg <PowerStatus>) istream)
  "Deserializes a message object of type '<PowerStatus>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'sources) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'sources)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance '<PowerSource>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<PowerStatus>)))
  "Returns string type for a message object of type '<PowerStatus>"
  "clearpath_horizon/PowerStatus")
(defmethod md5sum ((type (eql '<PowerStatus>)))
  "Returns md5sum for a message object of type '<PowerStatus>"
  "f246c359530c58415aee4fe89d1aca04")
(defmethod message-definition ((type (eql '<PowerStatus>)))
  "Returns full string definition for message of type '<PowerStatus>"
  (format nil "Header header~%PowerSource[] sources~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: clearpath_horizon/PowerSource~%float32 charge~%int16 capacity~%bool present~%bool in_use~%uint8 description~%~%~%"))
(defmethod serialization-length ((msg <PowerStatus>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (reduce #'+ (slot-value msg 'sources) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <PowerStatus>))
  "Converts a ROS message object to a list"
  (list '<PowerStatus>
    (cons ':header (header-val msg))
    (cons ':sources (sources-val msg))
))
