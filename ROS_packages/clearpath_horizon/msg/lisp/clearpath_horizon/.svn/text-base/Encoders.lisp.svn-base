; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude Encoders.msg.html

(defclass <Encoders> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (encoders
    :reader encoders-val
    :initarg :encoders
    :type (vector <Encoder>)
   :initform (make-array 0 :element-type '<Encoder> :initial-element (make-instance '<Encoder>))))
)
(defmethod serialize ((msg <Encoders>) ostream)
  "Serializes a message object of type '<Encoders>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'encoders))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'encoders))
)
(defmethod deserialize ((msg <Encoders>) istream)
  "Deserializes a message object of type '<Encoders>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'encoders) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'encoders)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance '<Encoder>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Encoders>)))
  "Returns string type for a message object of type '<Encoders>"
  "clearpath_horizon/Encoders")
(defmethod md5sum ((type (eql '<Encoders>)))
  "Returns md5sum for a message object of type '<Encoders>"
  "2ea748832c2014369ffabd316d5aad8c")
(defmethod message-definition ((type (eql '<Encoders>)))
  "Returns full string definition for message of type '<Encoders>"
  (format nil "Header header~%Encoder[] encoders~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: clearpath_horizon/Encoder~%float64 travel~%float64 speed~%~%~%"))
(defmethod serialization-length ((msg <Encoders>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (reduce #'+ (slot-value msg 'encoders) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <Encoders>))
  "Converts a ROS message object to a list"
  (list '<Encoders>
    (cons ':header (header-val msg))
    (cons ':encoders (encoders-val msg))
))
