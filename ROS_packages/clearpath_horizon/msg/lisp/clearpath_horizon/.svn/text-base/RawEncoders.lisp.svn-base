; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude RawEncoders.msg.html

(defclass <RawEncoders> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (ticks
    :reader ticks-val
    :initarg :ticks
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0)))
)
(defmethod serialize ((msg <RawEncoders>) ostream)
  "Serializes a message object of type '<RawEncoders>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'ticks))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'ticks))
)
(defmethod deserialize ((msg <RawEncoders>) istream)
  "Deserializes a message object of type '<RawEncoders>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'ticks) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'ticks)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<RawEncoders>)))
  "Returns string type for a message object of type '<RawEncoders>"
  "clearpath_horizon/RawEncoders")
(defmethod md5sum ((type (eql '<RawEncoders>)))
  "Returns md5sum for a message object of type '<RawEncoders>"
  "d105c5e4b407d4a68215fce86175fe75")
(defmethod message-definition ((type (eql '<RawEncoders>)))
  "Returns full string definition for message of type '<RawEncoders>"
  (format nil "Header header~%int32[] ticks~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <RawEncoders>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (reduce #'+ (slot-value msg 'ticks) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <RawEncoders>))
  "Converts a ROS message object to a list"
  (list '<RawEncoders>
    (cons ':header (header-val msg))
    (cons ':ticks (ticks-val msg))
))
