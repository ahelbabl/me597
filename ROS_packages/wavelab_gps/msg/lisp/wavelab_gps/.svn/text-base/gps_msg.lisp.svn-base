; Auto-generated. Do not edit!


(in-package wavelab_gps-msg)


;//! \htmlinclude gps_msg.msg.html

(defclass <gps_msg> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (status
    :reader status-val
    :initarg :status
    :type integer
    :initform 0)
   (lat
    :reader lat-val
    :initarg :lat
    :type float
    :initform 0.0)
   (lon
    :reader lon-val
    :initarg :lon
    :type float
    :initform 0.0)
   (lat_hemi
    :reader lat_hemi-val
    :initarg :lat_hemi
    :type integer
    :initform 0)
   (lon_hemi
    :reader lon_hemi-val
    :initarg :lon_hemi
    :type integer
    :initform 0)
   (speed
    :reader speed-val
    :initarg :speed
    :type float
    :initform 0.0)
   (heading
    :reader heading-val
    :initarg :heading
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <gps_msg>) ostream)
  "Serializes a message object of type '<gps_msg>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'status)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'lat))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'lon))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'lat_hemi)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'lon_hemi)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'speed))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'heading))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <gps_msg>) istream)
  "Deserializes a message object of type '<gps_msg>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'status)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'lat) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'lon) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'lat_hemi)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'lon_hemi)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<gps_msg>)))
  "Returns string type for a message object of type '<gps_msg>"
  "wavelab_gps/gps_msg")
(defmethod md5sum ((type (eql '<gps_msg>)))
  "Returns md5sum for a message object of type '<gps_msg>"
  "099306047b8eb0298fd4874f010e6694")
(defmethod message-definition ((type (eql '<gps_msg>)))
  "Returns full string definition for message of type '<gps_msg>"
  (format nil "Header header~%~%char status~%float32 lat~%float32 lon~%char lat_hemi~%char lon_hemi~%float32 speed~%float32 heading~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <gps_msg>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     4
     4
     1
     1
     4
     4
))
(defmethod ros-message-to-list ((msg <gps_msg>))
  "Converts a ROS message object to a list"
  (list '<gps_msg>
    (cons ':header (header-val msg))
    (cons ':status (status-val msg))
    (cons ':lat (lat-val msg))
    (cons ':lon (lon-val msg))
    (cons ':lat_hemi (lat_hemi-val msg))
    (cons ':lon_hemi (lon_hemi-val msg))
    (cons ':speed (speed-val msg))
    (cons ':heading (heading-val msg))
))
