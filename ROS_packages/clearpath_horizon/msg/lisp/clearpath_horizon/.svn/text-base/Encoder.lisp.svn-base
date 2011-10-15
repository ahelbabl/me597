; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude Encoder.msg.html

(defclass <Encoder> (ros-message)
  ((travel
    :reader travel-val
    :initarg :travel
    :type float
    :initform 0.0)
   (speed
    :reader speed-val
    :initarg :speed
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Encoder>) ostream)
  "Serializes a message object of type '<Encoder>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'travel))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'speed))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <Encoder>) istream)
  "Deserializes a message object of type '<Encoder>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'travel) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Encoder>)))
  "Returns string type for a message object of type '<Encoder>"
  "clearpath_horizon/Encoder")
(defmethod md5sum ((type (eql '<Encoder>)))
  "Returns md5sum for a message object of type '<Encoder>"
  "620a74fe7e8deb0e96bf85b534453633")
(defmethod message-definition ((type (eql '<Encoder>)))
  "Returns full string definition for message of type '<Encoder>"
  (format nil "float64 travel~%float64 speed~%~%~%"))
(defmethod serialization-length ((msg <Encoder>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <Encoder>))
  "Converts a ROS message object to a list"
  (list '<Encoder>
    (cons ':travel (travel-val msg))
    (cons ':speed (speed-val msg))
))
