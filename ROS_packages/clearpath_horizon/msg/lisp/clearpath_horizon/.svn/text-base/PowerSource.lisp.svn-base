; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude PowerSource.msg.html

(defclass <PowerSource> (ros-message)
  ((charge
    :reader charge-val
    :initarg :charge
    :type float
    :initform 0.0)
   (capacity
    :reader capacity-val
    :initarg :capacity
    :type fixnum
    :initform 0)
   (present
    :reader present-val
    :initarg :present
    :type boolean
    :initform nil)
   (in_use
    :reader in_use-val
    :initarg :in_use
    :type boolean
    :initform nil)
   (description
    :reader description-val
    :initarg :description
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <PowerSource>) ostream)
  "Serializes a message object of type '<PowerSource>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'charge))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'capacity)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'capacity)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'present) 1 0)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'in_use) 1 0)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'description)) ostream)
)
(defmethod deserialize ((msg <PowerSource>) istream)
  "Deserializes a message object of type '<PowerSource>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'charge) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'capacity)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'capacity)) (read-byte istream))
  (setf (slot-value msg 'present) (not (zerop (read-byte istream))))
  (setf (slot-value msg 'in_use) (not (zerop (read-byte istream))))
  (setf (ldb (byte 8 0) (slot-value msg 'description)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PowerSource>)))
  "Returns string type for a message object of type '<PowerSource>"
  "clearpath_horizon/PowerSource")
(defmethod md5sum ((type (eql '<PowerSource>)))
  "Returns md5sum for a message object of type '<PowerSource>"
  "adbe384d7d69a337a7f2b6bf1d0139cb")
(defmethod message-definition ((type (eql '<PowerSource>)))
  "Returns full string definition for message of type '<PowerSource>"
  (format nil "float32 charge~%int16 capacity~%bool present~%bool in_use~%uint8 description~%~%~%"))
(defmethod serialization-length ((msg <PowerSource>))
  (+ 0
     4
     2
     1
     1
     1
))
(defmethod ros-message-to-list ((msg <PowerSource>))
  "Converts a ROS message object to a list"
  (list '<PowerSource>
    (cons ':charge (charge-val msg))
    (cons ':capacity (capacity-val msg))
    (cons ':present (present-val msg))
    (cons ':in_use (in_use-val msg))
    (cons ':description (description-val msg))
))
