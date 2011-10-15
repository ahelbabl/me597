; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude Joy.msg.html

(defclass <Joy> (ros-message)
  ((axes
    :reader axes-val
    :initarg :axes
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (buttons
    :reader buttons-val
    :initarg :buttons
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0)))
)
(defmethod serialize ((msg <Joy>) ostream)
  "Serializes a message object of type '<Joy>"
  (let ((__ros_arr_len (length (slot-value msg 'axes))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'axes))
  (let ((__ros_arr_len (length (slot-value msg 'buttons))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'buttons))
)
(defmethod deserialize ((msg <Joy>) istream)
  "Deserializes a message object of type '<Joy>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'axes) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'axes)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'buttons) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'buttons)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Joy>)))
  "Returns string type for a message object of type '<Joy>"
  "clearpath_horizon/Joy")
(defmethod md5sum ((type (eql '<Joy>)))
  "Returns md5sum for a message object of type '<Joy>"
  "e3ef016fcdf22397038b36036c66f7c8")
(defmethod message-definition ((type (eql '<Joy>)))
  "Returns full string definition for message of type '<Joy>"
  (format nil "float32[] axes~%int32[] buttons~%~%~%"))
(defmethod serialization-length ((msg <Joy>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'axes) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'buttons) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <Joy>))
  "Converts a ROS message object to a list"
  (list '<Joy>
    (cons ':axes (axes-val msg))
    (cons ':buttons (buttons-val msg))
))
