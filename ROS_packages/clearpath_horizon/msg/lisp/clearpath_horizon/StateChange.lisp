; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude StateChange.msg.html

(defclass <StateChange> (ros-message)
  ((new_state
    :reader new_state-val
    :initarg :new_state
    :type string
    :initform "")
   (joystick
    :reader joystick-val
    :initarg :joystick
    :type string
    :initform ""))
)
(defmethod serialize ((msg <StateChange>) ostream)
  "Serializes a message object of type '<StateChange>"
  (let ((__ros_str_len (length (slot-value msg 'new_state))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'new_state))
  (let ((__ros_str_len (length (slot-value msg 'joystick))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'joystick))
)
(defmethod deserialize ((msg <StateChange>) istream)
  "Deserializes a message object of type '<StateChange>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'new_state) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'new_state) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'joystick) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'joystick) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<StateChange>)))
  "Returns string type for a message object of type '<StateChange>"
  "clearpath_horizon/StateChange")
(defmethod md5sum ((type (eql '<StateChange>)))
  "Returns md5sum for a message object of type '<StateChange>"
  "44a4273c39fe35090d35b71e32a477da")
(defmethod message-definition ((type (eql '<StateChange>)))
  "Returns full string definition for message of type '<StateChange>"
  (format nil "string new_state~%string joystick~%~%~%"))
(defmethod serialization-length ((msg <StateChange>))
  (+ 0
     4 (length (slot-value msg 'new_state))
     4 (length (slot-value msg 'joystick))
))
(defmethod ros-message-to-list ((msg <StateChange>))
  "Converts a ROS message object to a list"
  (list '<StateChange>
    (cons ':new_state (new_state-val msg))
    (cons ':joystick (joystick-val msg))
))
