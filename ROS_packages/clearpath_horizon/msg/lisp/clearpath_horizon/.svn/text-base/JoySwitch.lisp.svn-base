; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude JoySwitch.msg.html

(defclass <JoySwitch> (ros-message)
  ((robot_id
    :reader robot_id-val
    :initarg :robot_id
    :type string
    :initform "")
   (attach
    :reader attach-val
    :initarg :attach
    :type fixnum
    :initform 0)
   (joystick
    :reader joystick-val
    :initarg :joystick
    :type string
    :initform ""))
)
(defmethod serialize ((msg <JoySwitch>) ostream)
  "Serializes a message object of type '<JoySwitch>"
  (let ((__ros_str_len (length (slot-value msg 'robot_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'robot_id))
    (write-byte (ldb (byte 8 0) (slot-value msg 'attach)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'joystick))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'joystick))
)
(defmethod deserialize ((msg <JoySwitch>) istream)
  "Deserializes a message object of type '<JoySwitch>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'robot_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'robot_id) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'attach)) (read-byte istream))
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
(defmethod ros-datatype ((msg (eql '<JoySwitch>)))
  "Returns string type for a message object of type '<JoySwitch>"
  "clearpath_horizon/JoySwitch")
(defmethod md5sum ((type (eql '<JoySwitch>)))
  "Returns md5sum for a message object of type '<JoySwitch>"
  "0b9e4d12a122fa671dc7b4bd8741705d")
(defmethod message-definition ((type (eql '<JoySwitch>)))
  "Returns full string definition for message of type '<JoySwitch>"
  (format nil "string robot_id~%uint8 attach~%string joystick~%~%~%"))
(defmethod serialization-length ((msg <JoySwitch>))
  (+ 0
     4 (length (slot-value msg 'robot_id))
     1
     4 (length (slot-value msg 'joystick))
))
(defmethod ros-message-to-list ((msg <JoySwitch>))
  "Converts a ROS message object to a list"
  (list '<JoySwitch>
    (cons ':robot_id (robot_id-val msg))
    (cons ':attach (attach-val msg))
    (cons ':joystick (joystick-val msg))
))
