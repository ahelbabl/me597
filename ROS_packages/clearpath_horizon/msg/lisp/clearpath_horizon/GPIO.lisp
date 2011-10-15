; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude GPIO.msg.html

(defclass <GPIO> (ros-message)
  ()
)
(defmethod serialize ((msg <GPIO>) ostream)
  "Serializes a message object of type '<GPIO>"
)
(defmethod deserialize ((msg <GPIO>) istream)
  "Deserializes a message object of type '<GPIO>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GPIO>)))
  "Returns string type for a message object of type '<GPIO>"
  "clearpath_horizon/GPIO")
(defmethod md5sum ((type (eql '<GPIO>)))
  "Returns md5sum for a message object of type '<GPIO>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<GPIO>)))
  "Returns full string definition for message of type '<GPIO>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <GPIO>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GPIO>))
  "Converts a ROS message object to a list"
  (list '<GPIO>
))
