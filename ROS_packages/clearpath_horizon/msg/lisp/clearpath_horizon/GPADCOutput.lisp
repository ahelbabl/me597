; Auto-generated. Do not edit!


(in-package clearpath_horizon-msg)


;//! \htmlinclude GPADCOutput.msg.html

(defclass <GPADCOutput> (ros-message)
  ()
)
(defmethod serialize ((msg <GPADCOutput>) ostream)
  "Serializes a message object of type '<GPADCOutput>"
)
(defmethod deserialize ((msg <GPADCOutput>) istream)
  "Deserializes a message object of type '<GPADCOutput>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GPADCOutput>)))
  "Returns string type for a message object of type '<GPADCOutput>"
  "clearpath_horizon/GPADCOutput")
(defmethod md5sum ((type (eql '<GPADCOutput>)))
  "Returns md5sum for a message object of type '<GPADCOutput>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<GPADCOutput>)))
  "Returns full string definition for message of type '<GPADCOutput>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <GPADCOutput>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GPADCOutput>))
  "Converts a ROS message object to a list"
  (list '<GPADCOutput>
))
