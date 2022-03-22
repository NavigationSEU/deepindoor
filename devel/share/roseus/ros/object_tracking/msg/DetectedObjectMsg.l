;; Auto-generated. Do not edit!


(when (boundp 'object_tracking::DetectedObjectMsg)
  (if (not (find-package "OBJECT_TRACKING"))
    (make-package "OBJECT_TRACKING"))
  (shadow 'DetectedObjectMsg (find-package "OBJECT_TRACKING")))
(unless (find-package "OBJECT_TRACKING::DETECTEDOBJECTMSG")
  (make-package "OBJECT_TRACKING::DETECTEDOBJECTMSG"))

(in-package "ROS")
;;//! \htmlinclude DetectedObjectMsg.msg.html
(if (not (find-package "JSK_RECOGNITION_MSGS"))
  (ros::roseus-add-msgs "jsk_recognition_msgs"))
(if (not (find-package "VISUALIZATION_MSGS"))
  (ros::roseus-add-msgs "visualization_msgs"))


(defclass object_tracking::DetectedObjectMsg
  :super ros::object
  :slots (_markers _boxes ))

(defmethod object_tracking::DetectedObjectMsg
  (:init
   (&key
    ((:markers __markers) (instance visualization_msgs::MarkerArray :init))
    ((:boxes __boxes) (instance jsk_recognition_msgs::BoundingBoxArray :init))
    )
   (send-super :init)
   (setq _markers __markers)
   (setq _boxes __boxes)
   self)
  (:markers
   (&rest __markers)
   (if (keywordp (car __markers))
       (send* _markers __markers)
     (progn
       (if __markers (setq _markers (car __markers)))
       _markers)))
  (:boxes
   (&rest __boxes)
   (if (keywordp (car __boxes))
       (send* _boxes __boxes)
     (progn
       (if __boxes (setq _boxes (car __boxes)))
       _boxes)))
  (:serialization-length
   ()
   (+
    ;; visualization_msgs/MarkerArray _markers
    (send _markers :serialization-length)
    ;; jsk_recognition_msgs/BoundingBoxArray _boxes
    (send _boxes :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; visualization_msgs/MarkerArray _markers
       (send _markers :serialize s)
     ;; jsk_recognition_msgs/BoundingBoxArray _boxes
       (send _boxes :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; visualization_msgs/MarkerArray _markers
     (send _markers :deserialize buf ptr-) (incf ptr- (send _markers :serialization-length))
   ;; jsk_recognition_msgs/BoundingBoxArray _boxes
     (send _boxes :deserialize buf ptr-) (incf ptr- (send _boxes :serialization-length))
   ;;
   self)
  )

(setf (get object_tracking::DetectedObjectMsg :md5sum-) "a0ee17136c14b2823c04c42625441d79")
(setf (get object_tracking::DetectedObjectMsg :datatype-) "object_tracking/DetectedObjectMsg")
(setf (get object_tracking::DetectedObjectMsg :definition-)
      "visualization_msgs/MarkerArray markers
jsk_recognition_msgs/BoundingBoxArray boxes
================================================================================
MSG: visualization_msgs/MarkerArray
Marker[] markers

================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

================================================================================
MSG: jsk_recognition_msgs/BoundingBoxArray
# BoundingBoxArray is a list of BoundingBox.
# You can use jsk_rviz_plugins to visualize BoungingBoxArray on rviz.
Header header
BoundingBox[] boxes

================================================================================
MSG: jsk_recognition_msgs/BoundingBox
# BoundingBox represents a oriented bounding box.
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
uint32 label

")



(provide :object_tracking/DetectedObjectMsg "a0ee17136c14b2823c04c42625441d79")


