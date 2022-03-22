
(cl:in-package :asdf)

(defsystem "object_tracking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :jsk_recognition_msgs-msg
               :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "DetectedObjectMsg" :depends-on ("_package_DetectedObjectMsg"))
    (:file "_package_DetectedObjectMsg" :depends-on ("_package"))
  ))