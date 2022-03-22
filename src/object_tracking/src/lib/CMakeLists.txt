add_library(tracking
            BasicObjectTracking.cpp
            ObjectTracking.cpp
            ObjectTrackingIndoor.cpp
            ObjectTrackingOutdoor.cpp
            ObjectTrackingDynamicCarrier.cpp
            ObjectTrackingIndoorDynamicCarrier.cpp
            DetectedObject.cpp)

target_link_libraries(tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES})