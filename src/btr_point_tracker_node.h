#ifndef btr_point_tracker_node_H
#define btr_point_tracker_node_H

//std
#include <string>
#include <list>

//this package
#include "point_tracker.h"

//ros dependencies
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/image_encodings.h>
#include <btr_point_tracker/PointTracks.h> //custom message
#include <btr_point_tracker/tracker_paramsConfig.h> //configs

/** \brief wrapping class
 * 
 * Wraps functionality of PointTracker object, while implements ROS layer (comms, configs)
 * 
 **/
class BtrPointTrackerNode
{
      protected:
            
            //point tracker object
            PointTracker tracker;
            
            //ros node handle
            ros::NodeHandle nh;
            
            //image publisher/subscriber
            image_transport::ImageTransport it;
            image_transport::Publisher imagePub;            
            cv_bridge::CvImage cvImgPub;
            image_transport::Subscriber imageSubs;            
            cv_bridge::CvImagePtr cvImgPtrSubs;            

            //circle set publisher
            btr_point_tracker::PointTracks tracksMsg;            
            ros::Publisher tracksPub;
            
            //camera info subscriber
            sensor_msgs::CameraInfo cameraInfoMsg;
            ros::Subscriber cameraInfoSubs;

            //dynamic reconfigure 
            btr_point_tracker::tracker_paramsConfig config; 
            
            //flag indicating a new image has been received
            bool newImageFlag;
                        
            //img encoding id
            std::string imgEncoding;            
            
            //image time stamp
            unsigned int tsec;
            unsigned int tnsec;            

      protected:
            //callback to image subscription
            void imageCallback(const sensor_msgs::ImageConstPtr & msg);
            
            //callback to camera info subscription
            void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);
            
      public:
            //constructor
            BtrPointTrackerNode();
            
            //destructor
            ~BtrPointTrackerNode();
            
            //checks if a new image has been received
            bool newImage();
            
            //execute point tracker
            void process();        
            
            //publish the output image (input image + marked tracks)
            void publishImage();
            
            //publish track data
            void publishTracks();
};
#endif
