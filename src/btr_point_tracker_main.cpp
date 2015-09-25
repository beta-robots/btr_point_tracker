//ros dependencies
#include "btr_point_tracker_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "btr_point_tracker_node");
      
      //create ros wrapper object
      BtrPointTrackerNode tracker;
      
      //set node loop rate
      ros::Rate loop_rate(20);
      
      //node loop 
      while ( ros::ok() )
      {
            //if new image , do things
            if ( tracker.newImage() )
            {
                  tracker.process();
                  tracker.publishImage();
                  tracker.publishTracks();
            }
            
            //execute pending callbacks
            ros::spinOnce(); 
            
            //relax to fit output rate
            loop_rate.sleep();
      }
            
      //exit program
      return 0;
}