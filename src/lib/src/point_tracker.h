#ifndef point_tracker_H
#define point_tracker_H

//std
#include <iostream>
#include <vector>
#include <list>
#include <cmath>

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
// #include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

//this lib
#include "track_stamped.h"

//constants
const int MIN_HESSIAN_DEFAULT = 250;//500 //SURF features 
const unsigned int MIN_FEATURES_DEFAULT = 30; //ORB features
const double MAX_CORRESPONDENCE_DIST_DEFAULT = 10; //[pixels]
const double OLD_TRACK_LATENCY_DEFAULT = 0.5; //keep old tracks during a short time period before remove them
enum featureTypeEnum {NONE=0, SURF_FEATURES, ORB_FEATURES, BRISK_FEATURES};
enum viewModeEnum {VIEW_POINTS=1, VIEW_CORRESPONDENCES, VIEW_TRACKS, VIEW_TRACKS_VELOCITY, VIEW_CORRESPONDENCE_MASK};

//structs
/** \brief Configuration parameters for point tracker
 * 
 * Configuration parameters for point tracker
 * 
 **/
struct pointTrackerParams
{
      bool verbose; //true-> executio in verbose mode
      int min_hessian;//hessian lower threshold (SURF)
      unsigned int min_features; //minimum number of features (ORB)
      double max_correspondence_dist; //maximum distance in pixels allowed for a correspondence pair [pixels]
      double old_track_latency; //indicates how many seconds track will remain on visualization image [s]
      featureTypeEnum featureType;
      viewModeEnum viewMode; 
};


class PointTracker
{
	protected: 
		/**
		 * 
		 * Index indicating the current buffer for image and point sets
		 * 
		 **/
		unsigned int fs_index;

		/**
		 * 
		 * Number of current iteration
		 * 
		 **/		
		unsigned int iterationId;

		/**
		 * 
		 * Id to be set to the next point track
		 * 
		 **/			
		unsigned int nextTrackId;

		/**
		 * 
		 * Output Image to be displayed for visualization purposes
		 * 
		 **/		
		cv::Mat outputImage;
		
		/**
		 * 
		 * Previous and current image frame. 
		 * Current is indicated by frame[fs_index] and previous by frame[!fs_index]
		 * 
		 **/				
		cv::Mat frame[2]; 
		
		/**
		 * 
		 * matching mask allowing only point corresponcences closer enough between consecutive frames
		 * 
		 **/
		cv::Mat slowMotionMask;
		
		/**
		 * 
		 * Time stamp of the current image frame
		 * 
		 **/
            double frameTS;
		
            /** \brief Pointer to a Feature detector
            * 
            * Pointer to a Feature detector. It is allocated at setParameters(), depending on params.featureType
            * 
            **/
//             cv::Ptr<cv::SURF> surfDetector;
            cv::Ptr<cv::ORB> orbDetector;
            cv::Ptr<cv::BRISK> briskDetector;
                
		/**
		 * 
		 * Set of visual features keypoints (image coordinates)
		 * Double buffer corresponding to current, featureSet[fs_index], and previous featureSet[!fs_index]
		 * 
		 **/
		cv::vector<cv::KeyPoint> featureSet[2]; 

            /** \brief Pointer to a Feature descriptor extractor
            * 
            * Pointer to a Feature descriptor extractor. It is allocated at setParameters(), depending on params.featureType
            * 
            **/
            cv::Ptr<cv::DescriptorExtractor> fDescriptor;
		
		/** \brief Set of feature descriptors. 
		 * 
		 * Set of feature descriptors. 
             * Double buffer corresponding to current, descriptors[fs_index], and previous descriptors[!fs_index]
		 * 
		 **/
		cv::Mat descriptors[2]; 
		
		/** \brief Pointer to the matcher object
		 * 
		 * Matcher object to find corresponcences between feature descriptors
		 * 
		 **/
		cv::Ptr<cv::BFMatcher> matcher; //(2,true);
		
		/** \brief Set of feature matches
		 * 
		 * Set of feature matches. Output result of the feature matcher
		 * 
		 **/
		cv::vector<cv::DMatch> matches;
		
		/** \brief Vector of point pair correspondences
		 * 
		 * Point coordinates corresponding to matched feature pairs.
		 * pointSet[fs_index] is related to current image frame and pointSet[!fs_index] relates to the previous image frame
		 * 
		 **/
		cv::vector<cv::Point2f> pointSet[2]; 
		
		/** \brief list of current tracks
		 * 
		 * Track of all point features
		 * 
		 **/
		std::list<TrackStamped> trackList;
            
            /** \brief Configuration parameters
             * 
             * Configuration parameters
             * 
             **/
            pointTrackerParams params; 

	public:
		/** \brief Constructor
		 * 
		 * Constructor 
		 * 
		 **/
            PointTracker();

		/** \brief Destructor
		 * 
		 * Destructor 
		 * 
		 **/
		~PointTracker();
            
            /** \brief Sets configuration
             * 
             * Sets configuration parameters to this tracker
             * 
             **/
            void setParameters(pointTrackerParams & prms);
            
            /** \brief Sets default configuration
             * 
             * Sets default configuration parameters to this tracker
             * 
             **/
            void setDefaultParameters();            
		
		/** \brief Switc buffer index
		 * 
		 * Switch buffer index. 
		 * 
		 **/
		void switchBuffers();

		/** \brief Sets time stamp
		 * 
		 * Sets time stamp with the provided value
		 * 
		 **/
		void setTimeStamp(const double ts);

            /** \brief returns time stamp value
            * 
            * Gets time stamp value
            * 
            **/
            double getTimeStamp() const;

            /** \brief Returns a reference to the track list
            * 
            * Returns a reference to the track list
            * 
            **/
            std::list<TrackStamped> & getTrackList();                
            
            /** \brief Gets output image
            * 
            * Sets to img a clone copy of this->outputImage
            * 
            **/
            void getDisplayImage(cv::Mat & img);

		/** \brief Find features in vFrame image. 
		 * 
		 * Find features in vFrame image. 
		 * Sets point detections to this->featureSet[fs_index] and feature descriptors to this->descriptors[fs_index]
		 * 
		 **/
		void findFeatures(const cv::Mat & vFrame, bool equalize=true);
		
		/** \brief Find matches between previous and current descriptors
		 * 
		 * Finds matches between current and previous frame. Sets pointSet
		 * 
		 **/
		void findCorrespondences();
		
		/** \brief Update tracks
		 * 
		 * Updates tracks by creating new tracks when no previous match was found
		 * and removing them if a track has no detections during an OLD_TRACK_LATENCY period
		 * 
		 **/
		void updateTracks();
		
            /** \brief build marked image
             * 
             * Build output image, which is a composition of frame[fs_index] with some overpainted marks 
             * Returns true if all ok, false if an error occurs
             * 
             **/
            bool buildOutputImage();
            
            /** \brief Visualize marked image
		 * 
		 * Visualize outputImage, which is a composition of frame[fs_index] with some overpainted marks 
		 * 
		 **/
		void displayOutputImage();
            
            /** \brief Save marked image
             * 
             * Save outputImage, which is a composition of frame[fs_index] with some overpainted marks 
             * 
             **/
            void saveOutputImage(std::string folderName = "");
            
            /** \brief Prints current parameters
             * 
             * Prints current configuration parameter values to std out
             * 
             **/
            void printConfig() const; 
};
#endif

