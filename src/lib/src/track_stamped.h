#ifndef track_stamped_H
#define track_stamped_H

//std
#include <iostream>
#include <vector>
#include <list>
#include <cmath>

//opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

/** \brief A point pair correspondence
 * 
 * A point pair correspondence with respective time stamps 
 * 
 **/
struct correspondencePair
{
      /** \brief Time stamps 
      * 
      * Time stamp of each point [seconds]
      *
      **/
      double ts[2];

      /** \brief Point image coordinates
      * 
      * Image coordinates of the two corresponding points [pixels]
      *
      **/	
      cv::Point2f pt[2];

      /** \brief Print pair data
      * 
      * Print corresponcence pair data
      * 
      **/
      void print()
      {
            std::cout << ts[0] << "; (" << pt[0].x << "," << pt[0].y << ")" << std::endl;
            std::cout << ts[1] << "; (" << pt[1].x << "," << pt[1].y << ")" << std::endl;
      }
};

/** \brief Point feature track 
 * 
 * Track coordinates of a point feature over a set of time stamped frames
 * 
 **/
class TrackStamped
{
      protected:
		/** \brief Track id
		* 
		* Track id
		* 
		**/
		unsigned int id;

            /** \brief viusalization color 
            * 
            * assigned color for visualization purposes (rgb channels)
            *
            **/
            cv::Scalar_<uchar> vizColor;                
		
		/** \brief Time stamp vector
		* 
		* Time stamp of each point entry of the track record
		* 
		**/
		std::vector<double> time;
		
		/** \brief track vector
		* 
		* image coordinates of track points 
		* 
		**/
		cv::vector<cv::Point2f> track;
			
	public:
		/** \brief Constructor
		 * 
		 * Constructor
		 * 
		 **/
		TrackStamped();

		/** \brief Destructor
		 * 
		 * Destructor
		 * 
		 **/
		~TrackStamped();
		
		/** \brief Clear track and time vectors
		 * 
		 * Clear track and time vector
		 *
		 **/
		void clear();
		
		/** \brief Sets id
		 * 
		 * Sets id
		 * 
		 **/
		void setId(const unsigned int tId);
		
            /** \brief Gets id
             * 
             * Gets id
             * 
             **/
            unsigned int getId() const;

            /** \brief Push a new entry to the track
		 * 
		 * Push a new time/point entry to the track
		 * 
		 **/
		void addPoint(const double ts, const cv::Point2f & pt);
		
		/** \brief Returns track size
		 * 
		 * Returns track size
		 * 
		 **/
		unsigned int getTrackSize() const;
		
		/** \brief Returns i^th point of the track
		 * 
		 * Returns i^th point of the track
		 * 
		 **/
            cv::Point2f & getPoint(const unsigned int ii);
            
            /** \brief Returns last X pixel coordinate of i^th point of the track
             * 
             * Returns last X pixel coordinate of i^th point of the track
             * 
             **/
            double getX();
            
            /** \brief Returns last Y pixel coordinate of i^th point of the track
             * 
             * Returns last Y pixel coordinate of i^th point of the track
             * 
             **/
            double getY();
            
            /** \brief Sets a visualization color
            *
            * Randomly sets a visualization color 
            * 
            **/
            void setVizColor();                
		
		/** \brief Returns visualization color 3-vector
		 * 
		 * Returns visualization color 3-vector (alpha channel not set)
		 * 
		 **/
		cv::Scalar_<uchar> & getVizColor();

            /** \brief Gets the index of the time stamp closest to tt
            * 
            * Gets the index of the time stamp closest to tt, assuming time vector is ordered and with a timeStamp
            * difference to tt less than maxDt
            * Input: 
            *      tt: time stamp
            *      maxDt: maximum tolerated time distance
            *      ii0: is the initial track index from which the function starts the search. Default value is zero.
            * Return value:
            *      (-1) No point was found in the track fulfilling the conditions (tt,maxDt)
            *      (>=0) track index correponding to the point 
            * 
            **/
            int getIndexOfClosestPoint(const double tt, const double maxDt, const int ii0 = 0) const;
                
		/** \brief Gets corresponcence pair
		 * 
		 * Gets corresponcence pair closer to provided time stamps
		 * Inputs: 
		 * 	- t1: a time stamp
		 *	- t2: a time stamp after (bigger) than t1
		 * 	- maxDt: time tolerance within provided t1,t2 and point records
		 * Outputs:
		 * 	- return value: 
		 * 		(-1) if no pair corresponcence is found within t1 & t2 considering maxDt
		 * 		( 0) if a correct pair is found
		 * 	- pair: 
		 * 		The point corresponcence pair 
		 * 
		 **/
            int getCorrespondence(const double t1, const double t2, const double maxDt, correspondencePair & pair) const;
                
            /** \brief Gets interpolated correspondence pair
            * 
            * Gets corresponcence pair according to provided time stamps, but point coordinates are not directly 
            * track records, but instead they are computed as linear interpolation between time stamp closest points
            * between t1 and t2
            * Inputs: 
            *      - t1: a time stamp
            *      - t2: a time stamp after (bigger) than t1
            * Outputs:
            *      - return value: 
            *              (-1) if no pair corresponcence is found within t1 & t2
            *              ( 0) if a correct pair is found
            *      - pair: 
            *              The interpolated point corresponcence pair 
            * 
            **/
            int getCorrespondenceInterpolated(const double t1, const double t2, correspondencePair & pair) const;                
		
		/** \brief Retunrs last time stamp
		 * 
		 * Returns last time stamp
		 * 
		 **/
		double getLastTs() const;

		/** \brief Checks if provided point is the last of this track
		 * 
		 * Returns true if provided point is the last of this track
		 * Returns false otherwise
		 * 
		 **/
		bool isLastPoint(const cv::Point2f & pt) const;
                
            /** \brief Computes pixel velocity 
            * 
            * Computes pixel velocity between points of a given track.
            * Points are those just before t1 and just before t2, both with time difference least than maxDt 
            * 
            * Outputs:
            *      - return value: 
            *              (-1) If track does not fulfill timing requests
            *              ( 0) If track fulfills timing requests (t1,t2)
            *      - pxV: 
            *              pixel velocity 
            **/
            int getTrackVelocity(const double t1, const double t2, const double maxDt, cv::Point2f & pxV) const;
                
            /** \brief Computes pixel velocity 
            * 
            * Computes pixel velocity between start and end points of the track
            * Sets result to pxV
            * 
            **/
            void getTrackVelocity(cv::Point2f & pxV) const;
                	
		/** \brief Print track
		* 
		* Print track data to stdout
		* 
		**/
		void print() const;
};
#endif
