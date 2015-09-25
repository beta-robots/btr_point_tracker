#include "track_stamped.h"

TrackStamped::TrackStamped()
{

}

TrackStamped::~TrackStamped()
{
	
}

void TrackStamped::clear()
{
	track.clear();
}

void TrackStamped::setId(const unsigned int tId)
{
	id = tId;
}

unsigned int TrackStamped::getId() const
{
      return id;
}

void TrackStamped::addPoint(const double ts, const cv::Point2f & pt)
{
	time.push_back(ts);
	track.push_back(pt);
}

unsigned int TrackStamped::getTrackSize() const
{
	return this->track.size();
}

cv::Point2f & TrackStamped::getPoint(const unsigned int ii)
{
	return this->track[ii];
}

double TrackStamped::getX()
{
      return this->track.back().x;
}

double TrackStamped::getY()
{
      return this->track.back().y;
}


void TrackStamped::setVizColor()
{
        this->vizColor[0] = (int)(255.0*rand()/RAND_MAX);
        this->vizColor[1] = (int)(255.0*rand()/RAND_MAX);
        this->vizColor[2] = (int)(255.0*rand()/RAND_MAX);
}

cv::Scalar_<uchar> & TrackStamped::getVizColor()
{
	return vizColor; //cv::Scalar(this->vizColor[0],this->vizColor[1],this->vizColor[2]);
}

int TrackStamped::getIndexOfClosestPoint(const double tt, const double maxDt, const int ii0) const
{
        int ii, ii1;
        double dT, dTmin;

        //first check if tt is within time vector +- tolerance
        if ( ( tt < time.front()-maxDt) || ( tt > time.back()+maxDt) ) return -1;
        
        dTmin = 1e20; //initialized to some large value
        for (ii=ii0; ii<time.size(); ii++)
        {
                dT = fabs(time[ii] - tt);
                if (dT<dTmin) //we are approaching the 
                {
                        dTmin = dT;
                        ii1 = ii;
                }
                else break; //since time vector increases, once dT begins to increase we can break
        }

        return ii1;
}

int TrackStamped::getCorrespondence(const double t1, const double t2, const double maxDt, correspondencePair & pair) const
{
	unsigned int ii;
	int ii1,ii2;

      ii1 = getIndexOfClosestPoint(t1,maxDt);
	if ( ii1 == -1 ) return -1; //track has no entry closer enough to t1
	
      ii2 = getIndexOfClosestPoint(t2,maxDt,ii1);
	if ( ( ii2 == -1 ) || ( ii1 == ii2 ) ) return -1; //track has no entry closer enough to t2, or i1=i2

	//if we are here, set output pair
      pair.ts[0] = time[ii1];
      pair.ts[1] = time[ii2];
	pair.pt[0] = track[ii1];
	pair.pt[1] = track[ii2];
	return 0;
}

int TrackStamped::getCorrespondenceInterpolated(const double t1, const double t2, correspondencePair & pair) const
{
        unsigned int ii;
        int ii1,ii2;
        double a1,a2;

        //first check if the current track has at least two points to interpolate
        if ( this->track.size() < 2 ) return -1;

        //then check if t1 and t2 values are within time vector interval
        if ( ( t1 < time.front() ) || ( t1 > time.back() ) ) return -1;
        if ( ( t2 < time.front() ) || ( t2 > time.back() ) ) return -1;        

        //get tinme indexes corresponding to time stamps t1,t2
        ii1 = getIndexOfClosestPoint(t1,1.0);
        ii2 = getIndexOfClosestPoint(t2,1.0,ii1);

        //compute linear interpolation weight (a1) and first point
        if ( t1 >= time[ii1] ) 
        {
                a1 = (t1-time[ii1]) / (time[ii1+1]-time[ii1]);
                pair.pt[0].x = (1-a1)*track[ii1].x + a1*track[ii1+1].x; 
                pair.pt[0].y = (1-a1)*track[ii1].y + a1*track[ii1+1].y; 
        }
        else 
        {
                a1 = (time[ii1]-t1) / (time[ii1]-time[ii1-1]);
                pair.pt[0].x = (1-a1)*track[ii1].x + a1*track[ii1-1].x; 
                pair.pt[0].y = (1-a1)*track[ii1].y + a1*track[ii1-1].y;                 
        }

        //compute linear interpolation weight (a2) and second point
        if ( t2 >= time[ii2] ) 
        {
                a2 = (t2-time[ii2]) / (time[ii2+1]-time[ii2]);
                pair.pt[1].x = (1-a2)*track[ii2].x + a2*track[ii2+1].x; 
                pair.pt[1].y = (1-a2)*track[ii2].y + a2*track[ii2+1].y; 
        }
        else 
        {
                a2 = (time[ii2]-t2) / (time[ii2]-time[ii2-1]);
                pair.pt[1].x = (1-a2)*track[ii2].x + a2*track[ii2-1].x; 
                pair.pt[1].y = (1-a2)*track[ii2].y + a2*track[ii2-1].y;                 
        }
        
        //sets time stamps
        pair.ts[0] = t1;
        pair.ts[1] = t2;;
        
        //return success
        return 0;        
}

double TrackStamped::getLastTs() const
{
        return time.back();
}

bool TrackStamped::isLastPoint(const cv::Point2f & pt) const
{
	if ( this->track.back() == pt ) 
		return true;
	else 
		return false;
}

int TrackStamped::getTrackVelocity(const double t1, const double t2, const double maxDt, cv::Point2f & pxV) const
{
        unsigned int ii, ii1,ii2;
        double dT;
   
        ii1 = getIndexOfClosestPoint(t1,maxDt);
        if ( ii1 == -1 ) return -1; //track has no entry closer enough to t1

        ii2 = getIndexOfClosestPoint(t2,maxDt);
        if ( ( ii2 == -1 ) || ( ii1 == ii2 ) ) return -1; //track has no entry closer enough to t2, or i1=i2

        pxV.x = (track[ii2].x - track[ii1].x) / (t2-t1);
        pxV.y = (track[ii2].y - track[ii1].y) / (t2-t1);
        
        return 1;
}

void TrackStamped::getTrackVelocity(cv::Point2f & pxV) const
{
        pxV.x = (track.back().x - track.front().x) / (time.back() - time.front());
        pxV.y = (track.back().y - track.front().y) / (time.back() - time.front()); 
}

void TrackStamped::print() const 
{
	unsigned int ii;
	std::cout << "TrackID: " << id << std::endl;
	std::cout << "Size: " << track.size() << std::endl; 
	for (ii=0; ii<track.size(); ii++ )
	{
		std::cout << time[ii] << "; (" << track[ii].x << "," << track[ii].y << ")" << std::endl;
	}
	std::cout << std::endl;
}
