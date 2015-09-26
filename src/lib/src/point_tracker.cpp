#include "point_tracker.h"

PointTracker::PointTracker()
{
	//initializes window
	cv::namedWindow("Point Tracker",CV_WINDOW_AUTOSIZE); //| CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);//, CV_WINDOW_AUTOSIZE);
	
	//init fs_index, iteration counter
	fs_index = 0;
	iterationId = 0;
	nextTrackId = 1;
      
      params.featureType = NONE;
}

PointTracker::~PointTracker()
{
	trackList.clear();
	cv::destroyWindow("Point Tracker");

      if( params.featureType != NONE ) //inidcates that setParameters() has been called, so detector,descriptor and matcher has been allocated
      {
            switch(params.featureType)
            {
//                   case SURF_FEATURES:
//                         surfDetector.release();
//                         break;
                  case ORB_FEATURES:
                        orbDetector.release();
                        break;
                  case BRISK_FEATURES:
                        briskDetector.release();
                        break;                  
                  default:
                        break;
            }                  
            fDescriptor.release();
            matcher.release();
      }
}

void PointTracker::setParameters(pointTrackerParams & prms)
{
      //get params
      params.verbose = prms.verbose;
//       params.min_hessian = prms.min_hessian;
      params.min_features = prms.min_features;
      params.max_correspondence_dist = prms.max_correspondence_dist;
      params.old_track_latency = prms.old_track_latency;
      params.featureType = prms.featureType;
      params.viewMode = prms.viewMode;
            
      //init feature detector
      switch(params.featureType)
      {
//             case SURF_FEATURES:
//                         surfDetector = new cv::SURF(params.min_hessian);
//                         fDescriptor = cv::DescriptorExtractor::create("SURF");
//                         if (params.verbose) std::cout << __LINE__ << ": SURF Detector & Extractor allocated" << std::endl;
//                         break;      
            case ORB_FEATURES:
                        orbDetector = new cv::ORB(params.min_features);
                        fDescriptor = cv::DescriptorExtractor::create("ORB");
                        if (params.verbose) std::cout << __LINE__ << ": ORB Detector & Extractor allocated" << std::endl;
                        break;
            case BRISK_FEATURES:
                        briskDetector = new cv::BRISK();
                        fDescriptor = cv::DescriptorExtractor::create("BRISK");
                        if (params.verbose) std::cout << __LINE__ << ": BRISK Detector & Extractor allocated" << std::endl;
                        break;      
            default:
                  std::cout << __LINE__ << ": WARNING: PointTracker::CfeatureTracker(): Unknown feature type: " << params.featureType << ". No detector created" << std::endl;
                  break;                  
      }
      
      //init feature matcher
      matcher = cv::Ptr<cv::BFMatcher>(new cv::BFMatcher(cv::NORM_L2,false));      
}

void PointTracker::setDefaultParameters()
{
      pointTrackerParams pms;
      
      pms.verbose = false;
//       pms.min_hessian = MIN_HESSIAN_DEFAULT;
      pms.min_features = MIN_FEATURES_DEFAULT;
      pms.max_correspondence_dist = MAX_CORRESPONDENCE_DIST_DEFAULT;
      pms.old_track_latency = OLD_TRACK_LATENCY_DEFAULT;
      pms.featureType = ORB_FEATURES;
      pms.viewMode = VIEW_TRACKS;
      
      this->setParameters(pms);
}

void PointTracker::switchBuffers()
{
	fs_index = !fs_index;
}

void PointTracker::setTimeStamp(const double ts)
{
      frameTS = ts;
}

double PointTracker::getTimeStamp() const
{
      return frameTS;
}

std::list<TrackStamped> & PointTracker::getTrackList()
{
      return this->trackList;
}

void PointTracker::getDisplayImage(cv::Mat & img)
{
      img = outputImage.clone();
}

void PointTracker::findFeatures(const cv::Mat & vFrame, bool equalize)
{	
	cv::Mat vFrame1c[3];
	cv::Mat eq1c[3];

	if (equalize)
	{
// 		cv::split(vFrame,vFrame1c); //split RGB channels
// 		cv::equalizeHist(vFrame1c[0],eq1c[0]); //equalize c0
// 		cv::equalizeHist(vFrame1c[1],eq1c[1]); //equalize c1
// 		cv::equalizeHist(vFrame1c[2],eq1c[2]); //equalize c2
// 		cv::merge(eq1c,3,frame[fs_index]); //merge 3 channels to RGB image
	}
	else 
	{
		frame[fs_index] = vFrame;//.clone(); //copy the image "as is" to the frame buffer
	}

	//clear points of "before-last" iteration (t-2)
	featureSet[fs_index].clear();

	//1. Detect point-features using some Detector
      switch(params.featureType)
      {
//             case SURF_FEATURES:
//                   surfDetector->detect(frame[fs_index], featureSet[fs_index]);//, imageMask);
//                   break;
            case ORB_FEATURES:
                  orbDetector->detect(frame[fs_index], featureSet[fs_index]);//, imageMask);
                  break;
            case BRISK_FEATURES:                  
                  briskDetector->detect(frame[fs_index], featureSet[fs_index]);//, imageMask);
                  break;                  
            default:
                  break;
      }
      
      //2. Extract descriptors for each detected feature. 
      fDescriptor->compute(frame[fs_index],featureSet[fs_index],descriptors[fs_index]);
        
      if (params.verbose) std::cout << __LINE__ << ": Number of point features found: " << featureSet[fs_index].size() << std::endl;
}

void PointTracker::findCorrespondences()
{
	std::vector<int> indexPast, indexCurrent;
	unsigned int ii, jj;
	cv::Point2f p1, p2, auxPt;
	double dd;
	
	//reset vectors
	matches.clear();
	pointSet[fs_index].clear();
	pointSet[!fs_index].clear();

	//check for the existence of features
	if ( (featureSet[fs_index].size() == 0) || (featureSet[!fs_index].size() == 0) )
	{
		if (params.verbose) std::cout << __LINE__ << ": PointTracker::findCorrespondences(): Unable to find correspondences. Empty set !" << std::endl;
		return;
	}
		
	//set matching mask only allowing "small vehicle motion" correspondences 
	slowMotionMask = cv::Mat::zeros(featureSet[!fs_index].size(), featureSet[fs_index].size(), CV_8UC1);
	for(ii=0; ii<featureSet[!fs_index].size(); ii++)
	{
		p1.x = featureSet[!fs_index].at(ii).pt.x;
		p1.y = featureSet[!fs_index].at(ii).pt.y;
		for(jj=0; jj<featureSet[fs_index].size(); jj++)
		{
			p2.x = featureSet[fs_index].at(jj).pt.x;
			p2.y = featureSet[fs_index].at(jj).pt.y;
			dd = cv::norm(p1-p2);
			if ( dd < MAX_CORRESPONDENCE_DIST_DEFAULT )
			{
				slowMotionMask.at<unsigned char>(ii,jj) = 255; //allows ij correspondence
				//std::cout << ii << "," << jj << ": " << "dd = " << dd << std::endl;
			}
		}
	}

	//compute matches
      matcher->match(descriptors[!fs_index],descriptors[fs_index], matches, slowMotionMask);
      if (params.verbose) std::cout << __LINE__ << ": Number of correspondences found: " << matches.size() << std::endl;
      
      // Convert keypoints into Point2f
      for (ii=0; ii<matches.size(); ii++) 
      {
            auxPt.x = featureSet[!fs_index].at(matches[ii].queryIdx).pt.x;
            auxPt.y = featureSet[!fs_index].at(matches[ii].queryIdx).pt.y;	
            pointSet[!fs_index].push_back((auxPt));
            auxPt.x = featureSet[fs_index].at(matches[ii].trainIdx).pt.x;
            auxPt.y = featureSet[fs_index].at(matches[ii].trainIdx).pt.y;	
            pointSet[fs_index].push_back((auxPt));
      }
}

void PointTracker::updateTracks()
{
	std::list<TrackStamped> newTracks;
	std::list<TrackStamped>::iterator iiTrack;	
	unsigned int ii;
	TrackStamped auxTrack;
	bool isNewTrack;
	
	for (ii=0; ii<pointSet[fs_index].size(); ii++ )
	{	
		//first check if point was already tracked
		isNewTrack = true;//sets flag to true
		for(iiTrack = trackList.begin(); iiTrack != trackList.end(); iiTrack++)
		{
			if ( iiTrack->isLastPoint(pointSet[!fs_index][ii]) ) //matching case
			{
				isNewTrack = false;
				iiTrack->addPoint(this->frameTS,pointSet[fs_index][ii]);
				break;
			}
		}
		
		//point ii was not tracked, so a new track has to be created
		if (isNewTrack)
		{
			if (params.verbose) std::cout << __LINE__ << ": New track!" << std::endl;
			auxTrack.setId(nextTrackId);
			nextTrackId ++;
			auxTrack.setVizColor();
			auxTrack.addPoint(this->frameTS,pointSet[fs_index][ii]);
			newTracks.push_back(auxTrack);
			auxTrack.clear();
		}
	}
	
	//remove old tracks , older than params.old_track_latency
	iiTrack = trackList.begin(); 
	while( iiTrack != trackList.end() )
	{
            if ( iiTrack->getLastTs() < (this->frameTS-params.old_track_latency) ) 
		{
			iiTrack = trackList.erase(iiTrack);//erase returns an iterator to the next element after the erased one
		}
		else
		{
			iiTrack++;
		}
	}
	
	//add new tracks to the trackList set
	for(iiTrack = newTracks.begin(); iiTrack != newTracks.end(); iiTrack++)
	{
		trackList.push_back(*iiTrack);
	}
	newTracks.clear();
}

bool PointTracker::buildOutputImage()
{
      bool outputImageReady = false;
      std::list<TrackStamped>::iterator iiTrack;     
      cv::vector<cv::KeyPoint> keyPts;
      unsigned int ii;
      std::ostringstream ssVlabel;
      cv::Point2f pxV;
            
      //build image to be shown
      switch(params.viewMode)
      {
            case VIEW_POINTS: //Draw keypoints to outputImage
                  cv::drawKeypoints( frame[fs_index], featureSet[fs_index], outputImage, 255, cv::DrawMatchesFlags::DEFAULT );      
                  outputImageReady = true;
                  break;
                  
            case VIEW_CORRESPONDENCES: //draws matching results
                  if ( matches.size() != 0 )
                  {
                        cv::drawMatches(frame[!fs_index], featureSet[!fs_index], frame[fs_index], featureSet[fs_index], matches, outputImage,cv::Scalar::all(-1),0, std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);//DRAW_RICH_KEYPOINTS
                        outputImageReady = true;
                  }
                  break;
                  
            case VIEW_TRACKS:
            case VIEW_TRACKS_VELOCITY:
                  outputImage = frame[fs_index];
                  for(iiTrack = trackList.begin(); iiTrack != trackList.end(); iiTrack++)
                  {
                        if ( iiTrack->getTrackSize() > 10 ) //&& ( (this->frameTS - iiTrack->getLastTs()) < params.old_track_latency ) )
                        {
                              for(ii=0; ii<iiTrack->getTrackSize(); ii++)
                              {
                                    keyPts.push_back(cv::KeyPoint(iiTrack->getPoint(ii),1));
                              }
                              if ( params.viewMode == VIEW_TRACKS_VELOCITY )
                              {
                                    ssVlabel.str("");
                                    iiTrack->getTrackVelocity(pxV);
                                    ssVlabel << (int)cv::norm(pxV);
                                    cv::putText(outputImage,ssVlabel.str(),iiTrack->getPoint(ii-1),cv::FONT_HERSHEY_SIMPLEX,0.4,iiTrack->getVizColor(),1);
                              }                                                
                  
                              cv::drawKeypoints( outputImage, keyPts, outputImage, iiTrack->getVizColor(), cv::DrawMatchesFlags::DEFAULT );                                             
                              keyPts.clear();
                              if (params.verbose) iiTrack->print();
                        }
                  }
                  //cv::drawKeypoints( frame[fs_index], keyPts, outputImage, 255, cv::DrawMatchesFlags::DEFAULT );      
                  outputImageReady = true;
                  break;
            
            case VIEW_CORRESPONDENCE_MASK:
                  if ( (slowMotionMask.rows !=0) && (slowMotionMask.cols !=0) ) 
                  {
                        outputImage = slowMotionMask;
                        outputImageReady = true;
                  }
                  break;
                  
            default:
                  std::cout << __LINE__ << ": PointTracker::displayOutput(). Unknown View mode. viewMode: " << params.viewMode << std::endl;
                  break;            
      }      
      
      return outputImageReady;
}

void PointTracker::displayOutputImage()
{
	if( !outputImage.empty() )
	{
            //show image
		cv::imshow("Point Tracker", outputImage);// show the image		
		cv::waitKey(50); // wait for a key
      }
}

void PointTracker::saveOutputImage(std::string folderName)
{
      std::ostringstream  fileName;
      std::vector<int> imgParams;

      if( !outputImage.empty() )
      {
            fileName << folderName << iterationId << ".jpg";
            imgParams.push_back(CV_IMWRITE_JPEG_QUALITY);
            imgParams.push_back(100);
            cv::imwrite(fileName.str(), outputImage, imgParams);
            iterationId++;
      }
}

void PointTracker::printConfig() const
{
      std::cout << "Tracker Configuration:" << std::endl 
            << "    verbose: " << params.verbose << std::endl
//             << "    min_hessian: " << params.min_hessian << std::endl
            << "    min_features: " << params.min_features << std::endl
            << "    max_correspondence_dist: " << params.max_correspondence_dist << std::endl
            << "    old_track_latency: " << params.old_track_latency << std::endl
            << "    featureType: " << params.featureType << std::endl
            << "    viewMode: " << params.viewMode << std::endl;
}
