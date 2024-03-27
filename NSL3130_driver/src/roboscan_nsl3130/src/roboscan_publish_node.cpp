#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <roboscan_nsl3130/roboscan_nsl3130Config.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cartesian_transform.hpp"
#include "interface.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <visualization_msgs/Marker.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/LaserScan.h>


#define WIN_NAME "NSL-3130AA IMAGE"
#define MAX_LEVELS  	9
#define NUM_COLORS     	30000


#define PIXEL_VALID_DATA  	64000
#define LOW_AMPLITUDE       64001
#define ADC_OVERFLOW        64002
#define SATURATION          64003
#define BAD_PIXEL           64004
#define INTERFERENCE        64007
#define EDGE_FILTERED       64008

#define LEFTX_MAX	124	
#define RIGHTX_MIN	131
#define RIGHTX_MAX	319	
#define X_INTERVAL	4

#define LEFTY_MAX	116	
#define RIGHTY_MIN	123
#define RIGHTY_MAX	239	
#define Y_INTERVAL	2

#define USED_INTENSITY //else gray pointcloud

using namespace nanosys;
using namespace cv;

int imageType = 2; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int lensType = 2;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
bool medianFilter;
bool averageFilter;
double temporalFilterFactor;
int temporalFilterThreshold;
int edgeThreshold;
int temporalEdgeThresholdLow;
int temporalEdgeThresholdHigh;
int interferenceDetectionLimit;
bool startStream;
bool useLastValue;
bool publishPointCloud;
bool cartesian;
int channel;
int frequencyModulation;
int int0, int1, int2, intGr; //integration times
int hdr_mode; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
int minAmplitude;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;
int old_lensCenterOffsetX = 0;
int old_lensCenterOffsetY = 0;

int dual_beam = 0;
bool used_dual_beam_distance = true;

int roi_leftX = 0;
int roi_topY = 0;
int roi_rightX = 319;
int roi_bottomY = 239;

const int width   = 320;
const int width2  = 160;
const int height  = 240;
const int height2 = 120;
const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

uint8_t grayscaleIlluminationMode = 0;
int8_t bAdcOverflow = 1;
int8_t bSaturation = 1;

double transformAngle = 0;
int cutPixels = 0;
bool cvShow = false;
float maxDistance;
int mouseXpos = 0;
int mouseYpos = 0;
char winName[100];
std::vector<cv::Vec3b> colorVector;

//2D scan data
unsigned int num_readings = 320;
double laser_frequency = 200;
double ranges[320];
double intensities[320];

int count = 0;      
//

uint32_t frameSeq;
boost::signals2::connection connectionFrames;
boost::signals2::connection connectionCameraInfo;

ros::Publisher distanceImagePublisher;
ros::Publisher amplitudeImagePublisher;
ros::Publisher dcsImagePublisher;
ros::Publisher grayImagePublisher;

image_transport::Publisher imagePublisher;

ros::Publisher cameraInfoPublisher;
ros::Publisher pointCloud2Publisher;
ros::ServiceServer cameraInfoService;

Interface interface;
CartesianTransform cartesianTransform;
sensor_msgs::CameraInfo cameraInfo;

//ros::Publisher markerPub;
ros::Publisher scanPub;
//=======================================================================

typedef struct _RGB888Pixel
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RGB888Pixel;

double interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}

void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    double k = 1;
    double BIT0 = -0.125 * k - 0.25;
    double BIT1 = BIT0 + 0.25 * k;
    double BIT2 = BIT1 + 0.25 * k;
    double BIT3 = BIT2 + 0.25 * k;

    double G0 = BIT1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = BIT2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;

    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= BIT0 && i < BIT1 ){
        blue = interpolate(i, BIT0, 0, BIT1, 255);
    } else if((i >= BIT1)&&(i < BIT2)){
        blue = 255;
    } else if((i >= BIT2)&&(i < BIT3)) {
        blue = interpolate(i, BIT2, 255, BIT3, 0);
    } else{
        blue = 0;
    }

}


static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		mouseXpos = x;
		mouseYpos = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}

void setWinName(bool configCvShow)
{
	bool changedCvShow = cvShow != configCvShow ? true : false;	
	cvShow = configCvShow;

	printf("cvShow = %d/%d\n", configCvShow, changedCvShow);

	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( configCvShow == false || changedCvShow == false ) return;

	if( imageType == Frame::GRAYSCALE ){
		sprintf(winName,"%s(Gray)", WIN_NAME);
	}
	else if( imageType == Frame::DISTANCE ){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( imageType == Frame::DISTANCE_AMPLITUDE ){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else if( imageType == Frame::DCS ){
		sprintf(winName,"%s(DCS)", WIN_NAME);
	}
	else if( imageType == Frame::DISTANCE_GRAYSCALE ){
		sprintf(winName,"%s(Dist/Gray)", WIN_NAME);
	}
	else if( imageType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
		sprintf(winName,"%s(Dist/Ampl/Gray)", WIN_NAME);
	}

	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	//cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}


int Convert_To_RGB24( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
{
    if(fValue == ADC_OVERFLOW)
    {
        nRGBData->r = 169;//R
        nRGBData->g = 14;//G
        nRGBData->b = 255;//B
    }
    else if(fValue == SATURATION)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 128;//B
    }
    else if(fValue == INTERFERENCE)
    {
        nRGBData->r = 0;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue == 0) //Invalide Pixel
    {
        nRGBData->r = 0;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue < fMinValue)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 0;//B
    }
    else if(fValue > fMaxValue)
    {
        nRGBData->r = 255;//R
        nRGBData->g = 0;//G
        nRGBData->b = 255;//B
    }
    else
    {
        float fColorWeight;
        fColorWeight = (fValue-fMinValue) / (fMaxValue-fMinValue);

        if( (fColorWeight <= 1.0f) && (fColorWeight > 0.8f) )
        {
            nRGBData->r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));//값에 따라 증가
            nRGBData->g = 0;
            nRGBData->b = 255;
        } 
        else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
        {
            nRGBData->r = 0;
            nRGBData->g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));//값에 따라 감소
            nRGBData->b = 255;
        }
        else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
        {
            nRGBData->r = 0;
            nRGBData->g = 255;
            nRGBData->b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));//값에 따라 증가
        }
        else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
        {
            nRGBData->r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));//값에 따라 감소
            nRGBData->g = 255;
            nRGBData->b = 0;
        }
        else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
        {
            nRGBData->r = 255;
            nRGBData->g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));//값에 따라 증가
            nRGBData->b = 0;
        }
        else
        {
            nRGBData->r = 0;
            nRGBData->g = 0;
            nRGBData->b = 0;
        }
    }

    return true;
}

  void getGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
  {   
    if (value == SATURATION)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 128;
      imageLidar.at<Vec3b>(y, x)[1] = 0;
      imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value == ADC_OVERFLOW)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 255;
      imageLidar.at<Vec3b>(y, x)[1] = 14;
      imageLidar.at<Vec3b>(y, x)[2] = 169; 
    }
    else if (value > end_range)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 255;
      imageLidar.at<Vec3b>(y, x)[1] = 255;
      imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value < 0)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 0;
      imageLidar.at<Vec3b>(y, x)[1] = 0;
      imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else
    {
      int color = value * (255/end_range);

      //printf("color index = %d\n", color);

      imageLidar.at<Vec3b>(y, x)[0] = color;
      imageLidar.at<Vec3b>(y, x)[1] = color;
      imageLidar.at<Vec3b>(y, x)[2] = color; 
    }
}

void setParameters()
{
    interface.stopStream();    
//	interface.setUdpPort(0);
    interface.setMinAmplitude(minAmplitude);
    interface.setIntegrationTime(int0, int1, int2, intGr);

    interface.setHDRMode((uint8_t)hdr_mode);
    interface.setFilter(medianFilter, averageFilter, static_cast<uint16_t>(temporalFilterFactor * 1000), temporalFilterThreshold, edgeThreshold,
                        temporalEdgeThresholdLow, temporalEdgeThresholdHigh, interferenceDetectionLimit, useLastValue);

    interface.setAdcOverflowSaturation(bAdcOverflow, bSaturation);
    interface.setGrayscaleIlluminationMode(grayscaleIlluminationMode);

    uint8_t modIndex;
    if(frequencyModulation == 0) modIndex = 1;
    else if(frequencyModulation == 1)  modIndex = 0;
    else if(frequencyModulation == 2)  modIndex = 2;
    else    modIndex = 3;

    //maxDistance = frequencyModulation == 0 ? 6500.0f : frequencyModulation == 1 ? 12500.0f : frequencyModulation == 2 ? 25000.0f : 50000.0f;
    interface.setModulation(modIndex, channel);
    interface.setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY);
	interface.setDualBeam(dual_beam, used_dual_beam_distance);

    if(startStream){

        if(imageType == Frame::GRAYSCALE) interface.streamGrayscale();
        else if(imageType == Frame::DISTANCE) interface.streamDistance();
        else if(imageType == Frame::DISTANCE_AMPLITUDE) interface.streamDistanceAmplitude();
        else if(imageType == Frame::DCS) interface.streamDCS();
        else interface.streamDistanceGrayscale();

    }else{
        interface.stopStream();
    }

    if(old_lensCenterOffsetX != lensCenterOffsetX || old_lensCenterOffsetY != lensCenterOffsetY || old_lensType != lensType){
        cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType);
        old_lensCenterOffsetX = lensCenterOffsetX;
        old_lensCenterOffsetY = lensCenterOffsetY;
        old_lensType = lensType;
    }

    ROS_INFO("set parameters...");
    ROS_DEBUG("lens_type %d", lensType);
    ROS_DEBUG("lens_center_offset_x %d", lensCenterOffsetX);
    ROS_DEBUG("lens_center_offset_y %d", lensCenterOffsetY);
    ROS_DEBUG("image_type %d", imageType);
    ROS_DEBUG("start_stream %d", startStream);
    ROS_DEBUG("hdr_mode %d", hdr_mode);
    ROS_DEBUG("integration_time0 %d", int0);
    ROS_DEBUG("integration_time1 %d", int1);
    ROS_DEBUG("integration_time2 %d", int2);
    ROS_DEBUG("integration_time_gray %d", intGr);
    ROS_DEBUG("min_amplitude %d", minAmplitude);
    ROS_DEBUG("frequency_modulation %d", frequencyModulation);
    ROS_DEBUG("channel %d ", channel);
    ROS_DEBUG("median_filter %d ", medianFilter);
    ROS_DEBUG("average_filter %d", averageFilter);
    ROS_DEBUG("temporal_filter_factor %f", temporalFilterFactor);
    ROS_DEBUG("temporal_filter_threshold %d ", temporalFilterThreshold);
    ROS_DEBUG("edge_filter_threshold %d", edgeThreshold);
    ROS_DEBUG("interference_detection_limit %d ", interferenceDetectionLimit);
    ROS_DEBUG("use_last_value %d", useLastValue);
    ROS_DEBUG("cartesian %d", cartesian);
    ROS_DEBUG("publish_point_cloud %d", publishPointCloud);
    ROS_DEBUG("roi_left_x %d", roi_leftX);
    ROS_DEBUG("roi_right_x %d", roi_rightX);
    ROS_DEBUG("roi_height %d", roi_bottomY - roi_topY);
    ROS_DEBUG("transform_angle %d", transformAngle);
    ROS_DEBUG("cut_pixels %d", cutPixels);
    ROS_DEBUG("cv_show %d", cvShow);
    
}

void updateConfig(roboscan_nsl3130::roboscan_nsl3130Config &config, uint32_t level)
{
    startStream = config.start_stream;
    lensType = config.lens_type;
    lensCenterOffsetX = config.lens_center_offset_x;
    lensCenterOffsetY = config.lens_center_offset_y;
    imageType = config.image_type;    
    minAmplitude = config.min_amplitude;
    hdr_mode = config.hdr_mode;
    int0 = config.integration_time_tof_1;
    int1 = config.integration_time_tof_2;
    int2 = config.integration_time_tof_3;
    intGr = config.integration_time_gray; //grayscale integration time
    frequencyModulation = config.frequency_modulation;
    channel = config.channel;
    medianFilter = config.median_filter;
    averageFilter = config.average_filter;
    temporalFilterFactor = config.temporal_filter_factor;
    temporalFilterThreshold = static_cast<uint16_t>(config.temporal_filter_threshold);
    edgeThreshold = static_cast<uint16_t>(config.edge_filter_threshold);
    interferenceDetectionLimit = static_cast<uint16_t>(config.interference_detection_limit);
    useLastValue = config.use_last_value;
    cartesian = config.cartesian;
    publishPointCloud = config.publish_point_cloud;

    transformAngle = config.transform_angle;
    cutPixels = config.cut_pixels;
	maxDistance = config.max_distance;

	dual_beam = config.dual_beam;
	used_dual_beam_distance = config.dual_beam_dist;

	setWinName(config.cvShow);

    //add
    grayscaleIlluminationMode = 1;
    bAdcOverflow = 1;
    bSaturation = 1;


	if( config.roi_left_x != roi_leftX ){
		int x1_tmp = config.roi_left_x;

		if(x1_tmp % X_INTERVAL ) x1_tmp+=X_INTERVAL-(x1_tmp % X_INTERVAL );
		if(x1_tmp > LEFTX_MAX ) x1_tmp = LEFTX_MAX;

		config.roi_left_x = roi_leftX = x1_tmp;
	}
	
	if( config.roi_right_x != roi_rightX ){
		int x2_tmp = config.roi_right_x;
		
		if((x2_tmp-RIGHTX_MIN) % X_INTERVAL)	x2_tmp-=((x2_tmp-RIGHTX_MIN) % X_INTERVAL);
		if(x2_tmp < RIGHTX_MIN ) x2_tmp = RIGHTX_MIN;
		if(x2_tmp > RIGHTX_MAX ) x2_tmp = RIGHTX_MAX;

		config.roi_right_x = roi_rightX = x2_tmp;
	}

	if( config.roi_left_y != roi_topY ){
	    int y1_tmp = config.roi_left_y;

	    if(y1_tmp % Y_INTERVAL )	y1_tmp++;
	    if(y1_tmp > LEFTY_MAX ) y1_tmp = LEFTY_MAX;

		config.roi_left_y = roi_topY = y1_tmp;

		int y2_tmp = RIGHTY_MAX - y1_tmp;
		config.roi_right_y = roi_bottomY = y2_tmp;
	}

	if( config.roi_right_y != roi_bottomY ){
	    int y2_tmp = config.roi_right_y;

		if(y2_tmp % Y_INTERVAL == 0 )	y2_tmp++;
		if(y2_tmp < RIGHTY_MIN ) y2_tmp = RIGHTY_MIN;
		if(y2_tmp > RIGHTY_MAX ) y2_tmp = RIGHTY_MAX;

		config.roi_right_y = roi_bottomY = y2_tmp;

	    int y1_tmp = RIGHTY_MAX - y2_tmp;
		config.roi_left_y = roi_topY = y1_tmp;
	}
  

	//printf("x = %d y = %d width = %d height = %d\n", roi_leftX, roi_topY, roi_rightX, roi_bottomY);

    setParameters();
}


bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res)
{
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;

    cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    return true;
}

void startStreaming()
{
    ROS_INFO("startStream");
    switch(imageType) {
    case Frame::GRAYSCALE:
        interface.streamGrayscale();
        ROS_INFO("streaming grayscale");
        break;
    case Frame::DISTANCE:
        interface.streamDistance();
        ROS_INFO("streaming distance");
        break;
    case Frame::DISTANCE_AMPLITUDE:
        interface.streamDistanceAmplitude();
        ROS_INFO("streaming distance-amplitude");
        break;
    case Frame::DISTANCE_GRAYSCALE:
        interface.streamDistanceGrayscale();
        ROS_INFO("streaming distance-grayscale");
        break;
    case Frame::DISTANCE_AMPLITUDE_GRAYSCALE:
        interface.streamDistanceAmplitudeGrayscale();
        ROS_INFO("streaming distance-amplitude-grayscale");
        break;
    case Frame::DCS:
        interface.streamDCS();
        
        break;
    default:
        break;
    }
}


void updateCameraInfo(std::shared_ptr<CameraInfo> ci)
{
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
}


cv::Mat addDistanceInfo(cv::Mat distMat, std::shared_ptr<Frame> frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height)){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(xpos-10, ypos), cv::Point(xpos+10, ypos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(xpos, ypos-10), cv::Point(xpos, ypos+10), cv::Scalar(255, 255, 0), 2);

		if( xpos >= frame->width*2 ){
			xpos -= frame->width*2;
		}
		else if( xpos >= frame->width ){
			xpos -= frame->width;
		}

		std::string dist_caption;

		int real_xpos = xpos;
		int real_dist = frame->dist2BData[ypos*frame->width + real_xpos];
		if( real_dist > PIXEL_VALID_DATA ){

			if( real_dist == ADC_OVERFLOW )
				dist_caption = cv::format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( real_dist == SATURATION )
				dist_caption = cv::format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( real_dist == BAD_PIXEL )
				dist_caption = cv::format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( real_dist == INTERFERENCE )
				dist_caption = cv::format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( real_dist == EDGE_FILTERED )
				dist_caption = cv::format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist_caption = cv::format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( frame->dataType == Frame::DISTANCE_AMPLITUDE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos], frame->ampl2BData[ypos*frame->width + real_xpos]);
			else if( frame->dataType == Frame::DISTANCE_GRAYSCALE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos], frame->gray2BData[ypos*frame->width + real_xpos]);
			else if( frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb/%dlsb", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos], frame->ampl2BData[ypos*frame->width + real_xpos], frame->gray2BData[ypos*frame->width + real_xpos]);
			else if( frame->dataType == Frame::GRAYSCALE )	dist_caption = cv::format("X:%d,Y:%d %dlsb", xpos, ypos, frame->gray2BData[ypos*frame->width + real_xpos]);
			else	dist_caption = cv::format("X:%d,Y:%d %dmm", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos]);
		}

		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));
		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}

cv::Mat addDCSInfo(cv::Mat distMat, std::shared_ptr<Frame> frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height*2)){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(xpos-10, ypos), cv::Point(xpos+10, ypos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(xpos, ypos-10), cv::Point(xpos, ypos+10), cv::Scalar(255, 255, 0), 2);

		if( xpos >= frame->width ){
			xpos -= frame->width;
		}

		if( ypos >= frame->height ){
			ypos -= frame->height;
		}

		std::string dist_caption;

		int real_xpos = xpos;
		int real_dist = frame->dcs2BData[ypos*frame->width + real_xpos];
		if( real_dist > PIXEL_VALID_DATA ){

			if( real_dist == ADC_OVERFLOW )
				dist_caption = cv::format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( real_dist == SATURATION )
				dist_caption = cv::format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( real_dist == BAD_PIXEL )
				dist_caption = cv::format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( real_dist == INTERFERENCE )
				dist_caption = cv::format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( real_dist == EDGE_FILTERED )
				dist_caption = cv::format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist_caption = cv::format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			dist_caption = cv::format("X:%d,Y:%d %d/%d/%d/%d", xpos, ypos
										, frame->dcs2BData[ypos*frame->width + real_xpos]
										, frame->dcs2BData[(frame->width*frame->height) + ypos*frame->width + real_xpos]
										, frame->dcs2BData[(frame->width*frame->height * 2) + ypos*frame->width + real_xpos]
										, frame->dcs2BData[(frame->width*frame->height * 3) + ypos*frame->width + real_xpos]);
		}

		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));
		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}



void setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if( value == LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == BAD_PIXEL)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * (NUM_COLORS / end_range);
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index >= (int)colorVector.size() ){
			index = colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}


void setGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{   
	if (value == SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else
	{
		int color = value * (255/end_range);

		//printf("color index = %d\n", color);

		imageLidar.at<Vec3b>(y, x)[0] = color;
		imageLidar.at<Vec3b>(y, x)[1] = color;
		imageLidar.at<Vec3b>(y, x)[2] = color; 
	}
}




void updateFrame(std::shared_ptr<Frame> frame)
{
    int x, y, k, l, pc;
    //cv::Mat imageLidar(height, width, CV_8UC3, Scalar(255, 255, 255));
	cv::Mat dcs1(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat dcs2(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude
	cv::Mat dcs3(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// garycale
	cv::Mat dcs4(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));

//	printf("width = %d, height = %d roi_topY = %d\n", frame->width, frame->height, roi_topY);

    if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::DISTANCE_AMPLITUDE || frame->dataType == Frame::DISTANCE_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
        sensor_msgs::Image imgDistance;
        imgDistance.header.seq = frameSeq++;
        imgDistance.header.stamp = ros::Time::now();
        imgDistance.header.frame_id = "roboscan_frame";
        imgDistance.height = static_cast<uint32_t>(frame->height);
        imgDistance.width = static_cast<uint32_t>(frame->width);
        imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
        imgDistance.step = imgDistance.width * frame->px_size;
        imgDistance.is_bigendian = 0;
        imgDistance.data = frame->distData;
        distanceImagePublisher.publish(imgDistance);
    }

    if(frame->dataType == Frame::DISTANCE_AMPLITUDE || frame->dataType == Frame::DISTANCE_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
        sensor_msgs::Image imgAmpl;
        imgAmpl.header.seq = frameSeq;
        imgAmpl.header.stamp = ros::Time::now();
        imgAmpl.header.frame_id = "roboscan_frame";
        imgAmpl.height = static_cast<uint32_t>(frame->height);
        imgAmpl.width = static_cast<uint32_t>(frame->width);
        imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
        imgAmpl.step = imgAmpl.width * frame->px_size;
        imgAmpl.is_bigendian = 0;
        imgAmpl.data = frame->amplData;
        amplitudeImagePublisher.publish(imgAmpl);
    }

    if(frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
        sensor_msgs::Image imgGray;
        imgGray.header.seq = frameSeq;
        imgGray.header.stamp = ros::Time::now();
        imgGray.header.frame_id = "roboscan_frame";
        imgGray.height = static_cast<uint32_t>(frame->height);
        imgGray.width = static_cast<uint32_t>(frame->width);
        imgGray.encoding = sensor_msgs::image_encodings::MONO16;
        imgGray.step = imgGray.width * frame->px_size;
        imgGray.is_bigendian = 0;
        imgGray.data = frame->grayData;
        grayImagePublisher.publish(imgGray);
    }

    if(frame->dataType == Frame::DCS){
        sensor_msgs::Image imgDCS;
        imgDCS.header.seq = frameSeq;
        imgDCS.header.stamp = ros::Time::now();
        imgDCS.header.frame_id = "roboscan_frame";
        imgDCS.height = static_cast<uint32_t>(frame->height) * 4;
        imgDCS.width = static_cast<uint32_t>(frame->width);
        imgDCS.encoding = sensor_msgs::image_encodings::MONO16;
        imgDCS.step = imgDCS.width * frame->px_size;
        imgDCS.is_bigendian = 0;
        imgDCS.data = frame->dcsData;
        dcsImagePublisher.publish(imgDCS);
    }
    
    if(frame->dataType == Frame::GRAYSCALE){
      uint16_t gray; 
      for(k=0, l=0, y=0; y< frame->height; y++){
        for(x=0; x< frame->width; x++, k++, l+=2){
          gray = (frame->amplData[l+1] << 8)  + frame->amplData[l];
          getGrayscaleColor(dcs1, x, y, gray, 255);

        }
      }
    }

    if(publishPointCloud && frame->dataType != Frame::GRAYSCALE){

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        const size_t nPixel = frame->width * frame->height;
#ifdef USED_INTENSITY
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
#else
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
#endif
        cloud->header.frame_id = "roboscan_frame";
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(frame->width);
        cloud->height = static_cast<uint32_t>(frame->height);
        cloud->is_dense = false;
        cloud->points.resize(nPixel);

        uint16_t distance = 0;
        uint16_t amplitude = 0;
		uint16_t grayscale = 0;
        double px, py, pz;

        RGB888Pixel* pTex = new RGB888Pixel[1];


        //2d scan
        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "roboscan_frame"; //laser_frame
        //scan.angle_min = -0.959931; //110xPI/180=1.919862 -> 45xPI/180=0.785  
        //scan.angle_max = 0.959931;
        //scan.angle_increment = 1.919862 / num_readings; //3.14 / num_readings;      
        scan.angle_min = -0.942477; //90xPI/180=-1.57 -> 45xPI/180=0.785  
        scan.angle_max = 0.942477;
        scan.angle_increment = 1.884955 / num_readings; //3.14 / num_readings;      
        
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 1250.0;        
        //
        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);

        for(k=0, l=0, y=0; y< frame->height; y++){
            for(x=0, pc = frame->width-1; x< frame->width; x++, k++, l+=2, pc--){
#ifdef USED_INTENSITY
                pcl::PointXYZI &p = cloud->points[k];
#else
                pcl::PointXYZRGB &p = cloud->points[k];
#endif
                distance = (frame->distData[l+1] << 8) + frame->distData[l];
                amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];
				grayscale = (frame->grayData[l+1] << 8)  + frame->grayData[l];

                //distance 
                if(distance == LOW_AMPLITUDE || distance == INTERFERENCE || distance == EDGE_FILTERED){
                    distance = 0;              
					amplitude = 0;
                }
				else if(!(y > -x + cutPixels
	                    && y > x - (319-cutPixels)
	                    && y < x + (239-cutPixels)
	                    && y < -x + cutPixels + (239-cutPixels) + (319-cutPixels)))
                {
                    distance = 0;
					amplitude = 0;
                }

                Convert_To_RGB24((double)distance, pTex, 0.0f, maxDistance);
				dcs1.at<Vec3b>(y, x)[0] = pTex->b;
				dcs1.at<Vec3b>(y, x)[1] = pTex->g;
				dcs1.at<Vec3b>(y, x)[2] = pTex->r;

				if(frame->dataType == Frame::DISTANCE_AMPLITUDE){
					setAmplitudeColor(dcs2, x, y, amplitude, 2897);
				}
				else if( frame->dataType == Frame::DISTANCE_GRAYSCALE ){
					setGrayscaleColor(dcs3, x, y, grayscale, 2048);
				}
				else if( frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
					setAmplitudeColor(dcs2, x, y, amplitude, 2897);
					setGrayscaleColor(dcs3, x, y, grayscale, 2048);
				}

                if (distance > 0 && distance < maxDistance )
                {
                    if(cartesian){
                        cartesianTransform.transformPixel(pc, y+roi_topY, distance, px, py, pz, transformAngle);
                        p.x = static_cast<float>(pz / 1000.0); //mm -> m
                        p.y = static_cast<float>(px / 1000.0);
                        p.z = static_cast<float>(-py / 1000.0);

#ifdef USED_INTENSITY
                        if(frame->dataType == Frame::DISTANCE_AMPLITUDE) p.intensity = static_cast<float>(amplitude);
                        else p.intensity = static_cast<float>(pz / 1000.0);
#else        
                        p.r = dcs2.at<Vec3b>(y, x)[0];
                        p.g = dcs2.at<Vec3b>(y, x)[1];
                        p.b = dcs2.at<Vec3b>(y, x)[2];
#endif                        
                        
                    }else{
                        p.x = distance / 1000.0;
                        p.y = -(160-pc) / 100.0;
                        p.z = (120-y) / 100.0;
#ifdef USED_INTENSITY
                        if(frame->dataType == Frame::DISTANCE_AMPLITUDE) p.intensity =  static_cast<float>(amplitude);
                        else p.intensity = static_cast<float>(distance / 1000.0);
#endif
                    }

                    if(y == 120)
                    {
                        //ranges[i] = count;
                        scan.ranges[frame->width-x-1] = (double)distance*0.001;
                        scan.intensities[frame->width-x-1] = amplitude;
                    }
                
                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

		if( frame->dataType == Frame::DISTANCE ){
			dcs1 = addDistanceInfo(dcs1, frame);
		}
		else if( frame->dataType == Frame::DISTANCE_AMPLITUDE ){
			cv::hconcat(dcs1, dcs2, dcs1);
			dcs1 = addDistanceInfo(dcs1, frame);
		}
		else if( frame->dataType == Frame::DISTANCE_GRAYSCALE ){
			cv::hconcat(dcs1, dcs3, dcs1);
			dcs1 = addDistanceInfo(dcs1, frame);
		}
		else if( frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
			cv::hconcat(dcs1, dcs2, dcs1);
			cv::hconcat(dcs1, dcs3, dcs1);
			dcs1 = addDistanceInfo(dcs1, frame);
		}
		else if( frame->dataType == Frame::GRAYSCALE ){
			dcs1 = addDistanceInfo(dcs3, frame);
		}
		else if(frame->dataType == Frame::DCS){
			cv::hconcat(dcs1, dcs2, dcs1);
			cv::hconcat(dcs3, dcs4, dcs3);
			cv::vconcat(dcs1, dcs3, dcs1);
			dcs1 = addDCSInfo(dcs1, frame);
		}
		
		if(cvShow == true)
		{
			imshow(winName, dcs1);
			waitKey(1);
		}
        
        pointCloud2Publisher.publish(cloud);
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "roboscan_frame";
        cv_ptr->image = dcs1;
        cv_ptr->encoding = "bgr8";

        imagePublisher.publish(cv_ptr->toImageMsg());

        scanPub.publish(scan);

        delete[] pTex;
    }

}


//===================================================

void initialise()
{
    frameSeq = 0;
    ros::NodeHandle nh("~");

    nh.getParam("lens_Type", lensType);
    nh.getParam("lens_center_offset_x", lensCenterOffsetX);
    nh.getParam("lens_center_offset_y", lensCenterOffsetY);
    nh.getParam("start_stream", startStream);
    nh.getParam("image_type", imageType);
    nh.getParam("hdr_mode", hdr_mode);
    nh.getParam("int0", int0);
    nh.getParam("int1", int1);
    nh.getParam("int2", int2);
    nh.getParam("int_gray", intGr);
    nh.getParam("frequency_modulation", frequencyModulation);
    nh.getParam("channel", channel);   
    nh.getParam("min_amplitude", minAmplitude);
    nh.getParam("median_filter", medianFilter);
    nh.getParam("average_filter", averageFilter);
    nh.getParam("temporal_filter_factor", temporalFilterFactor);
    nh.getParam("temporal_filter_threshold", temporalFilterThreshold);
    nh.getParam("edge_threshold", edgeThreshold);
    nh.getParam("temporal_edge_threshold_low", temporalEdgeThresholdLow);
    nh.getParam("temporal_edge_threshold_high", temporalEdgeThresholdHigh);
    nh.getParam("interference_detection_limit", interferenceDetectionLimit);
    nh.getParam("use_last_value", useLastValue);
    nh.getParam("cartesian", cartesian);
    nh.getParam("publish_point_cloud", publishPointCloud);	
    nh.getParam("transform_angle", transformAngle);
    nh.getParam("cut_pixels", cutPixels);
    nh.getParam("cv_show", cvShow);


    //advertise publishers
    distanceImagePublisher = nh.advertise<sensor_msgs::Image>("distance_image_raw", 1000);
    amplitudeImagePublisher = nh.advertise<sensor_msgs::Image>("amplitude_image_raw", 1000);
    dcsImagePublisher = nh.advertise<sensor_msgs::Image>("dcs_image_raw", 1000);

#ifdef USED_INTENSITY
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 100);
#else
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("points", 100);
#endif
    
    cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
    grayImagePublisher = nh.advertise<sensor_msgs::Image>("gray_image_raw", 1000);
    scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);


    //advertise image Publisher
    image_transport::ImageTransport it_(nh);
    imagePublisher = it_.advertise("image_distance", 1000);

    //advertise services
    cameraInfoService = nh.advertiseService("set_camera_info", setCameraInfo);

    //connect to interface
    connectionCameraInfo = interface.subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
    connectionFrames = interface.subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });

    cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType); //0.02 mm - sensor pixel size
    old_lensCenterOffsetX = lensCenterOffsetX;
    old_lensCenterOffsetY = lensCenterOffsetY;
    old_lensType = lensType;

	int numSteps = NUM_COLORS;
	unsigned char red, green, blue;

	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(Vec3b(blue, green, red));
	}

    ROS_INFO("roboscan_nsl3130 node");
}


//==========================================================================

int main(int argc, char **argv)
{   
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "roboscan_publish_node");
    dynamic_reconfigure::Server<roboscan_nsl3130::roboscan_nsl3130Config> server;
    dynamic_reconfigure::Server<roboscan_nsl3130::roboscan_nsl3130Config>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialise();
    setParameters();
    startStreaming();

    ros::spin();
}
