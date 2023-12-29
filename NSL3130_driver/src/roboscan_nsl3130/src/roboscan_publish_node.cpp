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


#define LOW_AMPLITUDE       64001
#define ADC_OVERFLOW        64002
#define SATURATION          64003
#define BAD_PIXEL           64004
#define INTERFERENCE        64007
#define EDGE_FILTERED       64008


using namespace nanosys;
using namespace cv;

int imageType = 2; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int lensType = 1;  //0- wide field, 1- standard field, 2 - narrow field
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

//=======================================================================

typedef struct _RGB888Pixel
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RGB888Pixel;


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

    maxDistance = frequencyModulation == 0 ? 6500.0f : frequencyModulation == 1 ? 12500.0f : frequencyModulation == 2 ? 25000.0f : 50000.0f;
    interface.setModulation(modIndex, channel);

    interface.setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY);

    if(startStream){

        if(imageType == Frame::GRAYSCALE) interface.streamGrayscale();
        else if(imageType == Frame::DISTANCE) interface.streamDistance();
        else if(imageType == Frame::AMPLITUDE) interface.streamDistanceAmplitude();
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
    

    //add
    grayscaleIlluminationMode = 1;
    bAdcOverflow = 1;
    bSaturation = 1;

    

    roi_leftX   = config.roi_left_x;
    roi_rightX  = config.roi_right_x;

    if(roi_rightX - roi_leftX < 7)
        roi_rightX = roi_leftX + 7;

    roi_rightX -= (roi_rightX - roi_leftX + 1) % 4;
    config.roi_right_x = roi_rightX;

    config.roi_height -= config.roi_height % 4;
    if(config.roi_height < 8) config.roi_height = 8;

    roi_topY    = 120 - config.roi_height/2;
    roi_bottomY = 119 + config.roi_height/2;


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
    switch(imageType) {
    case Frame::GRAYSCALE:
        interface.streamGrayscale();
        ROS_INFO("streaming grayscale");
        break;
    case Frame::DISTANCE:
        interface.streamDistance();
        ROS_INFO("streaming distance");
        break;
    case Frame::AMPLITUDE:
        interface.streamDistanceAmplitude();
        ROS_INFO("streaming distance-amplitude");
        break;
    case Frame::DISTANCE_AND_GRAYSCALE:
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

void updateFrame(std::shared_ptr<Frame> frame)
{
    int x, y, k, l, pc;
    cv::Mat imageLidar(height, width, CV_8UC3, Scalar(255, 255, 255));

    if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::DISTANCE_AND_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
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

    if(frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::DISTANCE_AND_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
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
          getGrayscaleColor(imageLidar, x, y, gray, 255);

        }
      }
    }

    if(publishPointCloud && frame->dataType != Frame::GRAYSCALE){

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        const size_t nPixel = frame->width * frame->height;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = "roboscan_frame";
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(frame->width);
        cloud->height = static_cast<uint32_t>(frame->height);
        cloud->is_dense = false;
        cloud->points.resize(nPixel);

        uint16_t distance = 0;
        uint16_t amplitude = 0;
        double px, py, pz;

        RGB888Pixel* pTex = new RGB888Pixel[1];
        


        for(k=0, l=0, y=0; y< frame->height; y++){
            for(x=0, pc = frame->width-1; x< frame->width; x++, k++, l+=2, pc--){
                pcl::PointXYZI &p = cloud->points[k];
                distance = (frame->distData[l+1] << 8) + frame->distData[l];

                if(frame->dataType == Frame::AMPLITUDE)
                    amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];

                //distance 
                if(distance == LOW_AMPLITUDE || distance == INTERFERENCE || distance == EDGE_FILTERED)
                    distance = 0;

                Convert_To_RGB24((double)distance, pTex, 0.0f, 12500.0f);
                
                if(y > -x + cutPixels
                    && y > x - (319-cutPixels)
                    && y < x + (239-cutPixels)
                    && y < -x + cutPixels + (239-cutPixels) + (319-cutPixels))
                {
                    imageLidar.at<Vec3b>(y, x)[0] = pTex->b;
                    imageLidar.at<Vec3b>(y, x)[1] = pTex->g;
                    imageLidar.at<Vec3b>(y, x)[2] = pTex->r;
                }
                else
                {
                    imageLidar.at<Vec3b>(y, x)[0] = 0;
                    imageLidar.at<Vec3b>(y, x)[1] = 0;
                    imageLidar.at<Vec3b>(y, x)[2] = 0;
                }

                if (distance > 0 && distance < maxDistance
                    && y > -x + cutPixels
                    && y > x - (319-cutPixels)
                    && y < x + (239-cutPixels)
                    && y < -x + cutPixels + (239-cutPixels) + (319-cutPixels))
                {
                    if(cartesian){
                        cartesianTransform.transformPixel(pc, y, distance, px, py, pz, transformAngle);
                        p.x = static_cast<float>(pz / 1000.0); //mm -> m
                        p.y = static_cast<float>(px / 1000.0);
                        p.z = static_cast<float>(-py / 1000.0);

                        if(frame->dataType == Frame::AMPLITUDE) p.intensity = static_cast<float>(amplitude);
                        else p.intensity = static_cast<float>(pz / 1000.0);

                    }else{
                        p.x = distance / 1000.0;
                        p.y = -(160-pc) / 100.0;
                        p.z = (120-y) / 100.0;
                        if(frame->dataType == Frame::AMPLITUDE) p.intensity =  static_cast<float>(amplitude);
                        else p.intensity = static_cast<float>(distance / 1000.0);
                    }
                
                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        
        pointCloud2Publisher.publish(cloud);
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "roboscan_frame";
        cv_ptr->image = imageLidar;
        cv_ptr->encoding = "bgr8";

        imagePublisher.publish(cv_ptr->toImageMsg());

        
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
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 100);
    cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
    grayImagePublisher = nh.advertise<sensor_msgs::Image>("gray_image_raw", 1000);

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
