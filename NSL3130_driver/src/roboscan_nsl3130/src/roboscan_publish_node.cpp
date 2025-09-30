#include <cstdio>
#include <functional>
#include <experimental/filesystem>
#include <memory>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include <ros/package.h> 
#include "roboscan_publish_node.hpp"
#include <fstream>    
#include <stdexcept> 
using namespace NslOption;
using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

#define WIN_NAME "NSL-3130AA IMAGE"
#define MAX_LEVELS  9
#define NUM_COLORS     		30000

#define LEFTX_MAX	124	
#define RIGHTX_MIN	131
#define RIGHTX_MAX	319	
#define X_INTERVAL	4

#define LEFTY_MAX	116	
#define RIGHTY_MIN	123
#define RIGHTY_MAX	239	
#define Y_INTERVAL	2

#define DISTANCE_INFO_HEIGHT	80

std::atomic<int> x_start = -1, y_start = -1;
std::unique_ptr<NslPCD> latestFrame = std::make_unique<NslPCD>();


static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}
roboscanPublisher::roboscanPublisher() : nh_("~")
{
    ROS_INFO("start roboscanPublisher...\n");
    ros::NodeHandle public_nh;

    it_.reset(new image_transport::ImageTransport(public_nh));
    imgDistancePub = public_nh.advertise<sensor_msgs::Image>("roboscanDistance", 10);
    imgAmplPub     = public_nh.advertise<sensor_msgs::Image>("roboscanAmpl", 10);
    imgGrayPub     = public_nh.advertise<sensor_msgs::Image>("roboscanGray", 10);
    pointcloudPub  = public_nh.advertise<sensor_msgs::PointCloud2>("roboscanPointCloud", 10);
    std::string pkg = ros::package::getPath("roboscan_nsl3130");
    yaml_path_ = pkg + "/lidar_params.yaml";
	dr_server_.reset(new dynamic_reconfigure::Server<roboscan_nsl3130::RoboscanNSL3130Config>(param_mutex_, nh_));
	dr_server_->setCallback(boost::bind(&roboscanPublisher::dynamicReconfigureCallback, this, _1, _2));

	reconfigure = false;
	mouseXpos = -1;
	mouseYpos = -1;

    runThread = true;
    publisherThread.reset(new std::thread(&roboscanPublisher::threadCallback, this));

    ROS_INFO("\nRun rqt to view the image!\n");
}



roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();

	nsl_close();

    ROS_INFO("\nEnd roboscanPublisher()!\n");
}


void roboscanPublisher::threadCallback()
{
	auto lastTime = chrono::steady_clock::now();
	int frameCount = 0;

	while(ros::ok() && runThread){

        if (reconfigure) {
            reconfigure = false;
            setReconfigure();
        }   

		if( nsl_getPointCloudData(nsl_handle, latestFrame.get()) == NSL_ERROR_TYPE::NSL_SUCCESS )
		{
			frameCount++;			
			publishFrame(latestFrame.get());
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		auto now = chrono::steady_clock::now();
		auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
		if( elapsed >= 1000 ){
			viewerParam.frameCount = frameCount;
			frameCount = 0;
			lastTime = now;
//			ROS_INFO("frame = %d fps\n", viewerParam.frameCount);
		}
		
	}

	cv::destroyAllWindows();
	ROS_INFO("end threadCallback\n");
}

void roboscanPublisher::dynamicReconfigureCallback(roboscan_nsl3130::RoboscanNSL3130Config &config, uint32_t level)
{
    boost::recursive_mutex::scoped_lock lock(param_mutex_);
    (void)level;
    ROS_INFO("Reconfigure Request Received.");

    if (viewerParam.cvShow != config.cv_show) {
        viewerParam.cvShow = config.cv_show;
        viewerParam.changedCvShow = true; 
    }
    if (viewerParam.ipAddr != config.ip_addr) {
        ROS_INFO("IP address changed from %s to %s", viewerParam.ipAddr.c_str(), config.ip_addr.c_str());
        viewerParam.ipAddr     = config.ip_addr;
        viewerParam.reOpenLidar = true;
        viewerParam.saveParam   = true; 
    }
	/*
    if (viewerParam.netMask != config.net_mask) {
        ROS_INFO("Netmask changed %s -> %s", viewerParam.netMask.c_str(), config.net_mask.c_str());
        viewerParam.netMask    = config.net_mask;
        viewerParam.reOpenLidar = true;
        viewerParam.saveParam   = true;
    }
    if (viewerParam.gwAddr != config.gw_addr) {
        ROS_INFO("Gateway changed %s -> %s", viewerParam.gwAddr.c_str(), config.gw_addr.c_str());
        viewerParam.gwAddr     = config.gw_addr;
        viewerParam.reOpenLidar = true;
        viewerParam.saveParam   = true;
    }
	*/
    if (viewerParam.imageType != config.image_type) {
		if( config.image_type >= 1 && config.image_type <= 8 ){
			viewerParam.imageType = config.image_type;
			viewerParam.changedImageType = true;
			viewerParam.saveParam = true;
		}
    }
    if (viewerParam.lensType != config.lens_type) {
		if( config.lens_type >= 0 && config.lens_type <= 2 ){
			viewerParam.lensType = config.lens_type;
			viewerParam.reOpenLidar = true;
			viewerParam.saveParam	= true; 
		}
    }
    if (viewerParam.lidarAngle != config.transform_angle) {
        viewerParam.lidarAngle = config.transform_angle;
		viewerParam.reOpenLidar = true;
        viewerParam.saveParam  = true; 
    }
    if (viewerParam.frame_id != config.frame_id) {
        ROS_INFO("changed to frame ID %s -> %s", viewerParam.frame_id.c_str(), config.frame_id.c_str());
		if (config.frame_id.empty())
	        viewerParam.frame_id = "roboscan_frame";
		else
	        viewerParam.frame_id = config.frame_id;
        viewerParam.saveParam = true; 
    }
    if (viewerParam.maxDistance != config.max_distance) {
        viewerParam.maxDistance = config.max_distance;
        viewerParam.saveParam   = true; 
    }
    if (viewerParam.pointCloudEdgeThreshold != config.pointcloud_edge) {
        viewerParam.pointCloudEdgeThreshold = config.pointcloud_edge;
        viewerParam.saveParam               = true; 
    }
    if (config.hdr_mode >= 0 && config.hdr_mode <= 2) {
        nslConfig.hdrOpt = static_cast<NslOption::HDR_OPTIONS>(config.hdr_mode);
    }

    nslConfig.integrationTime3D        = config.int_0;
    nslConfig.integrationTime3DHdr1    = config.int_1;
    nslConfig.integrationTime3DHdr2    = config.int_2;
    nslConfig.integrationTimeGrayScale = config.int_gr;
    nslConfig.minAmplitude             = config.min_amplitude;

    if (config.mod_index >= 0 && config.mod_index <= 3) {
        nslConfig.mod_frequencyOpt = static_cast<NslOption::MODULATION_OPTIONS>(config.mod_index);
    }
    if (config.channel >= 0 && config.channel <= 15) {
        nslConfig.mod_channelOpt = static_cast<NslOption::MODULATION_CH_OPTIONS>(config.channel);
    }
    if (nslConfig.roiXMin != config.roi_left_x) {
        int x1_tmp = config.roi_left_x;
        if (x1_tmp % X_INTERVAL) x1_tmp += X_INTERVAL - (x1_tmp % X_INTERVAL);
        if (x1_tmp > LEFTX_MAX)   x1_tmp = LEFTX_MAX;
        if (config.roi_left_x != x1_tmp) {
            ROS_DEBUG("Correcting roi_left_x from %d to %d", config.roi_left_x, x1_tmp);
            config.roi_left_x = x1_tmp;         
        }
        nslConfig.roiXMin = x1_tmp;
    }
    if (nslConfig.roiXMax != config.roi_right_x) {
        int x2_tmp = config.roi_right_x;
        if ((x2_tmp - RIGHTX_MIN) % X_INTERVAL) x2_tmp -= ((x2_tmp - RIGHTX_MIN) % X_INTERVAL);
        if (x2_tmp < RIGHTX_MIN) x2_tmp = RIGHTX_MIN;
        if (x2_tmp > RIGHTX_MAX) x2_tmp = RIGHTX_MAX;
        if (config.roi_right_x != x2_tmp) {
            ROS_DEBUG("Correcting roi_right_x from %d to %d", config.roi_right_x, x2_tmp);
            config.roi_right_x = x2_tmp;        
        }
        nslConfig.roiXMax = x2_tmp;
    }
    if (nslConfig.roiYMin != config.roi_top_y) {
        int y1_tmp = config.roi_top_y;
        if (y1_tmp % Y_INTERVAL) y1_tmp++;
        if (y1_tmp > LEFTY_MAX)  y1_tmp = LEFTY_MAX;
        if (config.roi_top_y != y1_tmp) {
            ROS_DEBUG("Correcting roi_top_y from %d to %d", config.roi_top_y, y1_tmp);
            config.roi_top_y = y1_tmp;        
        }
        nslConfig.roiYMin = y1_tmp;
        nslConfig.roiYMax = RIGHTY_MAX;          
    }

    nslConfig.medianOpt                        = static_cast<NslOption::FUNCTION_OPTIONS>(config.median_filter);
    nslConfig.gaussOpt                         = static_cast<NslOption::FUNCTION_OPTIONS>(config.gaussian_filter);
    nslConfig.temporalFactorValue              = static_cast<int>(config.temporal_filter_factor * 1000);
    nslConfig.temporalThresholdValue           = config.temporal_filter_threshold;
    nslConfig.edgeThresholdValue               = config.edge_filter_threshold;
    nslConfig.interferenceDetectionLimitValue  = config.interference_detection_limit;
    nslConfig.interferenceDetectionLastValueOpt= static_cast<NslOption::FUNCTION_OPTIONS>(config.use_last_value);
    nslConfig.dbModOpt                         = static_cast<NslOption::DUALBEAM_MOD_OPTIONS>(config.dual_beam);
    nslConfig.dbOpsOpt                         = static_cast<NslOption::DUALBEAM_OPERATION_OPTIONS>(config.dual_beam_option);
    nslConfig.grayscaleIlluminationOpt         = static_cast<NslOption::FUNCTION_OPTIONS>(config.grayscale_led);

    reconfigure = true;
}




void roboscanPublisher::timeDelay(int milli)
{
	auto start = std::chrono::steady_clock::now();
	while ( runThread != 0 ) {
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= milli) {
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void roboscanPublisher::renewParameter()
{
	
    roboscan_nsl3130::RoboscanNSL3130Config cfg_for_gui;
	
    cfg_for_gui.ip_addr = viewerParam.ipAddr;
    cfg_for_gui.cv_show = viewerParam.cvShow;
    cfg_for_gui.frame_id = viewerParam.frame_id;
    cfg_for_gui.lens_type = viewerParam.lensType;
    cfg_for_gui.image_type = viewerParam.imageType;
    cfg_for_gui.transform_angle = viewerParam.lidarAngle;
    cfg_for_gui.pointcloud_edge = viewerParam.pointCloudEdgeThreshold;
    cfg_for_gui.max_distance = viewerParam.maxDistance;

    cfg_for_gui.hdr_mode = static_cast<int>(nslConfig.hdrOpt);
    cfg_for_gui.int_0 = nslConfig.integrationTime3D;
    cfg_for_gui.int_1 = nslConfig.integrationTime3DHdr1;
    cfg_for_gui.int_2 = nslConfig.integrationTime3DHdr2;
    cfg_for_gui.int_gr = nslConfig.integrationTimeGrayScale;
    cfg_for_gui.min_amplitude = nslConfig.minAmplitude;
    cfg_for_gui.mod_index = static_cast<int>(nslConfig.mod_frequencyOpt);
    cfg_for_gui.channel = static_cast<int>(nslConfig.mod_channelOpt);
    cfg_for_gui.roi_left_x = nslConfig.roiXMin;
    cfg_for_gui.roi_top_y = nslConfig.roiYMin;
    cfg_for_gui.roi_right_x = nslConfig.roiXMax;
    cfg_for_gui.median_filter = static_cast<bool>(nslConfig.medianOpt);
    cfg_for_gui.gaussian_filter = static_cast<bool>(nslConfig.gaussOpt);
    cfg_for_gui.temporal_filter_factor = nslConfig.temporalFactorValue / 1000.0;
    cfg_for_gui.temporal_filter_threshold = nslConfig.temporalThresholdValue;
    cfg_for_gui.edge_filter_threshold = nslConfig.edgeThresholdValue;
    cfg_for_gui.interference_detection_limit = nslConfig.interferenceDetectionLimitValue;
    cfg_for_gui.use_last_value = static_cast<bool>(nslConfig.interferenceDetectionLastValueOpt);
    cfg_for_gui.dual_beam = static_cast<int>(nslConfig.dbModOpt);
    cfg_for_gui.dual_beam_option = static_cast<int>(nslConfig.dbOpsOpt);
    cfg_for_gui.grayscale_led = static_cast<bool>(nslConfig.grayscaleIlluminationOpt);
	
    if (dr_server_) { 
        dr_server_->updateConfig(cfg_for_gui);
    }
	
	
    nh_.setParam("ip_addr",               viewerParam.ipAddr);
    nh_.setParam("cv_show",               viewerParam.cvShow);
    nh_.setParam("frame_id",              viewerParam.frame_id);

    nh_.setParam("lens_type",             viewerParam.lensType);
    nh_.setParam("image_type",            viewerParam.imageType);
    nh_.setParam("hdr_mode",              static_cast<int>(nslConfig.hdrOpt));

    nh_.setParam("int_0",                 nslConfig.integrationTime3D);
    nh_.setParam("int_1",                 nslConfig.integrationTime3DHdr1);
    nh_.setParam("int_2",                 nslConfig.integrationTime3DHdr2);
    nh_.setParam("int_gr",                nslConfig.integrationTimeGrayScale);

    nh_.setParam("min_amplitude",         nslConfig.minAmplitude);
    nh_.setParam("mod_index",             static_cast<int>(nslConfig.mod_frequencyOpt));
    nh_.setParam("channel",               static_cast<int>(nslConfig.mod_channelOpt));

    nh_.setParam("roi_left_x",            nslConfig.roiXMin);
    nh_.setParam("roi_top_y",             nslConfig.roiYMin);
    nh_.setParam("roi_right_x",           nslConfig.roiXMax);

    nh_.setParam("transform_angle",       viewerParam.lidarAngle);

    nh_.setParam("median_filter",         static_cast<bool>(nslConfig.medianOpt));
    nh_.setParam("gaussian_filter",       static_cast<bool>(nslConfig.gaussOpt));
    nh_.setParam("temporal_filter_factor",nslConfig.temporalFactorValue / 1000.0);
    nh_.setParam("temporal_filter_threshold", nslConfig.temporalThresholdValue);
    nh_.setParam("edge_filter_threshold", nslConfig.edgeThresholdValue);

    nh_.setParam("interference_detection_limit",  nslConfig.interferenceDetectionLimitValue);
    nh_.setParam("use_last_value",        static_cast<bool>(nslConfig.interferenceDetectionLastValueOpt));

    nh_.setParam("dual_beam",             static_cast<int>(nslConfig.dbModOpt));
    nh_.setParam("dual_beam_option",      static_cast<int>(nslConfig.dbOpsOpt));

    nh_.setParam("grayscale_led",         static_cast<bool>(nslConfig.grayscaleIlluminationOpt));
    nh_.setParam("pointcloud_edge",       viewerParam.pointCloudEdgeThreshold);
    nh_.setParam("max_distance",          viewerParam.maxDistance);
	

    ROS_INFO("[renewParameter]!!!");
}


void roboscanPublisher::setReconfigure()
{	
	if( viewerParam.saveParam )
	{
		viewerParam.saveParam = false;
		save_params();
	}

	if( !viewerParam.changedCvShow )
	{
		nsl_streamingOff(nsl_handle);
		
		std::cout << " nsl_handle = "<< nsl_handle << "nsl_open :: reOpenLidar = "<< viewerParam.reOpenLidar << std::endl;
		
		if( nsl_handle < 0 && viewerParam.reOpenLidar ){

			nslConfig.lidarAngle = viewerParam.lidarAngle;
			nslConfig.lensType = static_cast<NslOption::LENS_TYPE>(viewerParam.lensType);
			nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
			viewerParam.reOpenLidar = false;

			if( nsl_handle >= 0 ){
				renewParameter();
			}
		}
		
		if( nsl_handle >= 0 ){
			nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
			nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
			nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
			nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
			nsl_set3DFilter(nsl_handle, viewerParam.pointCloudEdgeThreshold);
			nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
			nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
			nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
			nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
			nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);
			
			nsl_saveConfiguration(nsl_handle);
			
			startStreaming();
		}
	}

	setWinName();
	std::cout << "end setReconfigure"<< std::endl;

}

void roboscanPublisher::setWinName()
{
	bool changedCvShow = viewerParam.changedCvShow || viewerParam.changedImageType;
	viewerParam.changedCvShow = false;
	viewerParam.changedImageType = false;
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( viewerParam.cvShow == false || changedCvShow == false ) return;
	
	if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_MODE){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE){
		sprintf(winName,"%s(Gray)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE){
		sprintf(winName,"%s(Dist/Gray)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_MODE){
		sprintf(winName,"%s(RGB)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE){
		sprintf(winName,"%s(RGB/Dist)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE){
		sprintf(winName,"%s(RGB/Dist/Ampl)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE){
		sprintf(winName,"%s(RGB/Dist/Gray)", WIN_NAME);
	}
	else{
		sprintf(winName,"%s(READY)", WIN_NAME);
	}
	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	//cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}

void roboscanPublisher::initNslLibrary()
{
	nslConfig.lidarAngle = viewerParam.lidarAngle;
	nslConfig.lensType = static_cast<NslOption::LENS_TYPE>(viewerParam.lensType);
	nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
	if( nsl_handle < 0 ){
		std::cout << "nsl_open::handle open error::" << nsl_handle << std::endl;
		return;
	}

	nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
	nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
	nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
	nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
	nsl_set3DFilter(nsl_handle, viewerParam.pointCloudEdgeThreshold);
	nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
	nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
	nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, nslConfig.mod_enabledAutoChannelOpt);
	nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
	nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);

	startStreaming();
}


void roboscanPublisher::initialise()
{
	std::cout<<"Init roboscan_nsl3130 node\n"<<std::endl;

	roboscan_nsl3130::RoboscanNSL3130Config cfg = roboscan_nsl3130::RoboscanNSL3130Config::__getDefault__();

	viewerParam.ipAddr = cfg.ip_addr;
	viewerParam.frame_id = cfg.frame_id;
	viewerParam.maxDistance = cfg.max_distance;
	viewerParam.pointCloudEdgeThreshold = cfg.pointcloud_edge;
	viewerParam.imageType = cfg.image_type;
	viewerParam.lensType = cfg.lens_type;
	viewerParam.lidarAngle = cfg.transform_angle;
	viewerParam.cvShow = cfg.cv_show;
	viewerParam.saveParam = false;
	reconfigure = false;

	load_params(cfg);
	initNslLibrary();

	ROS_INFO("Attempting to connect to device at IP: %s", viewerParam.ipAddr.c_str());
	if (nsl_handle >= 0)
	{
		ROS_INFO("Successfully connected. Reading settings from the device.");
		cfg.hdr_mode = static_cast<int>(nslConfig.hdrOpt);
		cfg.int_0 = nslConfig.integrationTime3D;
		cfg.int_1 = nslConfig.integrationTime3DHdr1;
		cfg.int_2 = nslConfig.integrationTime3DHdr2;
		cfg.int_gr = nslConfig.integrationTimeGrayScale;
		cfg.min_amplitude = nslConfig.minAmplitude;
		cfg.mod_index = static_cast<int>(nslConfig.mod_frequencyOpt);
		cfg.channel = static_cast<int>(nslConfig.mod_channelOpt);
		cfg.roi_left_x = nslConfig.roiXMin;
		cfg.roi_top_y = nslConfig.roiYMin;
		cfg.roi_right_x = nslConfig.roiXMax;
		cfg.median_filter = (nslConfig.medianOpt == NslOption::FUNCTION_OPTIONS::FUNC_ON);
		cfg.gaussian_filter = (nslConfig.gaussOpt == NslOption::FUNCTION_OPTIONS::FUNC_ON);
		cfg.temporal_filter_factor = std::max(0, std::min(1000, nslConfig.temporalFactorValue)) / 1000.0;
		cfg.temporal_filter_threshold = nslConfig.temporalThresholdValue;
		cfg.edge_filter_threshold = nslConfig.edgeThresholdValue;
		cfg.interference_detection_limit = nslConfig.interferenceDetectionLimitValue;
		cfg.use_last_value = (nslConfig.interferenceDetectionLastValueOpt == NslOption::FUNCTION_OPTIONS::FUNC_ON);
		cfg.dual_beam = static_cast<int>(nslConfig.dbModOpt);
		cfg.dual_beam_option = static_cast<int>(nslConfig.dbOpsOpt);
		cfg.grayscale_led = (nslConfig.grayscaleIlluminationOpt == NslOption::FUNCTION_OPTIONS::FUNC_ON);

		startStreaming();
	}
	else
	{
		ROS_WARN("Failed to connect to the device. Falling back to settings from lidar_params.yaml");
	}   

	setWinName();


	if (dr_server_) { 
		dr_server_->updateConfig(cfg);
	}

	ROS_INFO("end initialise()\n");
}

void roboscanPublisher::startStreaming()
{	
	if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::GRAYSCALE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::GRAYSCALE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE);
	}
	else{
		std::cout << "operation mode NONE~~~"<< std::endl;
	}
}


cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, NslPCD *frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height)){

		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255)); 

		line(distMat, Point(xpos-10, ypos), Point(xpos+10, ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(xpos, ypos-10), Point(xpos, ypos+10), Scalar(255, 255, 0), 2);
		if( xpos >= frame->width ){ 
			xpos -= frame->width;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

        int distance2D = frame->distance2D[ypos][xpos]; 
		if( distance2D > NSL_LIMIT_FOR_VALID_DATA ){
			if( distance2D == NSL_ADC_OVERFLOW )
				dist2D_caption = format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( distance2D == NSL_SATURATION )
				dist2D_caption = format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( distance2D == NSL_BAD_PIXEL )
				dist2D_caption = format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( distance2D == NSL_INTERFERENCE )
				dist2D_caption = format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( distance2D == NSL_EDGE_DETECTED )
				dist2D_caption = format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist2D_caption = format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE || frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ) {
				dist2D_caption = format("2D X:%d Y:%d %dmm/%dlsb", xpos, ypos, frame->distance2D[ypos][xpos], frame->amplitude[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);

            }
			else{
				dist2D_caption = format("2D X:%d Y:%d <%d>mm", xpos, ypos, frame->distance2D[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
		}
		
		info_caption = format("%s:%dx%d %.2f'C, %d fps", toString(frame->operationMode), frame->width, frame->height, frame->temperature, viewerParam.frameCount);

		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist2D_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, distMat);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d %.2f'C, %d fps", toString(frame->operationMode), frame->width, frame->height, frame->temperature, viewerParam.frameCount);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}

void roboscanPublisher::setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color.b;
	image.at<Vec3b>(y,x)[1] = color.g;
	image.at<Vec3b>(y,x)[2] = color.r;
}

void roboscanPublisher::publishFrame(NslPCD *frame)
{
    ros::Time data_stamp = ros::Time::now();

	cv::Mat distanceMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat amplitudeMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude
#ifdef image_transfer_function
	cv::Mat rgbMat(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3, Scalar(255, 255, 255));
#endif


	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE )
	{
		sensor_msgs::Image imgDistance;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->distance2D[y+yMin][x+xMin] & 0xFF));		 // LSB
				result.push_back(static_cast<uint8_t>((frame->distance2D[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(distanceMat, x+xMin, y+yMin, nsl_getDistanceColor(frame->distance2D[y+yMin][x+xMin]));
			}
		}

		imgDistance.header.stamp = data_stamp;
		imgDistance.header.frame_id = viewerParam.frame_id;
		imgDistance.height = static_cast<uint32_t>(frame->height);
		imgDistance.width = static_cast<uint32_t>(frame->width);
		imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
		imgDistance.step = imgDistance.width * 2;
		imgDistance.is_bigendian = 0;
		imgDistance.data = result;
		imgDistancePub.publish(imgDistance);
	}

	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)
	{
		sensor_msgs::Image imgAmpl;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;

		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgAmpl.header.stamp = data_stamp;
		imgAmpl.header.frame_id = viewerParam.frame_id;
		imgAmpl.height = static_cast<uint32_t>(frame->height);
		imgAmpl.width = static_cast<uint32_t>(frame->width);
		imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
		imgAmpl.step = imgAmpl.width * 2;
		imgAmpl.is_bigendian = 0;
		imgAmpl.data = result;
		imgAmplPub.publish(imgAmpl);
	}	

	
	if(frame->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
		sensor_msgs::Image imgGray;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgGray.header.stamp = data_stamp;
		imgGray.header.frame_id = viewerParam.frame_id;
		imgGray.height = static_cast<uint32_t>(frame->height);
		imgGray.width = static_cast<uint32_t>(frame->width);
		imgGray.encoding = sensor_msgs::image_encodings::MONO16;
		imgGray.step = imgGray.width * 2;
		imgGray.is_bigendian = 0;
		imgGray.data = result;
		imgGrayPub.publish(imgGray);
	}		
	

#ifdef image_transfer_function
	if(frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
	
		int totalPixels = NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH;
		cv::Vec3b* dstPtr = rgbMat.ptr<cv::Vec3b>();
		NslOption::NslVec3b* srcPtr = &frame->rgb[0][0];
		
		for (int i = 0; i < totalPixels; ++i) {
			dstPtr[i] = cv::Vec3b(
				srcPtr[i].b,  // blue
				srcPtr[i].g,  // green
				srcPtr[i].r   // red
			);
		}

		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		cv_ptr->header.stamp = data_stamp;
		cv_ptr->header.frame_id = viewerParam.frame_id;
		cv_ptr->image = rgbMat;
		cv_ptr->encoding = "bgr8";
	
		imagePublisher.publish(cv_ptr->toImageMsg());		
	}
#endif

	if( frame->operationMode != OPERATION_MODE_OPTIONS::RGB_MODE
		&& frame->operationMode != OPERATION_MODE_OPTIONS::GRAYSCALE_MODE )
	{
		const size_t nPixel = frame->width * frame->height;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		cloud->header.frame_id = viewerParam.frame_id;
		cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
		//cloud->header.stamp = static_cast<uint64_t>(data_stamp.nanoseconds());
		cloud->width = static_cast<uint32_t>(frame->width);
		cloud->height = static_cast<uint32_t>(frame->height);
		cloud->is_dense = false;
		cloud->points.resize(nPixel);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;

		for(int y = 0, index = 0; y < frame->height; y++)
		{
			for(int x = 0; x < frame->width; x++, index++)
			{
				pcl::PointXYZI &point = cloud->points[index];

				if( frame->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA )
				{
					point.x = (double)(frame->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
					point.y = (double)(-frame->distance3D[OUT_X][y+yMin][x+xMin]/1000);
					point.z = (double)(-frame->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
					point.intensity = frame->amplitude[y+yMin][x+xMin];
				}
				else{
					point.x = std::numeric_limits<float>::quiet_NaN();
					point.y = std::numeric_limits<float>::quiet_NaN();
					point.z = std::numeric_limits<float>::quiet_NaN();
					point.intensity = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}

		
		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = viewerParam.frame_id;
		pointcloudPub.publish(msg);  
	}
	
	if(viewerParam.cvShow == true)
	{	
		getMouseEvent(mouseXpos, mouseYpos);
			
		if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE ){
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE ){
			distanceMat = addDistanceInfo(amplitudeMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE ){
			resize( rgbMat, rgbMat, Size( 640, 480 ), 0, 0);
			distanceMat = rgbMat;
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE ){
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			hconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		
		imshow(winName, distanceMat);
		waitKey(1);
	}
	
}

void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}

/*
	ubuntu usb device
	
	sudo apt-get install libopencv-dev
	sudo apt-get install libpcl-dev(1.8.1)

	$ sudo vi /etc/udev/rules.d/defined_lidar.rules
	KERNEL=="ttyACM*", ATTRS{idVendor}=="1FC9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyNsl3140"

	$ service udev reload
	$ service udev restart

	ubuntu Network UDP speed up
	sudo sysctl -w net.core.rmem_max=22020096
	sudo sysctl -w net.core.rmem_default=22020096
*/

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "roboscan_publish_node");
  nanosys::roboscanPublisher rp;
  rp.initialise();
  ros::spin();
  return 0;
}
