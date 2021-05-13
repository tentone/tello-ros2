#ifndef MONOCULAR_SLAM_NODE_HPP
#define MONOCULAR_SLAM_NODE_HPP


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>

#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"


class MonocularSlamNode : public rclcpp::Node
{

public:

    MonocularSlamNode(ORB_SLAM2::System* pSLAM, const string &strVocFile, const string &strSettingsFile);


    ~MonocularSlamNode();


private: 


    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void UpdateSLAMState();

    void UpdateMapState();


    void PublishFrame();

    void PublishCurrentCamera();

    void InitializeMarkersPublisher( const string &strSettingPath);
    void PublishMapPoints();
    void PublishKeyFrames();


    cv::Mat DrawFrame();

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);



    ORB_SLAM2::System* m_SLAM;

    std::mutex mMutex;

    cv_bridge::CvImagePtr m_cvImPtr;
    cv::Mat Tcw;
    
    
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;


    vector<ORB_SLAM2::KeyFrame*> mvKeyFrames;
    vector<ORB_SLAM2::MapPoint*> mvMapPoints;
    vector<ORB_SLAM2::MapPoint*> mvRefMapPoints;

    visualization_msgs::msg::Marker mPoints;
    visualization_msgs::msg::Marker mReferencePoints;
    visualization_msgs::msg::Marker mKeyFrames;
    visualization_msgs::msg::Marker mReferenceKeyFrames;
    visualization_msgs::msg::Marker mCovisibilityGraph;
    visualization_msgs::msg::Marker mMST;
    visualization_msgs::msg::Marker mCurrentCamera;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    int mState;
    bool mbOnlyTracking;
    bool mbUpdated;
    bool mbCameraUpdated;
    bool mbMapUpdated;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_annotated_image_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_map_publisher;

};



#endif