/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"


#include <Converter.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;

class ImageGrabber
{
public:
    ORB_SLAM2::System* mpSLAM;

    ros::Publisher kf_publisher;
    ros::Publisher kf_stamped_publisher;
    tf::TransformBroadcaster* br;    

public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle nh, tf::TransformBroadcaster* _br):mpSLAM(pSLAM){
        kf_publisher = nh.advertise<geometry_msgs::Pose>("/orb_slam/keyframe_optimized", 10);
        kf_stamped_publisher = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam/keyframe_stamped_optimized", 10);
        br = _br;
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM, nh, &br);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    vector<ORB_SLAM2::KeyFrame*> key_frames = mpSLAM->getMap()->GetAllKeyFrames();

    sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
    
    cv::Mat Two = key_frames[0]->GetPoseInverse();

    ORB_SLAM2::KeyFrame* pKF = mpSLAM->getTracker()->mCurrentFrame.mpReferenceKF;
    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    Trw = Trw*pKF->GetPose()*Two;
    cv::Mat lit = mpSLAM->getTracker()->mlRelativeFramePoses.back();
    cv::Mat Tcw = lit*Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    ORB_SLAM2::KeyFrame *key_frame = key_frames.back();

    if (key_frame->isBad())
        return;

    // cv::Mat R = key_frame->GetRotation().t();
    // vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
    // cv::Mat twc = key_frame->GetCameraCenter();

    geometry_msgs::Pose kf_pose;

    kf_pose.position.x = twc.at<float>(0);
    kf_pose.position.y = twc.at<float>(1);
    kf_pose.position.z = twc.at<float>(2);
    kf_pose.orientation.x = q[0];
    kf_pose.orientation.y = q[1];
    kf_pose.orientation.z = q[2];
    kf_pose.orientation.w = q[3];

    kf_publisher.publish(kf_pose);

    geometry_msgs::PoseStamped kf_pose_stamped;
    kf_pose_stamped.header.stamp = ros::Time::now();
    kf_pose_stamped.header.frame_id = "world";
    kf_pose_stamped.pose = kf_pose;
    kf_stamped_publisher.publish(kf_pose_stamped);

    tf::StampedTransform transform;
    tf::poseMsgToTF(kf_pose, transform);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "orb_slam"));
}