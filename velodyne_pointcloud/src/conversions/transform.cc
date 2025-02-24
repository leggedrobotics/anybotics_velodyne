/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /map frame of reference.

    @author Jack O'Quin
    @author Jesse Vera
    @author Sebastian Pütz

*/

#include "any_velodyne_pointcloud/transform.h"

#include <any_velodyne_pointcloud/pointcloudXYZIRT.h>
#include <any_velodyne_pointcloud/organized_cloudXYZIRT.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new velodyne_rawdata::RawData),
    first_rcfg_call(false),
    latestCloudStamp_(ros::Time(0)),
    lastPossibleMsgTime_(ros::Time(0))
  {
    boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration){
      ROS_INFO_STREAM("Calibration file loaded for VLP16.");
      config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else{
      ROS_ERROR_STREAM("Could not load calibration file!");
      throw std::runtime_error("Could not load calibration file!");
    }
    
    private_nh.param<std::string>("fixed_frame", config_.fixed_frame, "odom");
    private_nh.param<std::string>("frame_id", config_.target_frame, "lidar");

    ROS_INFO_STREAM("Fixed frame ID: " << config_.fixed_frame);
    ROS_INFO_STREAM("Target frame ID: " << config_.target_frame);

    // Read replay params
    private_nh.param<std::string>("input_rosbag_path", input_rosbag_path_, "");
    private_nh.param<std::string>("output_rosbag_path", output_rosbag_path_, "");

    // Remove the old bag file
    if (std::filesystem::exists(output_rosbag_path_.c_str()))
    {
      std::remove(output_rosbag_path_.c_str());
    }

    if ( (input_rosbag_path_ != "")){
      // Open the new bag file
      outputBag_.open(output_rosbag_path_, rosbag::bagmode::Write); 
      outputBag_.setCompression(rosbag::compression::LZ4);
      std::filesystem::permissions(
        output_rosbag_path_,
          std::filesystem::perms::owner_all | std::filesystem::perms::group_all,
          std::filesystem::perm_options::add
      );
    }else
    {
      ROS_ERROR_STREAM("Output rosbag path is empty. Exiting...");
      raise(SIGINT);
    }
    

    if ( (input_rosbag_path_ != "")){
    
      // ROS_WARN_STREAM("driver_ptr_->inputRosbagPath_: " << driver_ptr_->inputRosbagPath_);
      ROS_WARN_STREAM("Input rosbag path: " << input_rosbag_path_);
  
      // We need to do a dry-run on the rosbag to get the end time of the rosbag.
      rosbag::Bag inputBag;
      inputBag.open(input_rosbag_path_, rosbag::bagmode::Read);
      std::vector<std::string> viewtopics;
      viewtopics.push_back("/lidar/packets");
  
      {
        rosbag::View view(inputBag, rosbag::TopicQuery(viewtopics));
        ros::Time actualBagStartTime = ros::Time(0);

        {
          rosbag::View tfView(inputBag, rosbag::TopicQuery("/tf"));
          actualBagStartTime = tfView.getBeginTime();
        }

        ROS_WARN_STREAM("Actual Bag start time (/tf) = " << actualBagStartTime << " s");
        
        if (view.size() == 0) {
          ROS_WARN_STREAM("No messages found on " << viewtopics.front());
          return;
        }
  
        totalNumberOfPackets_ = view.size();
  
        auto first_it = view.begin();
        if (first_it != view.end())
        {
          // Instantiate as your actual message type
          any_velodyne_msgs::VelodyneScan::ConstPtr first_msg = first_it->instantiate<any_velodyne_msgs::VelodyneScan>();
          if (first_msg) {
            ROS_WARN_STREAM("First message header stamp = " << first_msg->header.stamp << " s");
            bagStartTime_ = first_msg->header.stamp;
          } else {
            ROS_ERROR_STREAM("Could not instantiate the first message.");
          }
        }
  
        auto it = view.begin();
  
        rosbag::View::iterator last_item;
        rosbag::View::iterator lastlast_item;
        while (it != view.end())
        {
          last_item = it++;
  
          if (it == view.end())
          {
            break;
          }
        }
        any_velodyne_msgs::VelodyneScan::ConstPtr last_msg = last_item->instantiate<any_velodyne_msgs::VelodyneScan>();
        ROS_WARN_STREAM("Last message header stamp  = " << last_msg->header.stamp << " s");
        lastPossibleMsgTime_ = last_msg->header.stamp;
        bagEndTime_ = last_msg->header.stamp;
  
        ros::Duration rosbag_duration = bagEndTime_ - bagStartTime_;
        duration_ = rosbag_duration.toNSec();
        ROS_WARN_STREAM("Duration: " << rosbag_duration.toSec()  << " s");

        ros::Duration warningWaitTime = bagStartTime_ - actualBagStartTime;
        ROS_WARN_STREAM("Warning, Packets appear late in the bag: " << warningWaitTime.toSec()  << " s");
      }
  
    }else{
      ROS_ERROR_STREAM("Input rosbag path is empty. Exiting...");
      raise(SIGINT);
    }

    // TF listener
    tf_ptr_ = boost::make_shared<tf::TransformListener>();

    // Container
    container_ptr = boost::shared_ptr<PointcloudXYZIRT>(
        new PointcloudXYZIRT(config_.max_range, config_.min_range,
                            config_.target_frame, config_.fixed_frame,
                            data_->scansPerPacket(), tf_ptr_));

    // Ddvertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("/anymal/velodyne/points_undistorted", 1);

    outputDistorted_ =
      node.advertise<sensor_msgs::PointCloud2>("/anymal/velodyne/points", 1);

    // Dynamic reconfig.
    srv_ = boost::make_shared<dynamic_reconfigure::Server<TransformNodeConfig>> (private_nh);
    dynamic_reconfigure::Server<TransformNodeConfig>::CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets using transform filter
    velodyne_scan_.subscribe(node, "/lidar/packets", 1000);
    tf_filter_ptr_ = boost::shared_ptr<tf::MessageFilter<any_velodyne_msgs::VelodyneScan> >(
            new tf::MessageFilter<any_velodyne_msgs::VelodyneScan>(velodyne_scan_, *tf_ptr_, config_.target_frame, 10));
    tf_filter_ptr_->registerCallback(boost::bind(&Transform::processScan, this, _1));

  }
  
  Transform::~Transform()
  {
    std::cout << "Stoping Velodyne Replayer...." << std::endl;
    {
      boost::lock_guard<boost::mutex> guard(rosbagMutex_);
  
      if (outputBag_.isOpen())
      {
        outputBag_.close();
        std::cout << "Rosbag Closed." << std::endl;
      }
    }
    ros::shutdown();
    raise(SIGINT);
  }

  void Transform::reconfigure_callback(TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range,
                         config.view_direction, config.view_width);
    config_.fixed_frame = tf::resolve(tf_prefix_, config.fixed_frame);
    config_.target_frame = tf::resolve(tf_prefix_, config.target_frame);
    ROS_INFO_STREAM("Fixed frame ID (after reconfigure): " << config_.fixed_frame);
    ROS_INFO_STREAM("Target frame ID (after reconfigure): " << config_.target_frame);
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
      first_rcfg_call = false;
      config_.organize_cloud = config.organize_cloud;
      if(config_.organize_cloud)
      {
        ROS_INFO_STREAM("Using the organized cloud format...");
        container_ptr = boost::shared_ptr<OrganizedCloudXYZIRT>(
            new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                                    config_.target_frame, config_.fixed_frame,
                                    config_.num_lasers, data_->scansPerPacket()));
      }
      else
      {
        container_ptr = boost::shared_ptr<PointcloudXYZIRT>(
            new PointcloudXYZIRT(config_.max_range, config_.min_range,
                                config_.target_frame, config_.fixed_frame,
                                data_->scansPerPacket()));
      }
    }
    container_ptr->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const any_velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    counter_++;
    ros::Time packetHeaderTime = scanMsg->header.stamp;

    {
      boost::lock_guard<boost::mutex> guard(rosbagMutex_);
      outputBag_.write("/anymal/velodyne/packets", scanMsg->header.stamp, scanMsg);
    }

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->setup(scanMsg);

    const ros::Time referencePointCloudTime{scanMsg->packets[0].stamp};
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      ROS_DEBUG_STREAM("Unpacking and transforming Lidar Data Packet Index: " << i);
      container_ptr->computeTransformation(scanMsg->packets[i].stamp, referencePointCloudTime, scanMsg->header.frame_id);
      data_->unpack(scanMsg->packets[i], *container_ptr,  referencePointCloudTime);
    }

    // reset transformation to de-skew packets.
    container_ptr->resetTransformation();

    auto writeableScan = container_ptr->finishCloud(referencePointCloudTime);
    writeableScan.header.frame_id = "velodyne";
    {
      boost::lock_guard<boost::mutex> guard(rosbagMutex_);
      outputBag_.write("/anymal/velodyne/points_undistorted", referencePointCloudTime, writeableScan);
    }

    ////////////////////
    container_ptr->setup(scanMsg);

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      // ROS_DEBUG_STREAM("Unpacking Lidar Data Packet Index: " << i);
      data_->unpack(scanMsg->packets[i], *container_ptr, scanMsg->header.stamp);
    }

    auto writeableDistortedScan = container_ptr->finishCloud(scanMsg->header.stamp);
    writeableDistortedScan.header.frame_id = "velodyne";
    {
      boost::lock_guard<boost::mutex> guard(rosbagMutex_);
      outputBag_.write("/anymal/velodyne/points", scanMsg->header.stamp, writeableDistortedScan);
    }

    if ( ( (counter_ % 50 == 0)  ||  (totalNumberOfPackets_ - counter_) < 11 ) && (counter_ != 0))
    {
      uint64_t m_time = packetHeaderTime.toNSec();
      if ( (duration_ > 0) && (bagStartTime_.toNSec() > 0))
      {
        float progress = (float)(m_time - bagStartTime_.toNSec()) / (float)duration_ * 100;
        ROS_INFO("Processing PC message ( %.2f%% )", progress);
      }

      ROS_INFO_STREAM("Processed Message Ratio: " << counter_  << "/" << totalNumberOfPackets_);
    }


    if ( (packetHeaderTime == lastPossibleMsgTime_) || ( counter_ == totalNumberOfPackets_) )
    {
      ROS_INFO_STREAM("Total number of expected packets: " << totalNumberOfPackets_);
      ROS_INFO_STREAM("Packet Counter: " << counter_);
      ROS_INFO_STREAM("Velodyne Packets Start time: " << bagStartTime_);
      ROS_INFO_STREAM("\033[1;32mLast packet received. All good. Closing rosbag.\033[0m");
      this->~Transform();
    }

  }

} // namespace velodyne_pointcloud
