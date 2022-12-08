// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ****************************************************************************

// Eigen include
#include <Eigen/Geometry> 

#include <septentrio_gnss_driver/node/rosaic_node.hpp>

/**
 * @file rosaic_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

rosaic_node::ROSaicNode::ROSaicNode(const rclcpp::NodeOptions &options) :
    ROSaicNodeBase(options),
    IO_(this, &settings_),
    tfBuffer_(this->get_clock())
{
    param("activate_debug_log", settings_.activate_debug_log, false);
    if (settings_.activate_debug_log)
    {
        auto ret = rcutils_logging_set_logger_level(
            this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
    }
    
    this->log(LogLevel::DEBUG, "Called ROSaicNode() constructor..");

    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

    // Parameters must be set before initializing IO
    if (!getROSParams())
        return;

    IO_.initializeDiag();

    // Initializes Connection
    IO_.initializeIO();

    // Subscribes to all requested Rx messages by adding entries to the C++ multimap
    // storing the callback handlers and publishes ROS messages
    IO_.defineMessages();

    // Sends commands to the Rx regarding which SBF/NMEA messages it should output
    // and sets all its necessary corrections-related parameters
    if (!settings_.read_from_sbf_log && !settings_.read_from_pcap)
    {
        IO_.configureRx();
    }

    this->log(LogLevel::DEBUG, "Leaving ROSaicNode() constructor..");
}

bool rosaic_node::ROSaicNode::getROSParams()
{
    param("use_gnss_time", settings_.use_gnss_time, true);
    param("frame_id", settings_.frame_id, (std::string) "gnss");
    param("imu_frame_id", settings_.imu_frame_id, (std::string) "imu");
    param("poi_frame_id", settings_.poi_frame_id, (std::string) "base_link");
    param("vsm_frame_id", settings_.vsm_frame_id, (std::string) "vsm");
    param("aux1_frame_id", settings_.aux1_frame_id, (std::string) "aux1");
    param("vehicle_frame_id", settings_.vehicle_frame_id, settings_.poi_frame_id);
    param("lock_utm_zone", settings_.lock_utm_zone, true);
    getUint32Param("leap_seconds", settings_.leap_seconds, static_cast<uint32_t>(18));

    // Communication parameters
    param("device", settings_.device, std::string("/dev/ttyACM0"));
    getUint32Param("serial.baudrate", settings_.baudrate, static_cast<uint32_t>(921600));
    param("serial.hw_flow_control", settings_.hw_flow_control, std::string("off"));
    param("serial.rx_serial_port", settings_.rx_serial_port, std::string("USB1"));
    param("login.user", settings_.login_user, std::string(""));
    param("login.password", settings_.login_password, std::string(""));
    
    settings_.reconnect_delay_s = 2.0f; // Removed from ROS parameter list.
    param("receiver_type", settings_.septentrio_receiver_type, std::string("gnss"));
    if (!((settings_.septentrio_receiver_type == "gnss") || (settings_.septentrio_receiver_type == "ins")))
    {
        this->log(LogLevel::FATAL, "Unkown septentrio_receiver_type " + settings_.septentrio_receiver_type + " use either gnss or ins.");
        return false;
    }
	
    // Polling period parameters
    getUint32Param("polling_period.pvt", settings_.polling_period_pvt,
              static_cast<uint32_t>(1000));
    if (settings_.polling_period_pvt != 10 && settings_.polling_period_pvt != 20 &&
        settings_.polling_period_pvt != 50 && settings_.polling_period_pvt != 100 &&
        settings_.polling_period_pvt != 200 && settings_.polling_period_pvt != 250 &&
        settings_.polling_period_pvt != 500 && settings_.polling_period_pvt != 1000 &&
        settings_.polling_period_pvt != 2000 && settings_.polling_period_pvt != 5000 &&
        settings_.polling_period_pvt != 10000 && settings_.polling_period_pvt != 0)
    {
        this->log(LogLevel::FATAL,
            "Please specify a valid polling period for PVT-related SBF blocks and NMEA messages. " + std::to_string(settings_.polling_period_pvt));
        return false;
    }
    getUint32Param("polling_period.rest", settings_.polling_period_rest,
              static_cast<uint32_t>(1000));
    if (settings_.polling_period_rest != 10 && settings_.polling_period_rest != 20 &&
        settings_.polling_period_rest != 50 && settings_.polling_period_rest != 100 &&
        settings_.polling_period_rest != 200 && settings_.polling_period_rest != 250 &&
        settings_.polling_period_rest != 500 && settings_.polling_period_rest != 1000 &&
        settings_.polling_period_rest != 2000 && settings_.polling_period_rest != 5000 &&
        settings_.polling_period_rest != 10000)
    {
        this->log(LogLevel::FATAL,
            "Please specify a valid polling period for PVT-unrelated SBF blocks and NMEA messages.");
        return false;
    }

    // multi_antenna param
    param("multi_antenna", settings_.multi_antenna, false);

    // Publishing parameters
    param("publish.gpst", settings_.publish_gpst, false);
    param("publish.navsatfix", settings_.publish_navsatfix, true);
    param("publish.gpsfix", settings_.publish_gpsfix, false);
    param("publish.pose", settings_.publish_pose, false);
    param("publish.diagnostics", settings_.publish_diagnostics, false);
    param("publish.gpgga", settings_.publish_gpgga, false);
    param("publish.gprmc", settings_.publish_gprmc, false);
    param("publish.gpgsa", settings_.publish_gpgsa, false);
    param("publish.gpgsv", settings_.publish_gpgsv, false);
    param("publish.measepoch", settings_.publish_measepoch, false);
    param("publish.pvtcartesian", settings_.publish_pvtcartesian, false);
    param("publish.pvtgeodetic", settings_.publish_pvtgeodetic, (settings_.septentrio_receiver_type == "gnss"));
    param("publish.poscovcartesian", settings_.publish_poscovcartesian, false);
    param("publish.poscovgeodetic", settings_.publish_poscovgeodetic, (settings_.septentrio_receiver_type == "gnss"));
	param("publish.velcovgeodetic", settings_.publish_velcovgeodetic, (settings_.septentrio_receiver_type == "gnss"));
    param("publish.atteuler", settings_.publish_atteuler, ((settings_.septentrio_receiver_type == "gnss") && settings_.multi_antenna));
    param("publish.attcoveuler", settings_.publish_attcoveuler, ((settings_.septentrio_receiver_type == "gnss") && settings_.multi_antenna));
    param("publish.insnavcart", settings_.publish_insnavcart, false);
    param("publish.insnavgeod", settings_.publish_insnavgeod, (settings_.septentrio_receiver_type == "ins"));
    param("publish.imusetup", settings_.publish_imusetup, false);
    param("publish.velsensorsetup", settings_.publish_velsensorsetup, false);
    param("publish.exteventinsnavgeod", settings_.publish_exteventinsnavgeod, false);
    param("publish.exteventinsnavcart", settings_.publish_exteventinsnavcart, false);
    param("publish.extsensormeas", settings_.publish_extsensormeas, false);
    param("publish.imu", settings_.publish_imu, false);
    param("publish.localization", settings_.publish_localization, false);
    param("publish.tf", settings_.publish_tf, false);

    // Datum and marker-to-ARP offset
    param("datum", settings_.datum, std::string("ETRS89"));
    param("ant_type", settings_.ant_type, std::string("Unknown"));
    param("ant_aux1_type", settings_.ant_aux1_type, std::string("Unknown"));
    if (!param("ant_serial_nr", settings_.ant_serial_nr, std::string()))
    {
        uint32_t sn_tmp;
        if (getUint32Param("ant_serial_nr", sn_tmp, static_cast<uint32_t>(0)))
            settings_.ant_serial_nr = std::to_string(sn_tmp);
        else
            settings_.ant_serial_nr = "Unknown";
    }
    if (!param("ant_aux1_serial_nr", settings_.ant_aux1_serial_nr, std::string()))
    {
        uint32_t sn_tmp;
        if (getUint32Param("ant_aux1_serial_nr", sn_tmp, static_cast<uint32_t>(0)))
            settings_.ant_aux1_serial_nr = std::to_string(sn_tmp);
        else
            settings_.ant_aux1_serial_nr = "Unknown";
    }
    param("poi_to_arp.delta_e", settings_.delta_e, 0.0f);
    param("poi_to_arp.delta_n", settings_.delta_n, 0.0f);
    param("poi_to_arp.delta_u", settings_.delta_u, 0.0f);

    param("use_ros_axis_orientation", settings_.use_ros_axis_orientation, true);

	// INS Spatial Configuration
    // bool getConfigFromTf;
    // param("get_spatial_config_from_tf", getConfigFromTf, false);
    // if (getConfigFromTf)
    // {
    //     // Wait a little for tf to become ready
    //     using namespace std::chrono_literals;
    //     std::this_thread::sleep_for(1000ms);

    //     if (settings_.septentrio_receiver_type == "ins")
    //     {
    //         TransformStampedMsg T_imu_vehicle;
    //         getTransform(settings_.vehicle_frame_id, settings_.imu_frame_id, T_imu_vehicle);
    //         TransformStampedMsg T_poi_imu;
    //         getTransform(settings_.imu_frame_id, settings_.poi_frame_id, T_poi_imu);
    //         TransformStampedMsg T_vsm_imu;
    //         getTransform(settings_.imu_frame_id, settings_.vsm_frame_id, T_vsm_imu);
    //         TransformStampedMsg T_ant_imu;
    //         getTransform(settings_.imu_frame_id, settings_.frame_id, T_ant_imu); 

    //         // IMU orientation parameter
    //         double roll, pitch, yaw;
    //         getRPY(T_imu_vehicle.transform.rotation, roll, pitch, yaw);
    //         settings_.theta_x = parsing_utilities::rad2deg(roll);
    //         settings_.theta_y = parsing_utilities::rad2deg(pitch);
    //         settings_.theta_z = parsing_utilities::rad2deg(yaw);
    //         // INS antenna lever arm offset parameter
    //         settings_.ant_lever_x = T_ant_imu.transform.translation.x;
    //         settings_.ant_lever_y = T_ant_imu.transform.translation.y;
    //         settings_.ant_lever_z = T_ant_imu.transform.translation.z;
    //         // INS POI ofset paramter
    //         settings_.poi_x = T_poi_imu.transform.translation.x;
    //         settings_.poi_y = T_poi_imu.transform.translation.y;
    //         settings_.poi_z = T_poi_imu.transform.translation.z;
    //         // INS velocity sensor lever arm offset parameter
    //         settings_.vsm_x = T_vsm_imu.transform.translation.x;
    //         settings_.vsm_y = T_vsm_imu.transform.translation.y;
    //         settings_.vsm_z = T_vsm_imu.transform.translation.z;

    //         if (settings_.multi_antenna)
    //         {
    //             TransformStampedMsg T_aux1_imu;
    //             getTransform(settings_.imu_frame_id, settings_.aux1_frame_id, T_aux1_imu);
    //             // Antenna Attitude Determination parameter
    //             double dy = T_aux1_imu.transform.translation.y - T_ant_imu.transform.translation.y;
    //             double dx = T_aux1_imu.transform.translation.x - T_ant_imu.transform.translation.x;
    //             settings_.heading_offset = parsing_utilities::rad2deg(std::atan2(dy, dx));
    //             double dz = T_aux1_imu.transform.translation.z - T_ant_imu.transform.translation.z;
    //             double dr = std::sqrt(parsing_utilities::square(dx) + parsing_utilities::square(dy));
    //             settings_.pitch_offset = parsing_utilities::rad2deg(std::atan2(-dz, dr));
    //         }
    //     }
    //     if ((settings_.septentrio_receiver_type == "gnss") && settings_.multi_antenna)
    //     {
    //         TransformStampedMsg T_ant_vehicle;
    //         getTransform(settings_.vehicle_frame_id, settings_.frame_id, T_ant_vehicle);
    //         TransformStampedMsg T_aux1_vehicle;
    //         getTransform(settings_.vehicle_frame_id, settings_.aux1_frame_id, T_aux1_vehicle);

    //         // Antenna Attitude Determination parameter
    //         double dy = T_aux1_vehicle.transform.translation.y - T_ant_vehicle.transform.translation.y;
    //         double dx = T_aux1_vehicle.transform.translation.x - T_ant_vehicle.transform.translation.x;
    //         settings_.heading_offset = parsing_utilities::rad2deg(std::atan2(dy, dx));
    //         double dz = T_aux1_vehicle.transform.translation.z - T_ant_vehicle.transform.translation.z;
    //         double dr = std::sqrt(parsing_utilities::square(dx) + parsing_utilities::square(dy));
    //         settings_.pitch_offset = parsing_utilities::rad2deg(std::atan2(-dz, dr));
    //     }
    // }
    // else
    // {
    //     // IMU orientation parameter
    //     param("ins_spatial_config.imu_orientation.theta_x", settings_.theta_x, 0.0);
    //     param("ins_spatial_config.imu_orientation.theta_y", settings_.theta_y, 0.0);
    //     param("ins_spatial_config.imu_orientation.theta_z", settings_.theta_z, 0.0);
    //     // INS antenna lever arm offset parameter
    //     param("ins_spatial_config.ant_lever_arm.x", settings_.ant_lever_x, 0.0);
    //     param("ins_spatial_config.ant_lever_arm.y", settings_.ant_lever_y, 0.0);
    //     param("ins_spatial_config.ant_lever_arm.z", settings_.ant_lever_z, 0.0);
    //     // INS POI ofset paramter
    //     param("ins_spatial_config.poi_lever_arm.delta_x", settings_.poi_x, 0.0);
    //     param("ins_spatial_config.poi_lever_arm.delta_y", settings_.poi_y, 0.0);
    //     param("ins_spatial_config.poi_lever_arm.delta_z", settings_.poi_z, 0.0);
    //     // INS velocity sensor lever arm offset parameter
    //     param("ins_spatial_config.vel_sensor_lever_arm.vsm_x", settings_.vsm_x, 0.0);
    //     param("ins_spatial_config.vel_sensor_lever_arm.vsm_y", settings_.vsm_y, 0.0);
    //     param("ins_spatial_config.vel_sensor_lever_arm.vsm_z", settings_.vsm_z, 0.0);
    //     // Antenna Attitude Determination parameter
    //     param("att_offset.heading", settings_.heading_offset, 0.0);
    //     param("att_offset.pitch", settings_.pitch_offset, 0.0);
    // }
    
    // if (settings_.use_ros_axis_orientation)
    // {
    //     settings_.theta_x = parsing_utilities::wrapAngle180to180(settings_.theta_x + 180.0);
    //     settings_.theta_y *= -1.0;
    //     settings_.theta_z *= -1.0;
    //     settings_.ant_lever_y *= -1.0;
    //     settings_.ant_lever_z *= -1.0;
    //     settings_.poi_y *= -1.0;
    //     settings_.poi_z *= -1.0;
    //     settings_.vsm_y *= -1.0;
    //     settings_.vsm_z *= -1.0;
    //     settings_.heading_offset *= -1.0;
    //     settings_.pitch_offset   *= -1.0;
    // }

    // IMU orientation parameter
    param("ins_spatial_config.imu_orientation.theta_x", settings_.theta_x, 0.0);
    param("ins_spatial_config.imu_orientation.theta_y", settings_.theta_y, 0.0);
    param("ins_spatial_config.imu_orientation.theta_z", settings_.theta_z, 0.0);
    // INS antenna lever arm offset parameter
    param("ins_spatial_config.ant_lever_arm.x", settings_.ant_lever_x, 0.0);
    param("ins_spatial_config.ant_lever_arm.y", settings_.ant_lever_y, 0.0);
    param("ins_spatial_config.ant_lever_arm.z", settings_.ant_lever_z, 0.0);
    // INS POI ofset paramter
    param("ins_spatial_config.poi_lever_arm.delta_x", settings_.poi_x, 0.0);
    param("ins_spatial_config.poi_lever_arm.delta_y", settings_.poi_y, 0.0);
    param("ins_spatial_config.poi_lever_arm.delta_z", settings_.poi_z, 0.0);
    // INS velocity sensor lever arm offset parameter
    param("ins_spatial_config.vel_sensor_lever_arm.vsm_x", settings_.vsm_x, 0.0);
    param("ins_spatial_config.vel_sensor_lever_arm.vsm_y", settings_.vsm_y, 0.0);
    param("ins_spatial_config.vel_sensor_lever_arm.vsm_z", settings_.vsm_z, 0.0);
    // Antenna Attitude Determination parameter
    param("att_offset.heading", settings_.heading_offset, 0.0);
    param("att_offset.pitch", settings_.pitch_offset, 0.0);

    if (std::abs(settings_.heading_offset) > std::numeric_limits<double>::epsilon())
    {
        if (settings_.publish_atteuler)
        {
            this->log(LogLevel::WARN , "Pitch angle output by topic /atteuler is a tilt angle rotated by " + 
                                    std::to_string(settings_.heading_offset) + ".");
        }
        if (settings_.publish_pose && (settings_.septentrio_receiver_type == "gnss"))
        {
            this->log(LogLevel::WARN , "Pitch angle output by topic /pose is a tilt angle rotated by " + 
                                        std::to_string(settings_.heading_offset) + ".");
        }
    }

    this->log(LogLevel::DEBUG , "imu_orientation.theta_x: " + std::to_string(settings_.theta_x));
    this->log(LogLevel::DEBUG , "imu_orientation.theta_y: " + std::to_string(settings_.theta_y));
    this->log(LogLevel::DEBUG , "imu_orientation.theta_z: " + std::to_string(settings_.theta_z));
    this->log(LogLevel::DEBUG , "ant_lever_arm.x: " + std::to_string(settings_.ant_lever_x));
    this->log(LogLevel::DEBUG , "ant_lever_arm.y: " + std::to_string(settings_.ant_lever_y));
    this->log(LogLevel::DEBUG , "ant_lever_arm.z: " + std::to_string(settings_.ant_lever_z));
    this->log(LogLevel::DEBUG , "att_offset.heading:" + std::to_string(settings_.heading_offset));
    this->log(LogLevel::DEBUG , "att_offset.pitch: " + std::to_string(settings_.pitch_offset));
    
    // ins_initial_heading param
    param("ins_initial_heading", settings_.ins_initial_heading, std::string("auto"));

    // ins_std_dev_mask
    param("ins_std_dev_mask.att_std_dev", settings_.att_std_dev, 5.0f);
    param("ins_std_dev_mask.pos_std_dev", settings_.pos_std_dev, 10.0f);

    // diagnostics_updater
    param("diagnostics.hardware_id", settings_.hardware_id, std::string("septentrio"));
    param("diagnostics.check_rate", settings_.check_rate, 1.0);

    // Additional Settings
    if (settings_.septentrio_receiver_type == "ins")
    {
        param("navsatfix_output.navsatfix_topic", settings_.navsatfix_topic, std::string("/navsatfix"));

        param("coordinate_convert.coordinate", settings_.coordinate, std::string("PLANE"));
        param("coordinate_convert.height_type", settings_.height_type, std::string("Orthometric"));
        getUint32Param("coordinate_convert.plane_num", settings_.plane_num, static_cast<uint32_t>(7));
        param("coordinate_convert.correct_meridian_convergence", settings_.correct_heading, true);

        param("pose_output.pose_topic", settings_.pose_topic, std::string("/pose"));
        param("pose_output.pose_with_cov_topic", settings_.pose_cov_topic, std::string("/pose_with_covariance"));
        
        bool config_from_tf = false;
        param("baselink_pose_output.config_from_tf", config_from_tf, false);
        if (config_from_tf)
        {
            getTransform(settings_.vehicle_frame_id, settings_.frame_id, settings_.ins_to_baselink);
        }
        else
        {
            float ins_to_baselink_x, ins_to_baselink_y, ins_to_baselink_z;
            tf2::Quaternion tf2_quat;

            param("baselink_pose_output.ins_to_baselink.x", ins_to_baselink_x, 0.0f);
            param("baselink_pose_output.ins_to_baselink.y", ins_to_baselink_y, 0.0f);
            param("baselink_pose_output.ins_to_baselink.z", ins_to_baselink_z, 0.0f);
            
            settings_.ins_to_baselink.transform.translation.x = ins_to_baselink_x;
            settings_.ins_to_baselink.transform.translation.y = ins_to_baselink_y;
            settings_.ins_to_baselink.transform.translation.z = ins_to_baselink_z;
        }

        param("baselink_pose_output.pose_topic", settings_.baselink_pose_topic, std::string("/baslink_pose"));
        param("baselink_pose_output.pose_with_cov_topic", settings_.baselink_pose_cov_topic, std::string("/baselink_pose_with_covariance"));
        
        this->declare_parameter("output_stopping_limit.INSNavGeod_Error_enable",std::vector<int64_t>(1,0));
        settings_.enabled_errors = this->get_parameter("output_stopping_limit.INSNavGeod_Error_enable").as_integer_array();
        param("output_stopping_limit.max_longitude_covariance", settings_.min_lon_cov, 0.1f);
        param("output_stopping_limit.max_latitude_covariance", settings_.min_lat_cov, 0.1f);
        param("output_stopping_limit.max_height_covariance", settings_.min_height_cov, 5.0f);

        // for (auto &&i : settings_.enabled_errors)
        // {
        //     this->log(LogLevel::DEBUG , "output_stopping_limit.INSNavGeod_Error_enable: " + std::to_string(i));
        // }
        // this->log(LogLevel::DEBUG , "output_stopping_limit.max_longitude_covariance: " + std::to_string(settings_.min_lon_cov));
        // this->log(LogLevel::DEBUG , "output_stopping_limit.max_latitude_covariance: " + std::to_string(settings_.min_lat_cov));
        // this->log(LogLevel::DEBUG , "output_stopping_limit.max_height_covariance: " + std::to_string(settings_.min_height_cov));
    }

    // INS solution reference point
    param("ins_use_poi", settings_.ins_use_poi, false);

    if (settings_.publish_tf && !settings_.ins_use_poi)
    {
        this->log(LogLevel::ERROR , "If tf shall be published, ins_use_poi has to be set to true! It is set automatically to true."); 
        settings_.ins_use_poi = true;
    }

    // Correction service parameters
    param("ntrip_settings.mode", settings_.ntrip_mode, std::string("off"));
    param("ntrip_settings.caster", settings_.caster, std::string());
    getUint32Param("ntrip_settings.caster_port", settings_.caster_port, static_cast<uint32_t>(0));
    param("ntrip_settings.username", settings_.ntrip_username, std::string());
    if (!param("ntrip_settings.password", settings_.ntrip_password, std::string()))
    {
        uint32_t pwd_tmp;
        getUint32Param("ntrip_settings.password", pwd_tmp, static_cast<uint32_t>(0));
        settings_.ntrip_password = std::to_string(pwd_tmp);
    }
    param("ntrip_settings.mountpoint", settings_.mountpoint, std::string());
    param("ntrip_settings.ntrip_version", settings_.ntrip_version, std::string("v2"));
    param("ntrip_settings.send_gga", settings_.send_gga, std::string("auto"));
    param("ntrip_settings.rx_has_internet", settings_.rx_has_internet, false);
    param("ntrip_settings.rtcm_version", settings_.rtcm_version, std::string("RTCMv3"));
    getUint32Param("ntrip_settings.rx_input_corrections_tcp", settings_.rx_input_corrections_tcp,
              static_cast<uint32_t>(28785));
    param("ntrip_settings.rx_input_corrections_serial",
                settings_.rx_input_corrections_serial, std::string("USB2"));

    if (settings_.publish_atteuler)
    {
        if (!settings_.multi_antenna)
        {
            this->log(LogLevel::WARN ,"AttEuler needs multi-antenna receiver. Multi-antenna setting automatically activated. Deactivate publishing of AttEuler if multi-antenna operation is not available.");
            settings_.multi_antenna = true;
        }
    }

    // To be implemented: RTCM, raw data settings, PPP, SBAS ...
    this->log(LogLevel::DEBUG ,"Finished getROSParams() method");
    return true;
}

void rosaic_node::ROSaicNode::getTransform(const std::string& targetFrame, const std::string& sourceFrame, TransformStampedMsg& T_s_t)
{
    bool found = false;
    while (!found)
    {
        try
        {
            // try to get tf from source frame to target frame
            T_s_t = tfBuffer_.lookupTransform(targetFrame, sourceFrame, rclcpp::Time(0));
            found = true;
        }
        catch (const tf2::TransformException& ex)
        {
            this->log(LogLevel::WARN, "Waiting for transform from " + sourceFrame + " to " + targetFrame + ": " + ex.what() + ".");
            found = false;
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2000ms);
        }
    }
}

void rosaic_node::ROSaicNode::getRPY(const QuaternionMsg& qm, double& roll, double& pitch, double& yaw)
{
    Eigen::Quaterniond q(qm.w, qm.x, qm.y, qm.z);
	Eigen::Quaterniond::RotationMatrixType C = q.matrix();

	roll  = std::atan2(C(2, 1), C(2, 2));
	pitch = std::asin(-C(2, 0));
	yaw   = std::atan2(C(1, 0), C(0, 0));
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rosaic_node::ROSaicNode)