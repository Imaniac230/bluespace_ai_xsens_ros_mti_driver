// BSD 3-Clause License
//
// Copyright (c) 2021, BlueSpace.ai, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xdainterface.h"
#include "xdautils.h"

#include <vector>

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsfilterprofilearray.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"

#include "services/orientationreset.hpp"

XdaInterface::XdaInterface(const std::string &node_name, const rclcpp::NodeOptions &options)
	: Node(node_name, options)
	, m_device(nullptr)
	, m_xdaCallback(*this)
{
	declareCommonParameters();
	RCLCPP_INFO(get_logger(), "Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
	RCLCPP_INFO(get_logger(), "Cleaning up ...");
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)
		{
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

void XdaInterface::registerPublishers()
{
	bool should_publish;
	rclcpp::Node& node = *this;

	if (get_parameter("pub_imu", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<ImuPublisher>(node));
	}
	if (get_parameter("pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<OrientationPublisher>(node));
	}
	if (get_parameter("pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<AccelerationPublisher>(node));
	}
	if (get_parameter("pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<AngularVelocityPublisher>(node));
	}
	if (get_parameter("pub_mag", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<MagneticFieldPublisher>(node));
	}
	if (get_parameter("pub_dq", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<OrientationIncrementsPublisher>(node));
	}
	if (get_parameter("pub_dv", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<VelocityIncrementPublisher>(node));
	}
	if (get_parameter("pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<TimeReferencePublisher>(node));
	}
	if (get_parameter("pub_temperature", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<TemperaturePublisher>(node));
	}
	if (get_parameter("pub_pressure", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<PressurePublisher>(node));
	}
	if (get_parameter("pub_gnss", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<GnssPublisher>(node));
	}
	if (get_parameter("pub_twist", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<TwistPublisher>(node));
	}
	if (get_parameter("pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<FreeAccelerationPublisher>(node));
	}
	if (get_parameter("pub_transform", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<TransformPublisher>(node));
	}
	if (get_parameter("pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<PositionLLAPublisher>(node));
	}
	if (get_parameter("pub_velocity", should_publish) && should_publish)
	{
		registerCallback(std::make_unique<VelocityPublisher>(node));
	}
}

bool XdaInterface::connectDevice()
{
	XsPortInfo mtPort;
	XsBaudRate baudrate = XBR_Invalid;

        // Read device ID parameter if set
	bool checkDeviceID = false;
	std::string deviceId;
        get_parameter("device_id", deviceId);
        if (!deviceId.empty())
        {
                checkDeviceID = true;
                RCLCPP_INFO(get_logger(), "Found device ID parameter: %s.", deviceId.c_str());
        }

	// Check if scanning is enabled
	bool scan_for_devices = false;
	get_parameter("scan_for_devices", scan_for_devices);

	if (!scan_for_devices){
                // Read baudrate parameter
                int baudrateParam = 0;
                get_parameter("baudrate", baudrateParam);
                RCLCPP_INFO(get_logger(), "Found baudrate parameter: %d", baudrateParam);
                baudrate = XsBaud::numericToRate(baudrateParam);

		// Read port parameter
		std::string portName;
		get_parameter("port", portName);
		RCLCPP_INFO(get_logger(), "Found port name parameter: %s", portName.c_str());
		mtPort = XsPortInfo(portName, baudrate);

                // Try to connect
		RCLCPP_INFO(get_logger(), "Scanning port %s ...", portName.c_str());
		if (!XsScanner::scanPort(mtPort, baudrate))
			return handleError("No MTi device found. Verify port and baudrate.");
		if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
			return handleError("No MTi device found with matching device ID.");
	}
	else
	{
		RCLCPP_INFO(get_logger(), "Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found.");

	RCLCPP_INFO(get_logger(), "Found a device with ID: %s @ port: %s, baudrate: %d.", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	RCLCPP_INFO(get_logger(), "Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port.");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	RCLCPP_INFO(get_logger(), "Device: %s (HW: %s), with ID: %s, using firmware: '%s' was opened.",
                    m_device->productCode().toStdString().c_str(), m_device->hardwareVersion().toString().c_str(), m_device->deviceId().toString().c_str(), m_device->firmwareVersion().toString().c_str());

	m_device->addCallbackHandler(&m_xdaCallback);

        // Check if current baudrate differs from desired and apply a new one if necessary
        XsBaudRate config_baudrate = XBR_Invalid;
        int config_baudrateParam = 0;
        if (get_parameter("config_baudrate", config_baudrateParam))
        {
                RCLCPP_INFO(get_logger(), "Found config_baudrate parameter: %d.", config_baudrateParam);
                config_baudrate = XsBaud::numericToRate(config_baudrateParam);
        }
        if (config_baudrate != XBR_Invalid)
        {
                if (m_device->baudRate() != config_baudrate)
                {
                        RCLCPP_INFO(get_logger(), "Detected baudrate: %d, desired baudrate: %d.", XsBaud::rateToNumeric(m_device->baudRate()), XsBaud::rateToNumeric(config_baudrate));
                        RCLCPP_INFO(get_logger(), "Configuring baudrate to: %d.", XsBaud::rateToNumeric(config_baudrate));
                        if (!m_device->setSerialBaudRate(config_baudrate))
                                return handleError("Could not configure baudate.");
                        if (!m_device->reset())
                                return handleError("Could not reset device.");
                }
        }

	return true;
}

bool XdaInterface::configureDevice()
{
	const auto profiles = m_device->availableOnboardFilterProfiles();
	RCLCPP_INFO(get_logger(), "Supported device profiles:");
	for (const auto& profile : profiles)
	{
		RCLCPP_INFO(get_logger(), " - %s", profile.label());
	}

	const auto current_profile = m_device->onboardFilterProfile();
	RCLCPP_INFO(get_logger(), "Profile in use: %s", current_profile.label());

	std::string selected_profile;
	if (get_parameter("onboard_filter_profile", selected_profile) && !selected_profile.empty())
	{
		RCLCPP_INFO(get_logger(), "Found filter profile parameter: %s.", selected_profile.c_str());

		if (current_profile.label() == selected_profile)
		{
			RCLCPP_INFO(get_logger(), "Matching onboard filter profile already set.");
		}
		else
		{
			RCLCPP_INFO(get_logger(), "Selecting onboard filter profile: %s.", selected_profile.c_str());
			if (!m_device->setOnboardFilterProfile(selected_profile))
				return handleError("Could not set onboard filter profile.");
		}
	}

	const auto current_output_configuration = m_device->outputConfiguration();
	RCLCPP_INFO(get_logger(), "Currently configured output configuration:");
        RCLCPP_INFO(get_logger(), " - DATA_TYPE|DATA_FORMAT|FREQUENCY");
	for (const auto& cfg : current_output_configuration)
        {
		RCLCPP_INFO(get_logger(), " - %s|%s|%d", get_xs_data_identifier_name(cfg.m_dataIdentifier).c_str(), get_xs_format_identifier_name(cfg.m_dataIdentifier).c_str(), cfg.m_frequency);
	}

	std::vector<std::string> output_configuration;
	if (get_parameter("output_configuration", output_configuration) && !output_configuration.empty() && !output_configuration.front().empty())
	{
		RCLCPP_INFO(get_logger(), "Found output configuration parameter(s):");
                RCLCPP_INFO(get_logger(), " - DATA_TYPE|DATA_FORMAT|FREQUENCY");

		XsOutputConfigurationArray newConfigArray;
		for (const auto& cfg : output_configuration)
		{
                        // Try parsing the config
                        XsDataIdentifier data_identifier;
			int output_frequency;
			if (!parseConfigLine(cfg, data_identifier, output_frequency))
				return handleError("Could not parse line: '" + cfg + "'.");

                        RCLCPP_INFO(get_logger(), " - %s|%s|%d", get_xs_data_identifier_name(data_identifier).c_str(),
                                    get_xs_format_identifier_name(data_identifier).c_str(), output_frequency);

                        // Check if the data type supports the specified frequency
                        const std::vector<int> supported_rates = m_device->supportedUpdateRates(data_identifier);
                        if (std::find(supported_rates.begin(), supported_rates.end(), output_frequency) == std::end(supported_rates))
                        {
                                std::ostringstream oss;
                                oss << "Unsupported frequency for '" << get_xs_data_identifier_name(data_identifier) <<
                                    "', valid values are: ";
                                for (const auto& rate : supported_rates)
                                {
                                        oss << rate << ((rate == supported_rates.back()) ? "." : ", ");
                                }
                                return handleError(oss.str());
                        }

			newConfigArray.push_back(XsOutputConfiguration(data_identifier, output_frequency));
		}

		if (newConfigArray == current_output_configuration)
		{
			RCLCPP_INFO(get_logger(), "Matching output configuration already set.");
		}
		else
		{
			RCLCPP_INFO(get_logger(), "Setting output configuration.");
			if (!m_device->setOutputConfiguration(newConfigArray))
				return handleError("Could not set output configuration.");

                        // Check if the configured data output values were actually enabled
                        //TODO(unsupported-format): some data types (such as XDI_SampleTimeFine) will only actually output if the subformat is
                        //  0 (XDI_SubFormatFloat). The config will, however, still report the value as being normally enabled.
                        //  Find a way to catch this error.
                        for (const auto& config : newConfigArray)
                        {
                                if (!m_device->hasDataEnabled(config.m_dataIdentifier))
                                        return handleError("Could not enable dat output for " +
                                                           get_xs_data_identifier_name(config.m_dataIdentifier) +
                                                           ". Try setting a different DATA_FORMAT.");
                        }
		}
	}

	auto configureAlignmentQuat = [&](const std::string& name)
	{
		const auto frame = (name == "sensor") ? XAF_Sensor : XAF_Local;
		const auto parameterName = (frame == XAF_Sensor) ? "alignment_sensor_quat" : "alignment_local_quat";

		std::vector<XsReal> alignment_quat;
		if (get_parameter(parameterName, alignment_quat) && !alignment_quat.empty())
		{
			RCLCPP_INFO(get_logger(), "Configuration alignment rotation for %s.", parameterName);
			const auto currentAlignmentQuat = m_device->alignmentRotationQuaternion(frame);
			RCLCPP_INFO(get_logger(), " - current alignment quaternion for %s (%d): [%f %f %f %f].",
				parameterName, frame, currentAlignmentQuat.w(), currentAlignmentQuat.x(),
				currentAlignmentQuat.y(), currentAlignmentQuat.z());

			const auto paramAlignmentQuat = XsQuaternion{alignment_quat[0], alignment_quat[1], alignment_quat[2], alignment_quat[3]};
			if (paramAlignmentQuat.isEqual(currentAlignmentQuat, 0.01))
			{
                                RCLCPP_INFO(get_logger(), " - actual and desired are near each other, no action taken.");
			}
			else
			{
                                RCLCPP_INFO(get_logger(), " - desired alignment quaternion for %s (%d): [%f %f %f %f].", parameterName, frame,
                                            paramAlignmentQuat.w(), paramAlignmentQuat.x(), paramAlignmentQuat.y(), paramAlignmentQuat.z());
                                if (!m_device->setAlignmentRotationQuaternion(frame, paramAlignmentQuat))
                                  return handleError("Could not configure alignment quaternion.");
			}
		}

		return true;
	};

	if (!configureAlignmentQuat("local"))
		return handleError("Could not set the local rotation matrix.");
	if (!configureAlignmentQuat("sensor"))
		return handleError("Could not set the sensor rotation matrix.");

        const auto current_option_flags = m_device->deviceOptionFlags();
        RCLCPP_INFO(get_logger(), "Currently enabled option flags:%s", get_xs_all_enabled_flags(current_option_flags).c_str());

        std::vector<std::string> option_flags;
        if (get_parameter("option_flags", option_flags) && !option_flags.empty() && !option_flags.front().empty())
        {
                RCLCPP_INFO(get_logger(), "Found option flag(s):");

                XsDeviceOptionFlag new_flags = XDOF_None;
                for (const auto& flag_str : option_flags)
                {
                        RCLCPP_INFO(get_logger(), " - %s", flag_str.c_str());

                        XsDeviceOptionFlag flag = XDOF_None;
                        if (!get_xs_enabled_flag_by_name(flag_str, flag))
                                return handleError("Invalid option flag '" + flag_str + "'.");

                        new_flags = new_flags | flag;
                }

                if (new_flags == current_option_flags)
                {
                        RCLCPP_INFO(get_logger(), "Matching option flags already set.");
                }
                else
                {
                        RCLCPP_INFO(get_logger(), "Setting option flags.");
                        m_device->setDeviceOptionFlags(XDOF_None, XDOF_All);
                        if (!m_device->setDeviceOptionFlags(new_flags, XDOF_None))
                                return handleError("Could not set option flags, some flags might not be supported by the device.");
                }
        }

        const auto gnss_platform = m_device->gnssPlatform();
        RCLCPP_INFO(get_logger(), "GNSS platform setting currently in use: %s", get_xs_gnss_platform_name(gnss_platform).c_str());

        std::string selected_gnss_platform;
        if (get_parameter("gnss_platform", selected_gnss_platform) && !selected_gnss_platform.empty())
        {
                RCLCPP_INFO(get_logger(), "Found GNSS platform parameter: %s.", selected_gnss_platform.c_str());
                XsGnssPlatform new_gnss_platform;
                if (!get_xs_gnss_platform_by_name(selected_gnss_platform, new_gnss_platform))
                        return handleError("Invalid GNSS platform: '" + selected_gnss_platform + "'.");

                if (gnss_platform == new_gnss_platform)
                {
                        RCLCPP_INFO(get_logger(), "Matching GNSS platform already set.");
                }
                else
                {
                        RCLCPP_INFO(get_logger(), "Setting GNSS platform: %s.", selected_gnss_platform.c_str());
                        if (!m_device->setGnssPlatform(new_gnss_platform))
                                return handleError("Could not set GNSS platform.");
                }
        }

        const int current_location_id = m_device->locationId();
        RCLCPP_INFO(get_logger(), "Currently used location ID: %d", current_location_id);

        int param_location_id = -1;
        if (get_parameter("location_id", param_location_id) && (param_location_id != -1))
        {
                RCLCPP_INFO(get_logger(), "Found location ID parameter: %d.", param_location_id);
                if (param_location_id > UINT16_MAX)
                        return handleError("Invalid location ID parameter: " + std::to_string(param_location_id)
                                           + ", max allowed value is: " + std::to_string(UINT16_MAX) + ".");

                if (current_location_id == param_location_id)
                {
                        RCLCPP_INFO(get_logger(), "Matching location ID already set.");
                }
                else
                {
                        RCLCPP_INFO(get_logger(), "Setting location ID: %d.", param_location_id);
                        if (!m_device->setLocationId(param_location_id))
                                return handleError("Could not set location ID.");
                }
        }

        const XsVector current_position_lla = m_device->initialPositionLLA();
        RCLCPP_INFO(get_logger(), "Currently used initial LLA position: [%f, %f, %f].",
                    current_position_lla[0], current_position_lla[1], current_position_lla[2]);

        std::vector<XsReal> param_position_lla{};
        if (get_parameter("initial_position_lla", param_position_lla) && !param_position_lla.empty())
        {
                RCLCPP_INFO(get_logger(), "Found initial LLA position parameter: [%f, %f, %f].",
                            param_position_lla[0], param_position_lla[1], param_position_lla[2]);

                XsVector desired_position_lla{};
                desired_position_lla.setSize(3);
                desired_position_lla[0] = param_position_lla[0];
                desired_position_lla[1] = param_position_lla[1];
                desired_position_lla[2] = param_position_lla[2];
                if (current_position_lla.isEqual(desired_position_lla, 0.01))
                {
                        RCLCPP_INFO(get_logger(), "Desired and current are near each other, no action taken.");
                }
                else
                {
                        RCLCPP_INFO(get_logger(), "Setting initial LLA position: [%f, %f, %f].",
                                    desired_position_lla[0], desired_position_lla[1], desired_position_lla[2]);
                        if (!m_device->setInitialPositionLLA(desired_position_lla))
                                return handleError("Could not set initial LLA position.");
                }
        }

        const auto time = m_device->utcTime();
        RCLCPP_INFO(get_logger(), "Currently set UTC time on device: %d/%d/%d %d:%d:%d (day/month/year hour/minute/second)",
                    time.m_day, time.m_month, time.m_year, time.m_hour, time.m_minute, time.m_second);

        bool set_utc_time = false;
        if (get_parameter("apply_current_utc_time", set_utc_time) && set_utc_time)
        {
                const auto current_time = XsTimeInfo::currentTime();
                RCLCPP_INFO(get_logger(), "Applying current host machine UTC time (%d/%d/%d %d:%d:%d (day/month/year hour/minute/second)) to device.",
                            current_time.m_day, current_time.m_month, current_time.m_year, current_time.m_hour, current_time.m_minute, current_time.m_second);
                //TODO(time-validity): is this (at least one valid bit) enough?
                if (!m_device->setUtcTime(current_time) || (current_time.m_valid == 0))
                        return handleError("Could not apply UTC time to device.");
        }

        const XsVector current_lever_arm = m_device->gnssLeverArm();
        RCLCPP_INFO(get_logger(), "Currently configured GNSS Lever arm: [%f, %f, %f].",
                    current_lever_arm[0], current_lever_arm[1], current_lever_arm[2]);

        std::vector<XsReal> param_lever_arm{};
        if (get_parameter("gnss_lever_arm", param_lever_arm) && !param_lever_arm.empty())
        {
                RCLCPP_INFO(get_logger(), "Found GNSS Lever arm parameter: [%f, %f, %f].",
                            param_lever_arm[0], param_lever_arm[1], param_lever_arm[2]);

                XsVector desired_lever_arm{};
                desired_lever_arm.setSize(3);
                desired_lever_arm[0] = param_lever_arm[0];
                desired_lever_arm[1] = param_lever_arm[1];
                desired_lever_arm[2] = param_lever_arm[2];
                if (current_lever_arm.isEqual(desired_lever_arm, 0.01))
                {
                        RCLCPP_INFO(get_logger(), "Desired and current are near each other, no action taken.");
                }
                else
                {
                        RCLCPP_INFO(get_logger(), "Setting GNSS Lever arm: [%f, %f, %f].",
                                    desired_lever_arm[0], desired_lever_arm[1], desired_lever_arm[2]);
                        if (!m_device->setGnssLeverArm(desired_lever_arm))
                                return handleError("Could not set GNSS Lever arm.");
                }
        }

        XsIntArray port_config = m_device->portConfiguration();
        const auto current_rtcm_baudcode = static_cast<XsBaudCode>(port_config[2] & 0xFF);
        RCLCPP_INFO(get_logger(), "Currently configured RTCM input baudrate: %d", XsBaud::rateToNumeric(XsBaud::codeToRate(current_rtcm_baudcode)));

        int rtcm_baudrate = -1;
        if (get_parameter("rtcm_baudrate", rtcm_baudrate) && (rtcm_baudrate != -1))
        {
                RCLCPP_INFO(get_logger(), "Found rtcm_baudrate parameter: %d.", rtcm_baudrate);
                const XsBaudCode rtcm_baudcode = XsBaud::rateToCode(XsBaud::numericToRate(rtcm_baudrate));
                if (rtcm_baudcode == XBC_Invalid)
                        return handleError("Invalid RTCM baudrate: " + std::to_string(rtcm_baudrate));

                if (current_rtcm_baudcode == rtcm_baudcode)
                {
                        RCLCPP_INFO(get_logger(), "Matching RTCM input baudrate already set.");
                }
                else
                {
                        RCLCPP_INFO(get_logger(), "Setting RTCM input baudrate to: %d.", XsBaud::rateToNumeric(XsBaud::codeToRate(rtcm_baudcode)));
                        port_config[2] = port_config[2] & static_cast<XsIntArray::value_type>(0xFFFFFF00);
                        port_config[2] = port_config[2] | static_cast<XsIntArray::value_type>(rtcm_baudcode);

                        if (!m_device->setPortConfiguration(port_config))
                                return handleError("Could not configure RTCM input baudrate.");
                }
        }

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

        RCLCPP_INFO(get_logger(), "Configuring ...");
	if (!m_device->gotoConfig())
		return handleError("Could not go to config.");

	if (!configureDevice())
		return handleError("Could not not apply the custom device configuration.");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	RCLCPP_INFO(get_logger(), "Measuring ...");
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	bool enable_logging = false;
	get_parameter("enable_logging", enable_logging);

	if (enable_logging)
	{
		std::string logFile;
		if (get_parameter("log_file", logFile))
		{
			if (m_device->createLogFile(logFile) != XRV_OK)
				return handleError("Failed to create a log file! (" + logFile + ")");
			else
				RCLCPP_INFO(get_logger(), "Created a log file: %s", logFile.c_str());

			RCLCPP_INFO(get_logger(), "Recording to %s ...", logFile.c_str());
			if (!m_device->startRecording())
				return handleError("Could not start recording");
		}
	}

        bool reset_heading = false, reset_inclination = false;
        get_parameter("reset_heading", reset_heading);
        get_parameter("reset_inclination", reset_inclination);
        if (reset_heading && reset_inclination)
        {
                RCLCPP_INFO(get_logger(), "Performing orientation reset for heading and inclination.");
                if (!m_device->resetOrientation(XsResetMethod::XRM_Alignment))
                        return handleError("Could not perform orientation reset.");
        }
        else if (reset_heading)
        {
                RCLCPP_INFO(get_logger(), "Performing orientation reset for heading.");
                if (!m_device->resetOrientation(XsResetMethod::XRM_Heading))
                        return handleError("Could not perform orientation reset.");
        }
        else if (reset_inclination)
        {
                RCLCPP_INFO(get_logger(), "Performing orientation reset for inclination.");
                if (!m_device->resetOrientation(XsResetMethod::XRM_Inclination))
                        return handleError("Could not perform orientation reset.");
        }

        createServices();

	return true;
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(std::unique_ptr<PacketCallback> cb)
{
	m_callbacks.push_back(std::move(cb));
}

bool XdaInterface::handleError(const std::string& error)
{
	RCLCPP_ERROR(get_logger(), "%s", error.c_str());
	return false;
}

void XdaInterface::declareCommonParameters()
{
	// Declare ROS parameters common to all the publishers
	std::string frame_id = DEFAULT_FRAME_ID;
	declare_parameter("frame_id", frame_id);

	int pub_queue_size = 5;
	declare_parameter("publisher_queue_size", pub_queue_size);

	bool should_publish = true;
	declare_parameter("pub_imu", should_publish);
	declare_parameter("pub_quaternion", should_publish);
	declare_parameter("pub_acceleration", should_publish);
	declare_parameter("pub_angular_velocity", should_publish);
	declare_parameter("pub_mag", should_publish);
	declare_parameter("pub_dq", should_publish);
	declare_parameter("pub_dv", should_publish);
	declare_parameter("pub_sampletime", should_publish);
	declare_parameter("pub_temperature", should_publish);
	declare_parameter("pub_pressure", should_publish);
	declare_parameter("pub_gnss", should_publish);
	declare_parameter("pub_twist", should_publish);
	declare_parameter("pub_free_acceleration", should_publish);
	declare_parameter("pub_transform", should_publish);
	declare_parameter("pub_positionLLA", should_publish);
	declare_parameter("pub_velocity", should_publish);

	declare_parameter("scan_for_devices", true);
	declare_parameter("device_id", "");
	declare_parameter("port", "");
	declare_parameter("baudrate", XsBaud::rateToNumeric(XBR_Invalid));

	declare_parameter<std::string>("onboard_filter_profile", "");
	declare_parameter<std::vector<std::string>>("output_configuration", {});
	declare_parameter("config_baudrate", XsBaud::rateToNumeric(XBR_Invalid));
        declare_parameter("rtcm_baudrate", XsBaud::rateToNumeric(XBR_Invalid));
	declare_parameter<std::vector<XsReal>>("alignment_local_quat", {});
	declare_parameter<std::vector<XsReal>>("alignment_sensor_quat", {});
        declare_parameter<std::vector<std::string>>("option_flags", {});
        declare_parameter("reset_heading", false);
        declare_parameter("reset_inclination", false);
        declare_parameter<std::string>("gnss_platform", "");
        declare_parameter("location_id", -1);
        declare_parameter<std::vector<XsReal>>("initial_position_lla", {});
        declare_parameter("apply_current_utc_time", false);
        declare_parameter<std::vector<XsReal>>("gnss_lever_arm", {});

	declare_parameter("enable_logging", false);
	declare_parameter("log_file", "log.mtb");
}

void XdaInterface::createServices()
{
        rclcpp::Node& node = *this;
        m_services.push_back(std::make_unique<OrientationReset>(node, m_device));
}
