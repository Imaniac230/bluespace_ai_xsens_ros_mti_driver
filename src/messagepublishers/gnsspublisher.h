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

#ifndef GNSSPUBLISHER_H
#define GNSSPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

enum class GnssId : uint8_t
{
    Gps = 0,
    Sbas = 1,
    Galileo = 2,
    BeiDou = 3,
    Imes = 4,
    Qzss = 5,
    Glonass = 6,
};


struct GnssPublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    uint16_t used_services = 0;

    explicit GnssPublisher(rclcpp::Node &node)
    {
        int pub_queue_size = 5;
        node.get_parameter("publisher_queue_size", pub_queue_size);
        pub = node.create_publisher<sensor_msgs::msg::NavSatFix>("gnss", pub_queue_size);
        node.get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (packet.containsRawGnssSatInfo())
        {
            const XsRawGnssSatInfo info = packet.rawGnssSatInfo();
            used_services = 0;
            for (unsigned i = 0; i < info.m_numSvs; ++i)
            {
                if (info.m_satInfos[i].m_flags & XSIF_UsedForNavigation_Used)
                {
                    switch (static_cast<GnssId>(info.m_satInfos[i].m_gnssId))
                    {
                    case GnssId::Gps:
                        used_services |=
                            sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                        break;
                    case GnssId::Glonass:
                        used_services |=
                            sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
                        break;
                    case GnssId::BeiDou:
                        used_services |=
                            sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
                        break;
                    case GnssId::Galileo:
                        used_services |=
                            sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        if (packet.containsRawGnssPvtData())
        {
            sensor_msgs::msg::NavSatFix msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            const XsRawGnssPvtData gnss = packet.rawGnssPvtData();

            msg.latitude = (double)gnss.m_lat * 1e-7;
            msg.longitude = (double)gnss.m_lon * 1e-7;
            msg.altitude = (double)gnss.m_height * 1e-3;
            // Position covariance [m^2], ENU
            double sh = ((double)gnss.m_hAcc * 1e-3);
            double sv = ((double)gnss.m_vAcc * 1e-3);
            msg.position_covariance = {sh * sh, 0, 0, 0, sh * sh, 0, 0, 0, sv * sv};
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            switch (gnss.m_fixType)
            {
            case XPDQI_2DFix: // fall through
            case XPDQI_3DFix: // fall through
            case XPDQI_GnssAndDeadReck:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                break;
            default:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }
            msg.status.service = used_services;
            used_services = 0;

            pub->publish(msg);
        }
    }
};

#endif
