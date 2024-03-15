//
// Created by user on 15/03/24.
//

#ifndef BLUESPACE_AI_XSENS_MTI_DRIVER_PARSINGTESTCALLBACK_H
#define BLUESPACE_AI_XSENS_MTI_DRIVER_PARSINGTESTCALLBACK_H

#include "packetcallback.h"

#include <chrono>

struct ParsingTestCallback : public PacketCallback
{
  explicit ParsingTestCallback() = default;

  void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
  {
    const auto now =  std::chrono::high_resolution_clock::now().time_since_epoch().count();
    if (packet.containsGnssAge()) {
      const auto gnssAge = packet.gnssAge();
      std::cout << now << " gnss age -> " << static_cast<int>(gnssAge) << std::endl;
    }
    if (packet.containsRawGnssSatInfo()) {
      const auto satInfo = packet.rawGnssSatInfo();
      std::list<std::string> services{};
      std::cout << now << " num of satellites -> " << static_cast<int>(satInfo.m_numSvs) << std::endl;
      for (unsigned i = 0; i < satInfo.m_numSvs; ++i) {
        const auto info = satInfo.m_satInfos[i];
        std::string gnss;
        switch (info.m_gnssId) {
        case 0:
          gnss = "GPS";
          break;
        case 1:
          gnss = "SBAS";
          break;
        case 2:
          gnss = "Galileo";
          break;
        case 3:
          gnss = "Beidou";
          break;
        case 4:
          gnss = "IMES";
          break;
        case 5:
          gnss = "QZSS";
          break;
        case 6:
          gnss = "GLONASS";
          break;
        default:
          gnss = "UNKNOWN";
        }
        std::cout << now << " gnss id " << i << " -> " << gnss << std::endl;
        std::cout << now << " sat id " << i << " -> " << static_cast<int>(info.m_svId) << std::endl;
        std::cout << now << " cno " << i << " -> " << static_cast<int>(info.m_cno) << std::endl;
        std::string quality;
        switch (info.m_flags & XSIF_SignalQualityIndicator_Mask) {
        case XSIF_SignalQualityIndicator_NoSignal:
          quality = "no signal";
          break;
        case XSIF_SignalQualityIndicator_Searching:
          quality = "searching";
          break;
        case XSIF_SignalQualityIndicator_Acquired:
          quality = "acquired";
          break;
        case XSIF_SignalQualityIndicator_Unusable:
          quality = "detected but unusable";
          break;
        case XSIF_SignalQualityIndicator_CodeTimeOk:
          quality = "code locked, time synchronized";
          break;
        case XSIF_SignalQualityIndicator_CodeCarrierTimeOk1:
          quality = "code & carrier 1 locked, time synchronized";
          break;
        case XSIF_SignalQualityIndicator_CodeCarrierTimeOk2:
          quality = "code & carrier 2 locked, time synchronized";
          break;
        case XSIF_SignalQualityIndicator_CodeCarrierTimeOk3:
          quality = "code & carrier 3 locked, time synchronized";
          break;
        }
        std::cout << now << " flags " << i << " -> signal quality: " << quality << std::endl;
        std::cout << now << " flags " << i << " -> is used for nav: " << ((info.m_flags & XSIF_UsedForNavigation_Used) ? "YES" : "NO") << std::endl;
        if (info.m_flags & XSIF_UsedForNavigation_Used) services.push_back(gnss);
        std::string health;
        switch (info.m_flags & XSIF_HealthFlag_Mask) {
        case XSIF_HealthFlag_Unknown:
          health = "unknown";
          break;
        case XSIF_HealthFlag_Healthy:
          health = "healthy";
          break;
        case XSIF_HealthFlag_Unhealthy:
          health = "unhealthy";
          break;
        }
        std::cout << now << " flags " << i << " -> satellite health: " << health << std::endl;
        std::cout << now << " flags " << i << " -> differential corrections available: " << ((info.m_flags & XSIF_Differential_Available) ? "YES" : "NO") << std::endl;
      }
      std::cout << now << " used services: ";
      for (const auto& service : services) {
        std::cout << service << ", ";
      }
      std::cout << std::endl;
    }
    if (packet.containsRawGnssPvtData()) {
      const auto pvtData = packet.rawGnssPvtData();
      std::string fix;
      switch (pvtData.m_fixType) {
      case XPDQI_NoFix:
        fix = "none";
        break;
      case XPDQI_DeadReckiningOnly:
        fix = "dead reckoning";
        break;
      case XPDQI_2DFix:
        fix = "2D";
        break;
      case XPDQI_3DFix:
        fix = "3D";
        break;
      case XPDQI_GnssAndDeadReck:
        fix = "gnss and dead reckoning";
        break;
      case XPDQI_TimeOnlyFix:
        fix = "time only";
        break;
      }
      std::cout << now << " fix type -> " << fix << std::endl;
      std::cout << now << " flags -> fix valid: " << ((pvtData.m_flags & 0x01) ? "YES" : "NO") << std::endl;
      std::cout << now << " flags -> differential corrections applied: " << ((pvtData.m_flags & 0x02) ? "YES" : "NO") << std::endl;
      std::cout << now << " flags -> heading valid: " << ((pvtData.m_flags & 0x32) ? "YES" : "NO") << std::endl;
      std::cout << now << " num of used satellites -> " << static_cast<int>(pvtData.m_numSv) << std::endl;
    }
  }
};

#endif // BLUESPACE_AI_XSENS_MTI_DRIVER_PARSINGTESTCALLBACK_H
