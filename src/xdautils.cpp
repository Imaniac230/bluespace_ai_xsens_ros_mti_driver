#include "xdautils.h"

#include <algorithm>
#include <sstream>

std::string get_xs_data_identifier_name(const XsDataIdentifier& identifier)
{
    const XsDataIdentifier masked_identifier = identifier & XDI_FullTypeMask;
    switch (masked_identifier) {
        case XDI_TemperatureGroup: return "TemperatureGroup";
        case XDI_Temperature: return "Temperature";

        case XDI_TimestampGroup: return "TimestampGroup";
        case XDI_UtcTime: return "UtcTime";
        case XDI_PacketCounter: return "PacketCounter";
        case XDI_Itow: return "Itow";
        case XDI_GnssAge: return "GnssAge";
        case XDI_PressureAge: return "PressureAge";
        case XDI_SampleTimeFine: return "SampleTimeFine";
        case XDI_SampleTimeCoarse: return "SampleTimeCoarse";
        case XDI_FrameRange: return "FrameRange";
        case XDI_PacketCounter8: return "PacketCounter8";
        case XDI_SampleTime64: return "SampleTime64";

        case XDI_OrientationGroup: return "OrientationGroup";
        case XDI_Quaternion: return "Quaternion";
        case XDI_RotationMatrix: return "RotationMatrix";
        case XDI_EulerAngles: return "EulerAngles";

        case XDI_PressureGroup: return "PressureGroup";
        case XDI_BaroPressure: return "BaroPressure";

        case XDI_AccelerationGroup: return "AccelerationGroup";
        case XDI_DeltaV: return "DeltaV";
        case XDI_Acceleration: return "Acceleration";
        case XDI_FreeAcceleration: return "FreeAcceleration";
        case XDI_AccelerationHR: return "AccelerationHR";

        case XDI_IndicationGroup: return "IndicationGroup";
        case XDI_TriggerIn1: return "TriggerIn1";
        case XDI_TriggerIn2: return "TriggerIn2";
        case XDI_TriggerIn3: return "TriggerIn3";

        case XDI_PositionGroup: return "PositionGroup";
        case XDI_AltitudeMsl: return "AltitudeMsl";
        case XDI_AltitudeEllipsoid: return "AltitudeEllipsoid";
        case XDI_PositionEcef: return "PositionEcef";
        case XDI_LatLon: return "LatLon";

        case XDI_GnssGroup: return "GnssGroup";
        case XDI_GnssPvtData: return "GnssPvtData";
        case XDI_GnssSatInfo: return "GnssSatInfo";
        case XDI_GnssPvtPulse: return "GnssPvtPulse";

        case XDI_AngularVelocityGroup: return "AngularVelocityGroup";
        case XDI_RateOfTurn: return "RateOfTurn";
        case XDI_DeltaQ: return "DeltaQ";
        case XDI_RateOfTurnHR: return "RateOfTurnHR";

        case XDI_RawSensorGroup: return "RawSensorGroup";
        case XDI_RawAccGyrMagTemp: return "RawAccGyrMagTemp";
        case XDI_RawGyroTemp: return "RawGyroTemp";
        case XDI_RawAcc: return "RawAcc";
        case XDI_RawGyr: return "RawGyr";
        case XDI_RawMag: return "RawMag";
        case XDI_RawDeltaQ: return "RawDeltaQ";
        case XDI_RawDeltaV: return "RawDeltaV";
        case XDI_RawBlob: return "RawBlob";

        case XDI_AnalogInGroup: return "AnalogInGroup";
        case XDI_AnalogIn1: return "AnalogIn1";
        case XDI_AnalogIn2: return "AnalogIn2";

        case XDI_MagneticGroup: return "MagneticGroup";
        case XDI_MagneticField: return "MagneticField";
        case XDI_MagneticFieldCorrected: return "MagneticFieldCorrected";

        case XDI_SnapshotGroup: return "SnapshotGroup";
        case XDI_AwindaSnapshot: return "AwindaSnapshot";
        case XDI_FullSnapshot: return "FullSnapshot";
        case XDI_GloveSnapshotLeft: return "GloveSnapshotLeft";
        case XDI_GloveSnapshotRight: return "GloveSnapshotRight";

        case XDI_GloveDataGroup: return "GloveDataGroup";
        case XDI_GloveDataLeft: return "GloveDataLeft";
        case XDI_GloveDataRight: return "GloveDataRight";

        case XDI_VelocityGroup: return "VelocityGroup";
        case XDI_VelocityXYZ: return "VelocityXYZ";

        case XDI_StatusGroup: return "StatusGroup";
        case XDI_StatusByte: return "StatusByte";
        case XDI_StatusWord: return "StatusWord";
        case XDI_Rssi: return "Rssi";
        case XDI_DeviceId: return "DeviceId";
        case XDI_LocationId: return "LocationId";

        default: return "Unknown";
    }
}

bool get_xs_data_identifier_by_name(const std::string& name, XsDataIdentifier& identifier)
{
    std::map<std::string, XsDataIdentifier> name_mapping;
    name_mapping["TemperatureGroup"] = XDI_TemperatureGroup;
    name_mapping["Temperature"] = XDI_Temperature;

    name_mapping["TimestampGroup"] = XDI_TimestampGroup;
    name_mapping["UtcTime"] = XDI_UtcTime;
    name_mapping["PacketCounter"] = XDI_PacketCounter;
    name_mapping["Itow"] = XDI_Itow;
    name_mapping["GnssAge"] = XDI_GnssAge;
    name_mapping["PressureAge"] = XDI_PressureAge;
    name_mapping["SampleTimeFine"] = XDI_SampleTimeFine;
    name_mapping["SampleTimeCoarse"] = XDI_SampleTimeCoarse;
    name_mapping["FrameRange"] = XDI_FrameRange;
    name_mapping["PacketCounter8"] = XDI_PacketCounter8;
    name_mapping["SampleTime64"] = XDI_SampleTime64;

    name_mapping["OrientationGroup"] = XDI_OrientationGroup;
    name_mapping["Quaternion"] = XDI_Quaternion;
    name_mapping["RotationMatrix"] = XDI_RotationMatrix;
    name_mapping["EulerAngles"] = XDI_EulerAngles;

    name_mapping["PressureGroup"] = XDI_PressureGroup;
    name_mapping["BaroPressure"] = XDI_BaroPressure;

    name_mapping["AccelerationGroup"] = XDI_AccelerationGroup;
    name_mapping["DeltaV"] = XDI_DeltaV;
    name_mapping["Acceleration"] = XDI_Acceleration;
    name_mapping["FreeAcceleration"] = XDI_FreeAcceleration;
    name_mapping["AccelerationHR"] = XDI_AccelerationHR;

    name_mapping["IndicationGroup"] = XDI_IndicationGroup;
    name_mapping["TriggerIn1"] = XDI_TriggerIn1;
    name_mapping["TriggerIn2"] = XDI_TriggerIn2;
    name_mapping["TriggerIn3"] = XDI_TriggerIn3;

    name_mapping["PositionGroup"] = XDI_PositionGroup;
    name_mapping["AltitudeMsl"] = XDI_AltitudeMsl;
    name_mapping["AltitudeEllipsoid"] = XDI_AltitudeEllipsoid;
    name_mapping["PositionEcef"] = XDI_PositionEcef;
    name_mapping["LatLon"] = XDI_LatLon;

    name_mapping["GnssGroup"] = XDI_GnssGroup;
    name_mapping["GnssPvtData"] = XDI_GnssPvtData;
    name_mapping["GnssSatInfo"] = XDI_GnssSatInfo;
    name_mapping["GnssPvtPulse"] = XDI_GnssPvtPulse;

    name_mapping["AngularVelocityGroup"] = XDI_AngularVelocityGroup;
    name_mapping["RateOfTurn"] = XDI_RateOfTurn;
    name_mapping["DeltaQ"] = XDI_DeltaQ;
    name_mapping["RateOfTurnHR"] = XDI_RateOfTurnHR;

    name_mapping["RawSensorGroup"] = XDI_RawSensorGroup;
    name_mapping["RawAccGyrMagTemp"] = XDI_RawAccGyrMagTemp;
    name_mapping["RawGyroTemp"] = XDI_RawGyroTemp;
    name_mapping["RawAcc"] = XDI_RawAcc;
    name_mapping["RawGyr"] = XDI_RawGyr;
    name_mapping["RawMag"] = XDI_RawMag;
    name_mapping["RawDeltaQ"] = XDI_RawDeltaQ;
    name_mapping["RawDeltaV"] = XDI_RawDeltaV;
    name_mapping["RawBlob"] = XDI_RawBlob;

    name_mapping["AnalogInGroup"] = XDI_AnalogInGroup;
    name_mapping["AnalogIn1"] = XDI_AnalogIn1;
    name_mapping["AnalogIn2"] = XDI_AnalogIn2;

    name_mapping["MagneticGroup"] = XDI_MagneticGroup;
    name_mapping["MagneticField"] = XDI_MagneticField;
    name_mapping["MagneticFieldCorrected"] = XDI_MagneticFieldCorrected;

    name_mapping["SnapshotGroup"] = XDI_SnapshotGroup;
    name_mapping["AwindaSnapshot"] = XDI_AwindaSnapshot;
    name_mapping["FullSnapshot"] = XDI_FullSnapshot;
    name_mapping["GloveSnapshotLeft"] = XDI_GloveSnapshotLeft;
    name_mapping["GloveSnapshotRight"] = XDI_GloveSnapshotRight;

    name_mapping["GloveDataGroup"] = XDI_GloveDataGroup;
    name_mapping["GloveDataLeft"] = XDI_GloveDataLeft;
    name_mapping["GloveDataRight"] = XDI_GloveDataRight;

    name_mapping["VelocityGroup"] = XDI_VelocityGroup;
    name_mapping["VelocityXYZ"] = XDI_VelocityXYZ;

    name_mapping["StatusGroup"] = XDI_StatusGroup;
    name_mapping["StatusByte"] = XDI_StatusByte;
    name_mapping["StatusWord"] = XDI_StatusWord;
    name_mapping["Rssi"] = XDI_Rssi;
    name_mapping["DeviceId"] = XDI_DeviceId;
    name_mapping["LocationId"] = XDI_LocationId;

    return get_xs_value(name, name_mapping, identifier);
}

bool get_xs_format_identifier_by_name(const std::string& format_str, XsDataIdentifier& identifier)
{
    std::string lower_format_str = format_str;
    std::transform(format_str.begin(), format_str.end(), lower_format_str.begin(), ::tolower);

    std::map<std::string, XsDataIdentifier> format_mapping;
    format_mapping["float"] = XDI_SubFormatFloat;
    format_mapping["fp1220"] = XDI_SubFormatFp1220;
    format_mapping["fp1632"] = XDI_SubFormatFp1632;
    format_mapping["double"] = XDI_SubFormatDouble;

    return get_xs_value(lower_format_str, format_mapping, identifier);
}

std::string get_xs_format_identifier_name(const XsDataIdentifier& identifier)
{
    XsDataIdentifier masked_identifier = identifier & XDI_SubFormatMask;
    switch (masked_identifier)
    {
    case XDI_SubFormatFloat: return "Float";
    case XDI_SubFormatFp1220: return "Fp1220";
    case XDI_SubFormatFp1632: return "Fp1632";
    case XDI_SubFormatDouble: return "Double";

    default: return "Unknown";
    }
}

bool get_xs_enabled_flag_by_name(const std::string& name, XsDeviceOptionFlag& option_flag)
{
    std::map<std::string, XsDeviceOptionFlag> flag_mapping;
    flag_mapping["DisableAutoStore"] = XDOF_DisableAutoStore;
    flag_mapping["DisableAutoMeasurement"] = XDOF_DisableAutoMeasurement;
    flag_mapping["EnableBeidou"] = XDOF_EnableBeidou;
    flag_mapping["DisableGps"] = XDOF_DisableGps;
    flag_mapping["EnableAhs"] = XDOF_EnableAhs;
    flag_mapping["EnableOrientationSmoother"] = XDOF_EnableOrientationSmoother;
    flag_mapping["EnableConfigurableBusId"] = XDOF_EnableConfigurableBusId;
    flag_mapping["EnableInrunCompassCalibration"] = XDOF_EnableInrunCompassCalibration;
    flag_mapping["DisableSleepMode"] = XDOF_DisableSleepMode;
    flag_mapping["EnableConfigMessageAtStartup"] = XDOF_EnableConfigMessageAtStartup;
    flag_mapping["EnableColdFilterResets"] = XDOF_EnableColdFilterResets;
    flag_mapping["EnablePositionVelocitySmoother"] = XDOF_EnablePositionVelocitySmoother;
    flag_mapping["EnableContinuousZRU"] = XDOF_EnableContinuousZRU;

    flag_mapping["None"] = XDOF_None;
    flag_mapping["All"] = XDOF_All;

    return get_xs_value(name, flag_mapping, option_flag);
}

std::string get_xs_all_enabled_flags(const XsDeviceOptionFlag& option_flag)
{
    std::ostringstream oss;

    if (option_flag == XDOF_All)
    {
      oss << "\n - All";
      return oss.str();
    }

    if (option_flag == XDOF_None)
    {
      oss << "\n - None";
      return oss.str();
    }

    if (option_flag & XDOF_DisableAutoStore)
      oss << "\n - DisableAutoStore";
    if (option_flag & XDOF_DisableAutoMeasurement)
      oss << "\n - DisableAutoMeasurement";
    if (option_flag & XDOF_EnableBeidou)
      oss << "\n - EnableBeidou";
    if (option_flag & XDOF_DisableGps)
      oss << "\n - DisableGps";
    if (option_flag & XDOF_EnableAhs)
      oss << "\n - EnableAhs";
    if (option_flag & XDOF_EnableOrientationSmoother)
      oss << "\n - EnableOrientationSmoother";
    if (option_flag & XDOF_EnableConfigurableBusId)
      oss << "\n - EnableConfigurableBusId";
    if (option_flag & XDOF_EnableInrunCompassCalibration)
      oss << "\n - EnableInrunCompassCalibration";
    if (option_flag & XDOF_DisableSleepMode)
      oss << "\n - DisableSleepMode";
    if (option_flag & XDOF_EnableConfigMessageAtStartup)
      oss << "\n - EnableConfigMessageAtStartup";
    if (option_flag & XDOF_EnableColdFilterResets)
      oss << "\n - EnableColdFilterResets";
    if (option_flag & XDOF_EnablePositionVelocitySmoother)
      oss << "\n - EnablePositionVelocitySmoother";
    if (option_flag & XDOF_EnableContinuousZRU)
      oss << "\n - EnableContinuousZRU";

    if (oss.str().empty())
      oss << "\nUnknown";

    return oss.str();
}

std::string get_xs_gnss_platform_name(const XsGnssPlatform& platform)
{
    switch (platform)
    {
    case XGP_Portable: return "Portable";
    case XGP_Stationary: return "Stationary";
    case XGP_Pedestrian: return "Pedestrian";
    case XGP_Automotive: return "Automotive";
    case XGP_AtSea: return "AtSea";
    case XGP_Airborne1g: return "Airborne1g";
    case XGP_Airborne2g: return "Airborne2g";
    case XGP_Airborne4g: return "Airborne4g";
    case XGP_Wrist:return "Wrist";

    default: return "Unknown";
    }
}

bool get_xs_gnss_platform_by_name(const std::string& name, XsGnssPlatform& platform)
{
    std::string lower_platform_str = name;
    std::transform(name.begin(), name.end(), lower_platform_str.begin(), ::tolower);

    std::map<std::string, XsGnssPlatform> platform_mapping;
    platform_mapping["portable"] = XGP_Portable;
    platform_mapping["stationary"] = XGP_Stationary;
    platform_mapping["pedestrian"] = XGP_Pedestrian;
    platform_mapping["automotive"] = XGP_Automotive;
    platform_mapping["atsea"] = XGP_AtSea;
    platform_mapping["airborne1g"] = XGP_Airborne1g;
    platform_mapping["airborne2g"] = XGP_Airborne2g;
    platform_mapping["airborne4g"] = XGP_Airborne4g;
    platform_mapping["wrist"] = XGP_Wrist;

    return get_xs_value(lower_platform_str, platform_mapping, platform);
}

bool parseConfigLine(const std::string& line, XsDataIdentifier& identifier, int& frequency)
{
    std::istringstream stream(line);
    XsDataIdentifier type = XDI_None, format = XDI_None;
    bool format_parsing_success = false;
    int index=0;
    for (std::string token; std::getline(stream, token, '|') && index <= 3; ++index)
    {
        if (get_xs_data_identifier_by_name(token, type))
          continue;
        const bool parsing_result = get_xs_format_identifier_by_name(token, format);
        if (parsing_result)
          {
            format_parsing_success = parsing_result;
            continue;
          }
        try
        {
          std::transform(token.begin(), token.end(), token.begin(), ::tolower);
          frequency = std::stoi(token, nullptr, (token.find("0x") != std::string::npos) ? 16 : 10);
        }
        catch (std::invalid_argument const& err)
        {
          return false;
        }
    }

    if ((type != XDI_None) && format_parsing_success)
        identifier = type | format;
    else
        return false;

    return index == 3;
}

