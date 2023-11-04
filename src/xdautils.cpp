#include "xdautils.h"

#include <algorithm>
#include <sstream>

std::string get_xs_data_identifier_name(const XsDataIdentifier& identifier)
{
    const XsDataIdentifier masked_identifier = identifier & XDI_FullTypeMask;
    switch (masked_identifier) {
        case XDI_TemperatureGroup: return "XDI_TemperatureGroup";
        case XDI_Temperature: return "XDI_Temperature";

        case XDI_TimestampGroup: return "XDI_TimestampGroup";
        case XDI_UtcTime: return "XDI_UtcTime";
        case XDI_PacketCounter: return "XDI_PacketCounter";
        case XDI_Itow: return "XDI_Itow";
        case XDI_GnssAge: return "XDI_GnssAge";
        case XDI_PressureAge: return "XDI_PressureAge";
        case XDI_SampleTimeFine: return "XDI_SampleTimeFine";
        case XDI_SampleTimeCoarse: return "XDI_SampleTimeCoarse";
        case XDI_FrameRange: return "XDI_FrameRange";
        case XDI_PacketCounter8: return "XDI_PacketCounter8";
        case XDI_SampleTime64: return "XDI_SampleTime64";

        case XDI_OrientationGroup: return "XDI_OrientationGroup";
        case XDI_Quaternion: return "XDI_Quaternion";
        case XDI_RotationMatrix: return "XDI_RotationMatrix";
        case XDI_EulerAngles: return "XDI_EulerAngles";

        case XDI_PressureGroup: return "XDI_PressureGroup";
        case XDI_BaroPressure: return "XDI_BaroPressure";

        case XDI_AccelerationGroup: return "XDI_AccelerationGroup";
        case XDI_DeltaV: return "XDI_DeltaV";
        case XDI_Acceleration: return "XDI_Acceleration";
        case XDI_FreeAcceleration: return "XDI_FreeAcceleration";
        case XDI_AccelerationHR: return "XDI_AccelerationHR";

        case XDI_IndicationGroup: return "XDI_IndicationGroup";
        case XDI_TriggerIn1: return "XDI_TriggerIn1";
        case XDI_TriggerIn2: return "XDI_TriggerIn2";
        case XDI_TriggerIn3: return "XDI_TriggerIn3";

        case XDI_PositionGroup: return "XDI_PositionGroup";
        case XDI_AltitudeMsl: return "XDI_AltitudeMsl";
        case XDI_AltitudeEllipsoid: return "XDI_AltitudeEllipsoid";
        case XDI_PositionEcef: return "XDI_PositionEcef";
        case XDI_LatLon: return "XDI_LatLon";

        case XDI_GnssGroup: return "XDI_GnssGroup";
        case XDI_GnssPvtData: return "XDI_GnssPvtData";
        case XDI_GnssSatInfo: return "XDI_GnssSatInfo";
        case XDI_GnssPvtPulse: return "XDI_GnssPvtPulse";

        case XDI_AngularVelocityGroup: return "XDI_AngularVelocityGroup";
        case XDI_RateOfTurn: return "XDI_RateOfTurn";
        case XDI_DeltaQ: return "XDI_DeltaQ";
        case XDI_RateOfTurnHR: return "XDI_RateOfTurnHR";

        case XDI_RawSensorGroup: return "XDI_RawSensorGroup";
        case XDI_RawAccGyrMagTemp: return "XDI_RawAccGyrMagTemp";
        case XDI_RawGyroTemp: return "XDI_RawGyroTemp";
        case XDI_RawAcc: return "XDI_RawAcc";
        case XDI_RawGyr: return "XDI_RawGyr";
        case XDI_RawMag: return "XDI_RawMag";
        case XDI_RawDeltaQ: return "XDI_RawDeltaQ";
        case XDI_RawDeltaV: return "XDI_RawDeltaV";
        case XDI_RawBlob: return "XDI_RawBlob";

        case XDI_AnalogInGroup: return "XDI_AnalogInGroup";
        case XDI_AnalogIn1: return "XDI_AnalogIn1";
        case XDI_AnalogIn2: return "XDI_AnalogIn2";

        case XDI_MagneticGroup: return "XDI_MagneticGroup";
        case XDI_MagneticField: return "XDI_MagneticField";
        case XDI_MagneticFieldCorrected: return "XDI_MagneticFieldCorrected";

        case XDI_SnapshotGroup: return "XDI_SnapshotGroup";
        case XDI_AwindaSnapshot: return "XDI_AwindaSnapshot";
        case XDI_FullSnapshot: return "XDI_FullSnapshot";
        case XDI_GloveSnapshotLeft: return "XDI_GloveSnapshotLeft";
        case XDI_GloveSnapshotRight: return "XDI_GloveSnapshotRight";

        case XDI_GloveDataGroup: return "XDI_GloveDataGroup";
        case XDI_GloveDataLeft: return "XDI_GloveDataLeft";
        case XDI_GloveDataRight: return "XDI_GloveDataRight";

        case XDI_VelocityGroup: return "XDI_VelocityGroup";
        case XDI_VelocityXYZ: return "XDI_VelocityXYZ";

        case XDI_StatusGroup: return "XDI_StatusGroup";
        case XDI_StatusByte: return "XDI_StatusByte";
        case XDI_StatusWord: return "XDI_StatusWord";
        case XDI_Rssi: return "XDI_Rssi";
        case XDI_DeviceId: return "XDI_DeviceId";
        case XDI_LocationId: return "XDI_LocationId";

        default: return "Unknown";
    }
}

bool get_xs_data_identifier_by_name(const std::string& name, XsDataIdentifier& identifier)
{
    std::map<std::string, XsDataIdentifier> name_mapping;
    name_mapping["XDI_TemperatureGroup"] = XDI_TemperatureGroup;
    name_mapping["XDI_Temperature"] = XDI_Temperature;

    name_mapping["XDI_TimestampGroup"] = XDI_TimestampGroup;
    name_mapping["XDI_UtcTime"] = XDI_UtcTime;
    name_mapping["XDI_PacketCounter"] = XDI_PacketCounter;
    name_mapping["XDI_Itow"] = XDI_Itow;
    name_mapping["XDI_GnssAge"] = XDI_GnssAge;
    name_mapping["XDI_PressureAge"] = XDI_PressureAge;
    name_mapping["XDI_SampleTimeFine"] = XDI_SampleTimeFine;
    name_mapping["XDI_SampleTimeCoarse"] = XDI_SampleTimeCoarse;
    name_mapping["XDI_FrameRange"] = XDI_FrameRange;
    name_mapping["XDI_PacketCounter8"] = XDI_PacketCounter8;
    name_mapping["XDI_SampleTime64"] = XDI_SampleTime64;

    name_mapping["XDI_OrientationGroup"] = XDI_OrientationGroup;
    name_mapping["XDI_Quaternion"] = XDI_Quaternion;
    name_mapping["XDI_RotationMatrix"] = XDI_RotationMatrix;
    name_mapping["XDI_EulerAngles"] = XDI_EulerAngles;

    name_mapping["XDI_PressureGroup"] = XDI_PressureGroup;
    name_mapping["XDI_BaroPressure"] = XDI_BaroPressure;

    name_mapping["XDI_AccelerationGroup"] = XDI_AccelerationGroup;
    name_mapping["XDI_DeltaV"] = XDI_DeltaV;
    name_mapping["XDI_Acceleration"] = XDI_Acceleration;
    name_mapping["XDI_FreeAcceleration"] = XDI_FreeAcceleration;
    name_mapping["XDI_AccelerationHR"] = XDI_AccelerationHR;

    name_mapping["XDI_IndicationGroup"] = XDI_IndicationGroup;
    name_mapping["XDI_TriggerIn1"] = XDI_TriggerIn1;
    name_mapping["XDI_TriggerIn2"] = XDI_TriggerIn2;
    name_mapping["XDI_TriggerIn3"] = XDI_TriggerIn3;

    name_mapping["XDI_PositionGroup"] = XDI_PositionGroup;
    name_mapping["XDI_AltitudeMsl"] = XDI_AltitudeMsl;
    name_mapping["XDI_AltitudeEllipsoid"] = XDI_AltitudeEllipsoid;
    name_mapping["XDI_PositionEcef"] = XDI_PositionEcef;
    name_mapping["XDI_LatLon"] = XDI_LatLon;

    name_mapping["XDI_GnssGroup"] = XDI_GnssGroup;
    name_mapping["XDI_GnssPvtData"] = XDI_GnssPvtData;
    name_mapping["XDI_GnssSatInfo"] = XDI_GnssSatInfo;
    name_mapping["XDI_GnssPvtPulse"] = XDI_GnssPvtPulse;

    name_mapping["XDI_AngularVelocityGroup"] = XDI_AngularVelocityGroup;
    name_mapping["XDI_RateOfTurn"] = XDI_RateOfTurn;
    name_mapping["XDI_DeltaQ"] = XDI_DeltaQ;
    name_mapping["XDI_RateOfTurnHR"] = XDI_RateOfTurnHR;

    name_mapping["XDI_RawSensorGroup"] = XDI_RawSensorGroup;
    name_mapping["XDI_RawAccGyrMagTemp"] = XDI_RawAccGyrMagTemp;
    name_mapping["XDI_RawGyroTemp"] = XDI_RawGyroTemp;
    name_mapping["XDI_RawAcc"] = XDI_RawAcc;
    name_mapping["XDI_RawGyr"] = XDI_RawGyr;
    name_mapping["XDI_RawMag"] = XDI_RawMag;
    name_mapping["XDI_RawDeltaQ"] = XDI_RawDeltaQ;
    name_mapping["XDI_RawDeltaV"] = XDI_RawDeltaV;
    name_mapping["XDI_RawBlob"] = XDI_RawBlob;

    name_mapping["XDI_AnalogInGroup"] = XDI_AnalogInGroup;
    name_mapping["XDI_AnalogIn1"] = XDI_AnalogIn1;
    name_mapping["XDI_AnalogIn2"] = XDI_AnalogIn2;

    name_mapping["XDI_MagneticGroup"] = XDI_MagneticGroup;
    name_mapping["XDI_MagneticField"] = XDI_MagneticField;
    name_mapping["XDI_MagneticFieldCorrected"] = XDI_MagneticFieldCorrected;

    name_mapping["XDI_SnapshotGroup"] = XDI_SnapshotGroup;
    name_mapping["XDI_AwindaSnapshot"] = XDI_AwindaSnapshot;
    name_mapping["XDI_FullSnapshot"] = XDI_FullSnapshot;
    name_mapping["XDI_GloveSnapshotLeft"] = XDI_GloveSnapshotLeft;
    name_mapping["XDI_GloveSnapshotRight"] = XDI_GloveSnapshotRight;

    name_mapping["XDI_GloveDataGroup"] = XDI_GloveDataGroup;
    name_mapping["XDI_GloveDataLeft"] = XDI_GloveDataLeft;
    name_mapping["XDI_GloveDataRight"] = XDI_GloveDataRight;

    name_mapping["XDI_VelocityGroup"] = XDI_VelocityGroup;
    name_mapping["XDI_VelocityXYZ"] = XDI_VelocityXYZ;

    name_mapping["XDI_StatusGroup"] = XDI_StatusGroup;
    name_mapping["XDI_StatusByte"] = XDI_StatusByte;
    name_mapping["XDI_StatusWord"] = XDI_StatusWord;
    name_mapping["XDI_Rssi"] = XDI_Rssi;
    name_mapping["XDI_DeviceId"] = XDI_DeviceId;
    name_mapping["XDI_LocationId"] = XDI_LocationId;

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
    flag_mapping["XDOF_DisableAutoStore"] = XDOF_DisableAutoStore;
    flag_mapping["XDOF_DisableAutoMeasurement"] = XDOF_DisableAutoMeasurement;
    flag_mapping["XDOF_EnableBeidou"] = XDOF_EnableBeidou;
    flag_mapping["XDOF_DisableGps"] = XDOF_DisableGps;
    flag_mapping["XDOF_EnableAhs"] = XDOF_EnableAhs;
    flag_mapping["XDOF_EnableOrientationSmoother"] = XDOF_EnableOrientationSmoother;
    flag_mapping["XDOF_EnableConfigurableBusId"] = XDOF_EnableConfigurableBusId;
    flag_mapping["XDOF_EnableInrunCompassCalibration"] = XDOF_EnableInrunCompassCalibration;
    flag_mapping["XDOF_DisableSleepMode"] = XDOF_DisableSleepMode;
    flag_mapping["XDOF_EnableConfigMessageAtStartup"] = XDOF_EnableConfigMessageAtStartup;
    flag_mapping["XDOF_EnableColdFilterResets"] = XDOF_EnableColdFilterResets;
    flag_mapping["XDOF_EnablePositionVelocitySmoother"] = XDOF_EnablePositionVelocitySmoother;
    flag_mapping["XDOF_EnableContinuousZRU"] = XDOF_EnableContinuousZRU;

    flag_mapping["XDOF_None"] = XDOF_None;
    flag_mapping["XDOF_All"] = XDOF_All;

    return get_xs_value(name, flag_mapping, option_flag);
}

std::string get_xs_all_enabled_flags(const XsDeviceOptionFlag& option_flag)
{
    std::ostringstream oss;

    if (option_flag == XDOF_All)
    {
      oss << "\n - XDOF_All";
      return oss.str();
    }

    if (option_flag == XDOF_None)
    {
      oss << "\n - XDOF_None";
      return oss.str();
    }

    if (option_flag & XDOF_DisableAutoStore)
      oss << "\n - XDOF_DisableAutoStore";
    if (option_flag & XDOF_DisableAutoMeasurement)
      oss << "\n - XDOF_DisableAutoMeasurement";
    if (option_flag & XDOF_EnableBeidou)
      oss << "\n - XDOF_EnableBeidou";
    if (option_flag & XDOF_DisableGps)
      oss << "\n - XDOF_DisableGps";
    if (option_flag & XDOF_EnableAhs)
      oss << "\n - XDOF_EnableAhs";
    if (option_flag & XDOF_EnableOrientationSmoother)
      oss << "\n - XDOF_EnableOrientationSmoother";
    if (option_flag & XDOF_EnableConfigurableBusId)
      oss << "\n - XDOF_EnableConfigurableBusId";
    if (option_flag & XDOF_EnableInrunCompassCalibration)
      oss << "\n - XDOF_EnableInrunCompassCalibration";
    if (option_flag & XDOF_DisableSleepMode)
      oss << "\n - XDOF_DisableSleepMode";
    if (option_flag & XDOF_EnableConfigMessageAtStartup)
      oss << "\n - XDOF_EnableConfigMessageAtStartup";
    if (option_flag & XDOF_EnableColdFilterResets)
      oss << "\n - XDOF_EnableColdFilterResets";
    if (option_flag & XDOF_EnablePositionVelocitySmoother)
      oss << "\n - XDOF_EnablePositionVelocitySmoother";
    if (option_flag & XDOF_EnableContinuousZRU)
      oss << "\n - XDOF_EnableContinuousZRU";

    if (oss.str().empty())
      oss << "\nUnknown";

    return oss.str();
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

