#ifndef XDAUTILS_H
#define XDAUTILS_H

#include <xstypes/xsdataidentifier.h>
#include <xstypes/xsdeviceoptionflag.h>
#include <xscontroller/xsgnssplatform.h>

#include <map>
#include <string>

std::string get_xs_data_identifier_name(const XsDataIdentifier& identifier);
std::string get_xs_format_identifier_name(const XsDataIdentifier& identifier);
std::string get_xs_all_enabled_flags(const XsDeviceOptionFlag& option_flag);
std::string get_xs_gnss_platform_name(const XsGnssPlatform& platform);

bool get_xs_data_identifier_by_name(const std::string& name, XsDataIdentifier& identifier);
bool get_xs_format_identifier_by_name(const std::string& format_str, XsDataIdentifier& identifier);
bool get_xs_enabled_flag_by_name(const std::string& name, XsDeviceOptionFlag& option_flag);
bool get_xs_gnss_platform_by_name(const std::string& name, XsGnssPlatform& platform);

bool parseConfigLine(const std::string& line, XsDataIdentifier& identifier, int& frequency);

template <typename XsType>
inline bool get_xs_value(const std::string& name, const std::map<std::string, XsType>& map, XsType& value)
{
  const auto found = map.find(name);
  if (found != map.end())
  {
    value = found->second;
    return true;
  }
  else
  {
    return false;
  }
}

#endif
