#ifndef XDAUTILS_H
#define XDAUTILS_H

#include <xstypes/xsdataidentifier.h>
#include <string>

std::string get_xs_data_identifier_name(const XsDataIdentifier& identifier);
std::string get_xs_format_identifier_name(const XsDataIdentifier& identifier);
bool get_xs_data_identifier_by_name(const std::string& name, XsDataIdentifier& identifier);
bool get_xs_format_identifier_by_name(const std::string& format_str, XsDataIdentifier& identifier);
bool parseConfigLine(const std::string& line, XsDataIdentifier& identifier, int& frequency);

#endif
