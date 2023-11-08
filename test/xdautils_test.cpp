#include "gtest/gtest.h"

#include "xdautils.h"

TEST(Xda, test_get_xs_data_identifier_by_name)
{
    XsDataIdentifier id;
    ASSERT_TRUE(get_xs_data_identifier_by_name("Acceleration", id));
    ASSERT_EQ(id, XDI_Acceleration);
}

TEST(Xda, test_get_xs_data_identifier_by_name_non_existing)
{
    XsDataIdentifier id;
    ASSERT_FALSE(get_xs_data_identifier_by_name("Acceleration2", id));
}

TEST(Xda, test_get_xs_data_identifier_name)
{
    ASSERT_EQ(get_xs_data_identifier_name(XDI_Acceleration), "Acceleration");
    ASSERT_EQ(get_xs_data_identifier_name(static_cast<XsDataIdentifier>(0xFFFF)), "Unknown");
}

TEST(Xda, test_get_xs_format_identifier_by_name)
{
    XsDataIdentifier format;
    ASSERT_TRUE(get_xs_format_identifier_by_name("Double", format));
    ASSERT_EQ(format, XDI_SubFormatDouble);

    ASSERT_TRUE(get_xs_format_identifier_by_name("DoUbLe", format));
    ASSERT_EQ(format, XDI_SubFormatDouble);
}

TEST(Xda, test_get_xs_format_identifier_by_name_non_existing)
{
    XsDataIdentifier format;
    ASSERT_FALSE(get_xs_format_identifier_by_name("Doubllll", format));
}

TEST(Xda, test_get_xs_format_identifier_name)
{
    ASSERT_EQ(get_xs_format_identifier_name(XDI_SubFormatDouble), "Double");
    ASSERT_EQ(get_xs_data_identifier_name(static_cast<XsDataIdentifier>(0xFFFF)), "Unknown");
}

TEST(Xda, test_get_xs_enabled_flag_by_name)
{
    XsDeviceOptionFlag flag;
    ASSERT_TRUE(get_xs_enabled_flag_by_name("EnableContinuousZRU", flag));
    ASSERT_EQ(flag, XDOF_EnableContinuousZRU);
}

TEST(Xda, test_get_xs_enabled_flag_by_name_non_existing)
{
    XsDeviceOptionFlag flag;
    ASSERT_FALSE(get_xs_enabled_flag_by_name("EnableContinuousZRU2", flag));
}

TEST(Xda, test_get_xs_all_enabled_flags)
{
    std::string enabled_str = "\n - DisableGps\n - EnableOrientationSmoother\n - EnableContinuousZRU";
    ASSERT_EQ(get_xs_all_enabled_flags(XDOF_EnableContinuousZRU | XDOF_DisableGps | XDOF_EnableOrientationSmoother), enabled_str);

    enabled_str = "\n - None";
    ASSERT_EQ(get_xs_all_enabled_flags(XDOF_None), enabled_str);

    enabled_str = "\n - All";
    ASSERT_EQ(get_xs_all_enabled_flags(XDOF_All), enabled_str);

    enabled_str = "\nUnknown";
    ASSERT_EQ(get_xs_all_enabled_flags(static_cast<XsDeviceOptionFlag>(0xF0000000)), enabled_str);
}

TEST(Xda, test_get_xs_gnss_platform_by_name)
{
    XsGnssPlatform platform;
    ASSERT_TRUE(get_xs_gnss_platform_by_name("Automotive", platform));
    ASSERT_EQ(platform, XGP_Automotive);

    ASSERT_TRUE(get_xs_gnss_platform_by_name("auToMotive", platform));
    ASSERT_EQ(platform, XGP_Automotive);
}

TEST(Xda, test_get_xs_gnss_platform_by_name_non_existing)
{
    XsGnssPlatform platform;
    ASSERT_FALSE(get_xs_gnss_platform_by_name("Automotiveee2", platform));
}

TEST(Xda, test_get_xs_gnss_platform_name)
{
    ASSERT_EQ(get_xs_gnss_platform_name(XGP_Automotive), "Automotive");
    ASSERT_EQ(get_xs_gnss_platform_name(static_cast<XsGnssPlatform>(0xFFFF)), "Unknown");
}

TEST(Xda, test_parse_line)
{
    XsDataIdentifier identifier;
    int freq;
    ASSERT_TRUE(parseConfigLine("Acceleration|Double|1337", identifier, freq));
    ASSERT_EQ(identifier & XDI_FullTypeMask, XDI_Acceleration);
    ASSERT_EQ(identifier & XDI_SubFormatMask, XDI_SubFormatDouble);
    ASSERT_EQ(freq, 1337);
}

TEST(Xda, test_parse_line_error)
{
    XsDataIdentifier identifier;
    int freq;
    ASSERT_FALSE(parseConfigLine("Acceleration|Double", identifier, freq));
    ASSERT_FALSE(parseConfigLine("Acceleration", identifier, freq));
}

TEST(Xda, test_parse_line_error2)
{
    XsDataIdentifier identifier;
    int freq;
    ASSERT_FALSE(parseConfigLine("Acceleration|Floatttt|12", identifier, freq));
    ASSERT_FALSE(parseConfigLine("Acceleration|Float/12", identifier, freq));
    ASSERT_FALSE(parseConfigLine("Acceleration=Float|12", identifier, freq));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}