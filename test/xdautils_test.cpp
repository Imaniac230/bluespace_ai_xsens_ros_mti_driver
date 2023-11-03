#include "gtest/gtest.h"

#include "xdautils.h"

TEST(Xda, test_get_xs_data_identifier_by_name)
{
    XsDataIdentifier id;
    auto result = get_xs_data_identifier_by_name("XDI_Acceleration", id);
    ASSERT_TRUE(result);
    ASSERT_EQ(id, XDI_Acceleration);
}

TEST(Xda, test_get_xs_data_identifier_by_name_non_existing)
{
    XsDataIdentifier id;
    auto result = get_xs_data_identifier_by_name("XDI_Acceleration2", id);
    ASSERT_FALSE(result);
}

TEST(Xda, test_get_xs_data_identifier_name)
{
    auto name = get_xs_data_identifier_name(XDI_Acceleration);
    ASSERT_EQ(name, "XDI_Acceleration");
}

TEST(Xda, test_parse_line)
{
    XsDataIdentifier identifier;
    int freq;
    ASSERT_TRUE(parseConfigLine("XDI_Acceleration|Double|1337", identifier, freq));
    ASSERT_EQ(identifier & XDI_FullTypeMask, XDI_Acceleration);
    ASSERT_EQ(identifier & XDI_SubFormatMask, XDI_SubFormatDouble);
    ASSERT_EQ(freq, 1337);
}

TEST(Xda, test_parse_line_error)
{
    XsDataIdentifier identifier;
    int freq;
    ASSERT_FALSE(parseConfigLine("XDI_Acceleration|Double", identifier, freq));
    ASSERT_FALSE(parseConfigLine("XDI_Acceleration", identifier, freq));
}

TEST(Xda, test_parse_line_error2)
{
    XsDataIdentifier identifier;
    int freq;
    ASSERT_FALSE(parseConfigLine("XDI_Acceleration|Floatttt|12", identifier, freq));
    ASSERT_FALSE(parseConfigLine("XDI_Acceleration|Float/12", identifier, freq));
    ASSERT_FALSE(parseConfigLine("XDI_Acceleration=Float|12", identifier, freq));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}