#include <gtest/gtest.h>
#include "armin_robothw/armin_hw.hpp"


TEST(RobotHWCallbackParserTests, CorrectnessTest01)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "first10";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "first10");
    EXPECT_EQ(key_out, -1);
}

TEST(RobotHWCallbackParserTests, CorrectnessTest02)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "second ";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "second");
    EXPECT_EQ(key_out, -1);
}

TEST(RobotHWCallbackParserTests, CorrectnessTest03)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "third value";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "third");
    EXPECT_EQ(key_out, -1);
}

TEST(RobotHWCallbackParserTests, CorrectnessTest04)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "forth -6";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "forth");
    EXPECT_EQ(key_out, -6);
}

TEST(RobotHWCallbackParserTests, CorrectnessTest05)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "fifth 0";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "fifth");
    EXPECT_EQ(key_out, 0);
}

TEST(RobotHWCallbackParserTests, CorrectnessTest06)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "last 60";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "last");
    EXPECT_EQ(key_out, 60);
}

TEST(RobotHWCallbackParserTests, CorrectnessTestEmpty01)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = "";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, "");
    EXPECT_EQ(key_out, -1);
}
TEST(RobotHWCallbackParserTests, CorrectnessTestEmpty02)
{
    std::string v;
    std_msgs::StringPtr msgPtr( new std_msgs::String());
    
    int key_out;

    msgPtr->data = " ";
    armin_hardware_interface::ArminHW::CallbackParser(msgPtr, v, key_out);
    EXPECT_EQ(v, " ");
    EXPECT_EQ(key_out, -1);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

