#include <gtest/gtest.h>
#include "Acquisition.hpp"

namespace rosneuro {
    TEST(AcquisitionTestSuiteConf, Configure) {
        Acquisition* acquisition = new Acquisition();
        ros::param::set("~plugin", "rosneuro::EGDDevice");
        ros::param::set("~framerate", 256.0);
        ros::param::set("~reopen", true);
        ros::param::set("~autostart", true);
        ros::param::set("~device", "egd");
        ros::param::set("~devarg", "arg");
        EXPECT_TRUE(acquisition->configure());

        ros::param::set("~plugin", 0.0);
        EXPECT_FALSE(acquisition->configure());

        ros::param::set("~plugin", "rosneuro::EGDDevice");
        ros::param::set("~framerate", "error");
        EXPECT_FALSE(acquisition->configure());

        ros::param::set("~devarg", 0.0);
        ros::param::set("~framerate", 256.0);
        EXPECT_FALSE(acquisition->configure());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_acquisition_configuration");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}