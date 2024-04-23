#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "Acquisition.hpp"
#include "Device.hpp"

namespace rosneuro {
    class MockDevice : public Device {
        public:
            MockDevice() : Device() {}
            ~MockDevice() {}
            MOCK_METHOD2(Configure, bool(NeuroFrame* frame, unsigned int framerate));
            MOCK_METHOD0(Setup, bool());
            MOCK_METHOD0(Open, bool());
            MOCK_METHOD0(Close, bool());
            MOCK_METHOD0(Start, bool());
            MOCK_METHOD0(Stop, bool());
            MOCK_METHOD0(Get, size_t());
            MOCK_METHOD0(GetAvailable, size_t());

            MOCK_CONST_METHOD0(GetName, std::string());
            MOCK_METHOD0(Who, void());
            MOCK_METHOD0(Dump, void());
    };

    class MockAcquisition : public Acquisition {
        public:
            MockAcquisition() : Acquisition() {}
            ~MockAcquisition() {}
            MOCK_METHOD0(configure, bool());
    };

    class AcquisitionTestSuite : public ::testing::Test {
        public:
            AcquisitionTestSuite() {}
            ~AcquisitionTestSuite() {}
            void SetUp() {
                mockDevice = new MockDevice();
                mockAcquisition = new MockAcquisition();
                acquisition = new Acquisition();

                testing::Mock::AllowLeak(mockDevice);
                testing::Mock::AllowLeak(mockAcquisition);
            }

            Acquisition* acquisition;
            MockDevice* mockDevice;
            MockAcquisition* mockAcquisition;
    };

    TEST_F(AcquisitionTestSuite, Initialization) {
        EXPECT_EQ(acquisition->state_, Acquisition::IS_IDLE);
        EXPECT_EQ(acquisition->neuroseq_, 0);
        EXPECT_EQ(acquisition->topic_, "/neurodata");
        EXPECT_EQ(acquisition->autostart_, false);
        EXPECT_NE(acquisition->loader_, nullptr);
        EXPECT_EQ(acquisition->devname_, "");
    }

    TEST_F(AcquisitionTestSuite, RunSuccess) {
        EXPECT_CALL(*mockDevice, Open()).WillOnce(testing::Return(true));
        EXPECT_CALL(*mockDevice, Setup()).WillOnce(testing::Return(true));

        mockAcquisition->dev_ = boost::shared_ptr<MockDevice>(mockDevice);
        mockAcquisition->state_ = Acquisition::IS_QUIT;

        EXPECT_CALL(*mockAcquisition, configure()).WillOnce(testing::Return(true));
        EXPECT_TRUE(mockAcquisition->Run());
    }

    TEST_F(AcquisitionTestSuite, RunFailure) {
        EXPECT_CALL(*mockAcquisition, configure()).WillOnce(testing::Return(false));
        EXPECT_FALSE(mockAcquisition->Run());
    }

    TEST_F(AcquisitionTestSuite, RunFailureOpen) {
        EXPECT_CALL(*mockDevice, Open()).WillOnce(testing::Return(false));

        mockAcquisition->dev_ = boost::shared_ptr<MockDevice>(mockDevice);
        mockAcquisition->state_ = Acquisition::IS_QUIT;

        EXPECT_CALL(*mockAcquisition, configure()).WillOnce(testing::Return(true));
        EXPECT_FALSE(mockAcquisition->Run());
    }

    TEST_F(AcquisitionTestSuite, Integration){
        pluginlib::ClassLoader<Device> plug_loader("rosneuro_acquisition", "rosneuro::Device");
        boost::shared_ptr<Device> lsl_device, egd_device;
        ASSERT_NO_THROW(lsl_device = plug_loader.createInstance("rosneuro::LSLDevice"));
        ASSERT_NO_THROW(egd_device = plug_loader.createInstance("rosneuro::EGDDevice"));
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_acquisition");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}