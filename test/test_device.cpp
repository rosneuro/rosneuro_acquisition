#include "gtest/gtest.h"
#include "Device.hpp"

namespace rosneuro {

class DeviceTest : public Device {
    public:
        DeviceTest() : Device() {}
        DeviceTest(NeuroFrame* frame) : Device(frame) {}
        virtual ~DeviceTest() {}
        virtual bool Configure(NeuroFrame* frame, unsigned int framerate) { return true; }
        virtual bool Setup(void) { return true; }
        virtual bool Open(void) { return true; }
        virtual bool Close(void) { return true; }
        virtual bool Start(void) { return true; }
        virtual bool Stop(void) { return true; }
        virtual size_t Get(void) { return 0; }
        virtual size_t GetAvailable(void) { return 0; }
};

class DeviceTestSuite : public ::testing::Test {
    public:
        DeviceTestSuite() {}
        ~DeviceTestSuite() {}
        void SetUp() { device = new DeviceTest(); }
        void TearDown() { delete device; }
        DeviceTest* device;
        DeviceInfo info;
};

TEST_F(DeviceTestSuite, DefaultDeviceInfoConstructor) {
    EXPECT_EQ(info.model, "");
    EXPECT_EQ(info.id, "");
}

TEST_F(DeviceTestSuite, DefaultConstructor) {
    EXPECT_EQ(device->GetName(), "undefined");
}

TEST_F(DeviceTestSuite, ParameterizedConstructor) {
    NeuroFrame frame;
    device = new DeviceTest(&frame);
    EXPECT_EQ(device->GetName(), "undefined");
}

TEST_F(DeviceTestSuite, Destructor) {
    DeviceTest* device_throw = new DeviceTest();
    EXPECT_NO_THROW(delete device_throw);
}

TEST_F(DeviceTestSuite, Who) {
    testing::internal::CaptureStdout();
    device->Who();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "[undefined] - undefined device\n");
}

TEST_F(DeviceTestSuite, Dump) {
    testing::internal::CaptureStdout();
    device->Dump();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "[Dump] undefined info:\n |- Model:         \n |- Id:            \n");
}

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}