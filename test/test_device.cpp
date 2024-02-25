#include "gtest/gtest.h"
#include "rosneuro_acquisition/Device.hpp"

class DeviceTest : public rosneuro::Device {
public:
    DeviceTest() : rosneuro::Device() {}
    DeviceTest(rosneuro::NeuroFrame* frame) : rosneuro::Device(frame) {}
    virtual ~DeviceTest() {}
    virtual bool Configure(rosneuro::NeuroFrame* frame, unsigned int framerate) { return true; }
    virtual bool Setup(void) { return true; }
    virtual bool Open(void) { return true; }
    virtual bool Close(void) { return true; }
    virtual bool Start(void) { return true; }
    virtual bool Stop(void) { return true; }
    virtual size_t Get(void) { return 0; }
    virtual size_t GetAvailable(void) { return 0; }
};

namespace rosneuro {

TEST(DeviceTest, DefaultDeviceInfoConstructor) {
    DeviceInfo info;

    // check if info model and id are empty and string type
    EXPECT_EQ(info.model, "");
    EXPECT_EQ(info.id, "");
}

TEST(DeviceTest, DefaultConstructor) {
    DeviceTest device;
    EXPECT_EQ(device.GetName(), "undefined");
}

TEST(DeviceTest, ParameterizedConstructor) {
    NeuroFrame frame;
    DeviceTest device(&frame);
    EXPECT_EQ(device.GetName(), "undefined");
}

TEST(DeviceTest, Destructor) {
    DeviceTest* device = new DeviceTest;
    EXPECT_NO_THROW(delete device);
}

TEST(DeviceTest, Who) {
    DeviceTest device;
    testing::internal::CaptureStdout();  // Redirect stdout for testing
    device.Who();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "[undefined] - undefined device\n");
}

TEST(DeviceTest, Dump) {
    DeviceTest device;
    testing::internal::CaptureStdout();  // Redirect stdout for testing
    device.Dump();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "[Dump] undefined info:\n |- Model:         \n |- Id:            \n");
}

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}