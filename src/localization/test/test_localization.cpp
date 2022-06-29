#include "localization/mpu_localizer.hpp"
#include <gtest/gtest.h>

// Declare a test
TEST(MpuLocalizer, updateNoArgumentsTransform)
{
    ASSERT_TRUE(UpdateTransform());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
