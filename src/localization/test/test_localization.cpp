#include "localization/mpu_localizer.hpp"
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1)
{
    ASSERT_TRUE(false);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
