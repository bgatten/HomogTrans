#include <gtest/gtest.h>
#include <htf/htf.h>  // Include your library header

// Write your test cases here
TEST(HtfTest, ExampleTest) {
    htf::Htf myTransform;
    // Test assertions here
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
