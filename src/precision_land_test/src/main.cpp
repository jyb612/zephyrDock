#include "precision_land_test/precision_land_test.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PrecisionLandTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
