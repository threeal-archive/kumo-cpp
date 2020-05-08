#include <memory>

#include <kumo/http_handler.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("kumo");

  RCLCPP_INFO(node->get_logger(), "initializing the program");

  auto http_handler = std::make_shared<kumo::HttpHandler>(node, 8080);
  http_handler->start();

  RCLCPP_INFO(node->get_logger(), "entering the spin loop");
  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "shutting down the program");
  rclcpp::shutdown();

  return 0;
}