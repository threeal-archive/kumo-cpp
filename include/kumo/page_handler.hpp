#ifndef KUMO__PAGE_HANDLER_HPP_
#define KUMO__PAGE_HANDLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <simpleweb/http_server.hpp>

namespace kumo
{
  class PageHandler
  {
  public:

    PageHandler(std::shared_ptr<rclcpp::Node> node);

    bool handle_request(std::shared_ptr<SimpleWeb::HttpServer::Response> response,
      std::shared_ptr<SimpleWeb::HttpServer::Request> request);

  private:

    bool serve_file(std::string path, std::shared_ptr<SimpleWeb::HttpServer::Response> response);

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Logger logger;
  };
}

#endif