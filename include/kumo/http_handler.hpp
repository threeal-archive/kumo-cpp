#ifndef KUMO__HTTP_HANDLER_HPP_
#define KUMO__HTTP_HANDLER_HPP_

#include "kumo/page_handler.hpp"

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <simpleweb/http_server.hpp>

namespace kumo
{
  class HttpHandler
  {
  public:

    HttpHandler(std::shared_ptr<rclcpp::Node> node, int port = 8080);

    bool start();

    std::shared_ptr<PageHandler> page_handler;
    bool enable_authorization;

  private:

    bool authorize(std::shared_ptr<SimpleWeb::HttpServer::Response> response,
      std::shared_ptr<SimpleWeb::HttpServer::Request> request);

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Logger logger;

    SimpleWeb::HttpServer http_server;
    std::shared_ptr<std::thread> thread_handler;
    std::promise<bool> thread_lock;
  };
}

#endif