#include "kumo/http_handler.hpp"

#include <simpleweb/crypto.hpp>

using namespace kumo;

HttpHandler::HttpHandler(std::shared_ptr<rclcpp::Node> node, int port)
  : page_handler(std::make_shared<PageHandler>(node)),
    enable_authorization(true),
    node(node),
    logger(node->get_logger().get_child("http_handler"))
{
  if (rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_WARN)
    != RCUTILS_RET_OK)
  {
    RCUTILS_LOG_FATAL("could not set the logger level");
  }

  RCLCPP_INFO(logger, "set the port to " + std::to_string(port));
  http_server.config.port = port;

  http_server.default_resource["GET"] = [this](
    std::shared_ptr<SimpleWeb::HttpServer::Response> response,
    std::shared_ptr<SimpleWeb::HttpServer::Request> request
  ) {
    RCLCPP_INFO(logger, "response to GET request");
    if (this->enable_authorization && this->authorize(response, request) == false)
      return;

    this->page_handler->handle_request(response, request);
  };
}

bool HttpHandler::start()
{
  try {
    RCLCPP_INFO(logger, "starting the http server");
    thread_handler = std::make_shared<std::thread>([this]() { this->http_server.start(); });
    return true;
  }
  catch (std::exception &e) {
    RCLCPP_ERROR(logger, "could not start the http server ", std::string(e.what()));
    return false;
  }
}

bool HttpHandler::authorize(std::shared_ptr<SimpleWeb::HttpServer::Response> response,
  std::shared_ptr<SimpleWeb::HttpServer::Request> request)
{
  try {
    RCLCPP_INFO(logger, "authorizing request");

    auto auth_it = request->header.find("Authorization");
    if (auth_it == request->header.end())
      throw std::invalid_argument("unauthorized access");

    auto auth_content = auth_it->second;
    auto auth_key = SimpleWeb::Crypto::Base64::decode(
      auth_content.substr(auth_content.find(" ") + 1, auth_content.size())
    );

    auto auth_user = auth_key.substr(0, auth_key.find(":"));
    RCLCPP_DEBUG(logger, "got " + auth_user + " as the user");

    auto auth_pass = auth_key.substr(auth_key.find(":") + 1, auth_key.size());
    RCLCPP_DEBUG(logger, "got " + std::string(auth_pass.length(), '*') + " as the password");

    if (system(std::string("[ '" + auth_user + "' = \"$(whoami)\" ]").c_str()) != 0)
      throw std::invalid_argument("invalid username");

    if (system(std::string("echo '" + auth_pass + "' | sudo -S true > /dev/null 2>&1").c_str()) != 0)
      throw std::invalid_argument("invalid password");

    return true;
  }
  catch (std::exception &e) {
    RCLCPP_WARN(logger, "authorization failed " + std::string(e.what()));

    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("WWW-Authenticate", "Basic realm= \"User Visible Realm\", charset=\"UTF-8\"");
    response->write(SimpleWeb::StatusCode::client_error_unauthorized, e.what(), header);

    return false;
  }
}