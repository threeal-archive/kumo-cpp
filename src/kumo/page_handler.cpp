#include "kumo/page_handler.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>

using namespace kumo;

PageHandler::PageHandler(std::shared_ptr<rclcpp::Node> node)
  : node(node),
    logger(node->get_logger().get_child("page_handler"))
{
  if (rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_INFO)
    != RCUTILS_RET_OK)
  {
    RCUTILS_LOG_FATAL("could not set the logger level");
  }
}

bool PageHandler::handle_request(std::shared_ptr<SimpleWeb::HttpServer::Response> response,
  std::shared_ptr<SimpleWeb::HttpServer::Request> request)
{
  try {
    RCLCPP_INFO(logger, "handling " + request->path);

    auto root_path = boost::filesystem::canonical(
      ament_index_cpp::get_package_share_directory("kumo")
    );

    root_path /= "web";
    RCLCPP_DEBUG(logger, "got " + root_path.string() + " as the root path");

    auto path = boost::filesystem::canonical(root_path / request->path);
    if (std::distance(root_path.begin(), root_path.end()) > std::distance(path.begin(), path.end())
      || !std::equal(root_path.begin(), root_path.end(), path.begin()))
    {
      throw std::invalid_argument("forbidden access");
    }

    if (boost::filesystem::is_directory(path))
      path /= "index.html";

    return serve_file(path.string(), response);
  }
  catch (std::exception &e) {
    RCLCPP_ERROR(logger, "exception " + std::string(e.what()));
    response->write(SimpleWeb::StatusCode::client_error_bad_request, e.what());
    return false;
  }
}

bool PageHandler::serve_file(std::string path, std::shared_ptr<SimpleWeb::HttpServer::Response> response)
{
  try {
    RCLCPP_INFO(logger, "serving " + path);

    std::ifstream ifs;
    ifs.open(path, std::ifstream::in | std::ios_base::binary | std::ios_base::ate);

    if (!ifs)
      throw std::invalid_argument("could not read " + path);

    int length = ifs.tellg();
    ifs.seekg(0, std::ios_base::beg);

    std::vector<char> fbuf(length);
    auto read_length = ifs.read(&fbuf[0], static_cast<std::streamsize>(length)).gcount();
    if (read_length <= 0)
      throw std::invalid_argument("could not read " + path);

    if (read_length < length)
      RCLCPP_WARN(logger, "read length less than file length");

    SimpleWeb::CaseInsensitiveMultimap header;
    header.emplace("Content-Length", std::to_string(read_length));
    response->write(header);
    response->write(&fbuf[0], read_length);

    return true;
  }
  catch (std::exception &e) {
    RCLCPP_ERROR(logger, "exception " + std::string(e.what()));
    response->write(SimpleWeb::StatusCode::client_error_bad_request, e.what());
    return false;
  }
}