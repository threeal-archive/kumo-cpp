# Kumo

**Kumo** is a tools to create a web based interface to configure and monitor other nodes within the **ROS 2** environment.

## Changelog

### Version 0.1.0

Changes by **Alfi Maulana**, 8 May 2020:
- **Add `daemon` executable**.
  This executable will be used as the standard daemon to handle communication between **ROS 2** nodes and the web client.
  Currently this daemon could only handles page request from the web client.
- **Add `HttpHandler` class**.
  This class will be used to handle http request from the web client.
  Currently it could handles a **GET** request and pass it to the `PageHandler`
    and it could secure the **HTTP** connection by response to a basic authorization.
- **Add `PageHandler` class**.
  This class will be used to handle page that was requested by the web client.
  Currently it could handle file in the `share/kumo/web` directory within `/` (root) request path.