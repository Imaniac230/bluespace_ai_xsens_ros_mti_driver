#ifndef ORIENTATIONRESET_H
#define ORIENTATIONRESET_H

#include <std_srvs/srv/set_bool.hpp>

#include "servicehandler.hpp"

class OrientationReset : public ServiceHandler
{
    public:
        explicit OrientationReset(rclcpp::Node& node, XsDevice* m_device)
        {
            reset_heading_srv = node.create_service<std_srvs::srv::SetBool>(
                "reset_heading", [&node, m_device](const std_srvs::srv::SetBool::Request::SharedPtr,
                                        std_srvs::srv::SetBool::Response::SharedPtr response) {
                  RCLCPP_INFO(node.get_logger(), "Performing orientation reset for heading.");
                  response->success = m_device->resetOrientation(XsResetMethod::XRM_Heading);
                  if (!response->success)
                  {
                      response->message = "Could not perform orientation reset.";
                      RCLCPP_ERROR(node.get_logger(), response->message.c_str());
                  }
                  return response;
                });

            reset_inclination_srv = node.create_service<std_srvs::srv::SetBool>(
                "reset_inclination", [&node, m_device](const std_srvs::srv::SetBool::Request::SharedPtr,
                                            std_srvs::srv::SetBool::Response::SharedPtr response) {
                  RCLCPP_INFO(node.get_logger(), "Performing orientation reset for inclination.");
                  response->success = m_device->resetOrientation(XsResetMethod::XRM_Inclination);
                  if (!response->success)
                  {
                      response->message = "Could not perform orientation reset.";
                      RCLCPP_ERROR(node.get_logger(), response->message.c_str());
                  }
                  return response;
                });
        }

    private:
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_heading_srv;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_inclination_srv;
};

#endif // ORIENTATIONRESET_H
