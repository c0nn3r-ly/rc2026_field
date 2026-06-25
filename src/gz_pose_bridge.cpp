#include <functional>
#include <memory>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

class GzPoseBridge : public rclcpp::Node
{
public:
  GzPoseBridge()
  : rclcpp::Node("gz_pose_bridge")
  {
    const auto world_name =
      this->declare_parameter<std::string>("world_name", "robocon2026_world_scene");
    const auto default_service =
      "/world/" + world_name + "/set_pose/blocking";

    this->gz_service_name_ =
      this->declare_parameter<std::string>("gz_service_name", default_service);
    this->service_timeout_ms_ =
      this->declare_parameter<int>("service_timeout_ms", 5000);

    this->set_entity_pose_srv_ =
      this->create_service<ros_gz_interfaces::srv::SetEntityPose>(
        "/simulation/set_entity_pose",
        std::bind(
          &GzPoseBridge::HandleSetEntityPose,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Bridging /simulation/set_entity_pose to Gazebo service [%s]",
      this->gz_service_name_.c_str());
  }

private:
  void HandleSetEntityPose(
    const std::shared_ptr<ros_gz_interfaces::srv::SetEntityPose::Request> request,
    std::shared_ptr<ros_gz_interfaces::srv::SetEntityPose::Response> response)
  {
    if (request->entity.id == 0 && request->entity.name.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting pose request without entity id or name");
      response->success = false;
      return;
    }

    if (
      request->entity.type != ros_gz_interfaces::msg::Entity::NONE &&
      request->entity.type != ros_gz_interfaces::msg::Entity::MODEL)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Expected MODEL entity type for [%s], got [%u]",
        request->entity.name.c_str(),
        static_cast<unsigned int>(request->entity.type));
    }

    gz::msgs::Pose gz_pose;
    if (request->entity.id != 0) {
      gz_pose.set_id(request->entity.id);
    }
    gz_pose.set_name(request->entity.name);

    auto * position = gz_pose.mutable_position();
    position->set_x(request->pose.position.x);
    position->set_y(request->pose.position.y);
    position->set_z(request->pose.position.z);

    auto * orientation = gz_pose.mutable_orientation();
    orientation->set_x(request->pose.orientation.x);
    orientation->set_y(request->pose.orientation.y);
    orientation->set_z(request->pose.orientation.z);
    orientation->set_w(request->pose.orientation.w);

    gz::msgs::Boolean gz_response;
    bool gz_result = false;
    const bool request_sent = this->gz_node_.Request(
      this->gz_service_name_,
      gz_pose,
      this->service_timeout_ms_,
      gz_response,
      gz_result);

    response->success = request_sent && gz_result && gz_response.data();

    if (!response->success) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to set pose for [%s] via [%s] (sent=%s, result=%s, response=%s)",
        request->entity.name.c_str(),
        this->gz_service_name_.c_str(),
        request_sent ? "true" : "false",
        gz_result ? "true" : "false",
        gz_response.data() ? "true" : "false");
    }
  }

  gz::transport::Node gz_node_;
  rclcpp::Service<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_entity_pose_srv_;
  std::string gz_service_name_;
  int service_timeout_ms_{5000};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
