#include <memory>
#include <string>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>


namespace summer_robotics_gazebo_plugins
{

class ObjectPosePlugin : public gazebo::WorldPlugin
{
public:
    ObjectPosePlugin() = default;

    ~ObjectPosePlugin() override
    {
        if (executor_)
        {
            executor_->cancel();
        }

        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
    }

    void Load(
        gazebo::physics::WorldPtr world,
        sdf::ElementPtr /* sdf */) override
    {
        world_ = world;

        if (!rclcpp::ok())
        {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }

        ros_node_ = std::make_shared<rclcpp::Node>(
            "gazebo_object_pose_plugin"
        );

        set_entity_state_service_ =
            ros_node_->create_service<gazebo_msgs::srv::SetEntityState>(
                "/set_entity_state_custom",
                std::bind(
                    &ObjectPosePlugin::set_entity_state_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );

        executor_ =
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        executor_->add_node(ros_node_);

        executor_thread_ = std::thread(
            [this]()
            {
                executor_->spin();
            }
        );

        RCLCPP_INFO(
            ros_node_->get_logger(),
            "Gazebo Object Pose Plugin loaded."
        );

        RCLCPP_INFO(
            ros_node_->get_logger(),
            "Service available at /set_entity_state_custom"
        );
    }

private:
    void set_entity_state_callback(
        const std::shared_ptr<
            gazebo_msgs::srv::SetEntityState::Request
        > request,
        std::shared_ptr<
            gazebo_msgs::srv::SetEntityState::Response
        > response)
    {
        const std::string entity_name = request->state.name;

        gazebo::physics::ModelPtr model =
            world_->ModelByName(entity_name);

        if (!model)
        {
            response->success = false;

            RCLCPP_WARN(
                ros_node_->get_logger(),
                "Gazebo model not found: %s",
                entity_name.c_str()
            );

            return;
        }

        const auto &requested_pose = request->state.pose;

        ignition::math::Quaterniond rotation(
        requested_pose.orientation.w,
        requested_pose.orientation.x,
        requested_pose.orientation.y,
        requested_pose.orientation.z
    );

    ignition::math::Pose3d gazebo_pose(
        requested_pose.position.x,
        requested_pose.position.y,
        requested_pose.position.z,
        rotation.Roll(),
        rotation.Pitch(),
        rotation.Yaw()
    );

        model->SetWorldPose(gazebo_pose);

        response->success = true;

        RCLCPP_INFO(
            ros_node_->get_logger(),
            "Moved %s to x=%.3f, y=%.3f, z=%.3f",
            entity_name.c_str(),
            requested_pose.position.x,
            requested_pose.position.y,
            requested_pose.position.z
        );
    }

    gazebo::physics::WorldPtr world_;

    rclcpp::Node::SharedPtr ros_node_;

    rclcpp::Service<
        gazebo_msgs::srv::SetEntityState
    >::SharedPtr set_entity_state_service_;

    std::shared_ptr<
        rclcpp::executors::SingleThreadedExecutor
    > executor_;

    std::thread executor_thread_;
};

GZ_REGISTER_WORLD_PLUGIN(ObjectPosePlugin)

}  // namespace summer_robotics_gazebo_plugins
