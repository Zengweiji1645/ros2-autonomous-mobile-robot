#include <memory>
#include <cmath>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <sstream>

class MissionNode : public rclcpp::Node
{
public:
    //指定Action类型和GoalHandle类型
    using NavigateToPose =
        nav2_msgs::action::NavigateToPose;

    using GoalHandle =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MissionNode()
        : Node("mission_node")
    {
         this->declare_parameter<std::string>("waypoints_file","");
        // 创建一个导航客户端
         nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose"
        );

        loadWaypoints();

        // 创建一个订阅器
        command_subscriber_ =
        this->create_subscription<std_msgs::msg::String>(
        "/mission_command",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            receiveMission(msg->data);
        }
     );

       // 创建一个发布器
        status_publisher_ =
        this->create_publisher<std_msgs::msg::String>(
            "/mission_status",
            10
        );

        RCLCPP_INFO(
            this->get_logger(),
            "MissionNode started"
        );

    }

private:
    // 创建一个导航客户端
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
   
    //创建一个订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
   // 创建一个发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
   
    // 发送导航目标
    struct Waypoint
    {
        double x;
        double y;
        double yaw;
    };
    std::map<std::string, Waypoint> waypoints_ ;

    // 加载目标点数据
    void loadWaypoints()
{
   std::string file_path =
    this->get_parameter("waypoints_file").as_string();

if (file_path.empty())
{
    RCLCPP_ERROR(
        this->get_logger(),
        "Waypoint configuration path is empty"
    );
    return;
}

    try
    {
        YAML::Node config = YAML::LoadFile(file_path);

        for (const auto & item : config["waypoints"])
        {
            std::string name =
                item.first.as<std::string>();

            Waypoint point;

            point.x = item.second["x"].as<double>();
            point.y = item.second["y"].as<double>();
            point.yaw = item.second["yaw"].as<double>();

            waypoints_[name] = point;

            RCLCPP_INFO(
                this->get_logger(),
                "Loaded waypoint %s: x=%.2f y=%.2f yaw=%.2f",
                name.c_str(),
                point.x,
                point.y,
                point.yaw
            );
        }
    }
    catch (const YAML::Exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to load waypoints: %s",
            e.what()
        );
    }
}

    // 发送导航目标
    void sendGoal(const std::string & waypoint_name)
{
    // 检查目标点是否存在
    auto it = waypoints_.find(waypoint_name);
    if (it == waypoints_.end())
    {
        publishStatus("Unknown waypoint: " + waypoint_name);
        return;
    }

    //检查导航客户端Nav2是否准备好
    if (!nav_client_->action_server_is_ready())
    {
        publishStatus("Navigation not ready");
        return;
    }

    // 获取目标点数据
    Waypoint point = it->second;
    NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = point.x;
    goal.pose.pose.position.y = point.y;
    goal.pose.pose.position.z = 0.0;

    // 将yaw角转换为四元数
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(point.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(point.yaw / 2.0);

    //设置任务结果处理
    auto options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
        [this, waypoint_name](GoalHandle::SharedPtr handle)
        {
            if (!handle)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Goal %s rejected",
                    waypoint_name.c_str()
                );

                is_mission_running_ = false;
                publishStatus("Rejected" + waypoint_name);
                publishStatus("MISSION_FAILED");
                return;
            }

            //保存当前导航目标
            current_goal_handle_ = handle;

            //如果请求取消任务，发送取消请求
            if (cancel_requested_)
            {
              nav_client_->async_cancel_goal(handle);
              return;
            }
            
            publishStatus("NAVIGATING_TO_" + waypoint_name);
        };

  options.result_callback =
    [this, waypoint_name](
        const GoalHandle::WrappedResult & result)
    {
        // 当前目标已经结束
        current_goal_handle_.reset();

        // 如果之前请求过取消
        if (cancel_requested_)
        {
            is_mission_running_ = false;
            cancel_requested_ = false;

            if (result.code ==
                rclcpp_action::ResultCode::CANCELED)
            {
                publishStatus("MISSION_CANCELED");
            }
            else if (result.code ==
                     rclcpp_action::ResultCode::SUCCEEDED)
            {
                publishStatus("CANCEL_TOO_LATE");
                publishStatus("MISSION_STOPPED");
            }
            else
            {
                publishStatus("MISSION_FAILED");
            }

            return;
        }

        // 正常完成当前目标
        if (result.code ==
            rclcpp_action::ResultCode::SUCCEEDED)
        {
            publishStatus("REACHED_" + waypoint_name);

            current_mission_index_++;

            if (current_mission_index_ < mission_queue_.size())
            {
                sendGoal(
                    mission_queue_[current_mission_index_]
                );
            }
            else
            {
                is_mission_running_ = false;
                publishStatus("MISSION_COMPLETED");
            }
        }
        else
        {
            // 当前目标失败，停止后续任务
            is_mission_running_ = false;

            if (result.code ==
                rclcpp_action::ResultCode::CANCELED)
            {
                publishStatus("MISSION_CANCELED");
            }
            else
            {
                publishStatus("FAILED_" + waypoint_name);
                publishStatus("MISSION_FAILED");
            }
        }
    };

    // 发送导航目标    
    nav_client_->async_send_goal(goal, options);

}

   // 发布任务状态
    void publishStatus(const std::string & status)
{
    std_msgs::msg::String msg;
    msg.data = status;

    status_publisher_->publish(msg);

    RCLCPP_INFO(
        this->get_logger(),
        "Mission status: %s",
        status.c_str()
    );
}

// 任务队列
std::vector<std::string> mission_queue_;
// 正在执行的任务
std::size_t current_mission_index_ = 0;
//是否正在执行任务
bool is_mission_running_ = false;

// 接收任务指令
void receiveMission(const std::string & command)
{
    // CANCEL 是特殊控制指令
    if (command == "CANCEL")
    {
        cancelMission();
        return;
    }
    // 如果已有任务正在执行，不接受新任务
    if (is_mission_running_)
    {
        publishStatus("MISSION_BUSY");
        return;
    }

    std::istringstream stream(command);
    std::string name;

    std::vector<std::string> new_queue;

    // 将输入的任务拆成多个目标名称
    while (stream >> name)
    {
        // 检查目标点是否存在
        if (waypoints_.find(name) == waypoints_.end())
        {
            publishStatus("UNKNOWN_WAYPOINT_" + name);
            return;
        }

        new_queue.push_back(name);
    }

    if (new_queue.empty())
    {
        publishStatus("EMPTY_MISSION");
        return;
    }

    if (!nav_client_->action_server_is_ready())
    {
        publishStatus("NAV2_NOT_READY");
        return;
    }

    mission_queue_ = new_queue;

    current_mission_index_ = 0;

    is_mission_running_ = true;
    cancel_requested_ = false;

    current_goal_handle_.reset();

    publishStatus("MISSION_STARTED");

    // 开始执行第一个目标
    sendGoal(mission_queue_[current_mission_index_]);
}

// 保存当前正在执行的导航目标
GoalHandle::SharedPtr current_goal_handle_;

// 是否正在请求取消任务
bool cancel_requested_ = false;

// 取消当前任务
void cancelMission()
{
    if (!is_mission_running_)
    {
        publishStatus("NO_ACTIVE_MISSION");
        return;
    }

    if (cancel_requested_)
    {
        publishStatus("CANCEL_ALREADY_REQUESTED");
        return;
    }

    cancel_requested_ = true;

    publishStatus("CANCEL_REQUESTED");

    // 如果Nav2已经接受当前目标，就请求取消
    if (current_goal_handle_)
    {
        nav_client_->async_cancel_goal(current_goal_handle_);
    }

    // 如果还没有GoalHandle，
    // 等goal_response_callback收到后再取消
}
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}