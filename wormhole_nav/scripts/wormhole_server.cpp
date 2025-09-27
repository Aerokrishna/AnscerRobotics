#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "djkstra.cpp"
#include <my_robot_interfaces/action/nav_goal.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/load_map.hpp"

using NavGoal = my_robot_interfaces::action::NavGoal;
using NavGoalGoalHandle = rclcpp_action::ServerGoalHandle<NavGoal>;
using namespace std::placeholders;
using namespace std::chrono_literals; 

// It navigates a robot from point A to point B through multiple maps
// This is an action server, accepts action requests in the form of target pose and target map
// If the current map is not the target map
// Creates a higher level plan by creating shortest path through maps, by referring to the databse of wormholes
// It then queries the wormhole positions to generate waypoints 
// these waypoints are sent as goals to Nav2 through the API
// Each time the map switches, the updated map is loaded and the initial pose is published for localization

class NavGoalServerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavGoalServerNode() : Node("nav_goal_server")
    {   
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // action server for NavGoal
        nav_goal_server_ = rclcpp_action::create_server<NavGoal>(
            this,
            "nav_goal",
            std::bind(&NavGoalServerNode::goal_callback, this, _1, _2),
            std::bind(&NavGoalServerNode::cancel_callback, this, _1),
            std::bind(&NavGoalServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );

        // publisher for resetting robot's initial pose when switch in map
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // client to load map
        map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

        RCLCPP_INFO(this->get_logger(), "NavGoal action server has started");
        
        // maps.db is the database which contains the table wormhole with the columns
        std::string package_share = ament_index_cpp::get_package_share_directory("wormhole_nav");
        std::string db_path = package_share + "/scripts/maps.db";

        std::string package_share2 = ament_index_cpp::get_package_share_directory("tb3_nav2");
        map_yaml_path = package_share2 + "/maps/"; 

        // open wormhole database
        if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open DB: %s", sqlite3_errmsg(db_));
            db_ = nullptr; 
        }

        // create action client for Nav2's NavigateToPose to use the api
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        while (!client_ptr_->wait_for_action_server(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for NavigateToPose action server...");
        }
    }

private:

    // struct to represent wormhole waypoints and map transitions
    struct wormhole_waypoints {
        std::string from_map;
        std::string to_map;
        float wh_x;
        float wh_y;
        float wh_theta;
    };

    // this function retrives the wormhole coordinates given the from_map and to_map
    std::vector<float> get_waypoint(sqlite3* db, const std::string &from_map_id, const std::string &to_map_id) {
        sqlite3_stmt *stmt;
        const char *sql = 
            "SELECT from_wormhole_x, from_wormhole_y "
            "FROM wormholes "
            "WHERE from_map_id = ? AND to_map_id = ?;";

        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(db) << "\n";
            return {};
        }

        sqlite3_bind_text(stmt, 1, from_map_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, to_map_id.c_str(), -1, SQLITE_TRANSIENT);

        std::vector<float> result;
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            double x = sqlite3_column_double(stmt, 0); 
            double y = sqlite3_column_double(stmt, 1); 
            result = {static_cast<float>(x), static_cast<float>(y), 0.0f};
        }

        sqlite3_finalize(stmt);
        return result;
    }

    // goal callback, called whenever a goal request is received
    // it helps to reject or accepts the goal
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavGoal::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a new navigation goal");

        // reject if another goal is active
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_ && goal_handle_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "A goal is active, rejecting new goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
  
        std::string start_map = "mapA"; // assume current map is mapA
        std::string goal_map = goal->map_name;

        final_goal_point.from_map = goal->map_name;
        final_goal_point.wh_x = goal->robot_x;
        final_goal_point.wh_y = goal->robot_y;
        final_goal_point.wh_theta = goal->robot_theta;

        // run greedy dijkstra to compute shortest map path
        double start_x = 0.0, start_y = 0.0;
        auto path = dijkstra(db_, start_map, goal_map, start_x, start_y);

        if (path.empty()) {
            RCLCPP_INFO(this->get_logger(), "No path found, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        map_path = path;

        std::string joined;
        for (const auto &m : map_path) {
            joined += m + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Accepting goal. Planned path: %s", joined.c_str());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // cancel callback called when cancel request is received
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<NavGoalGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // called when goal is accepted, execute callback is called here
    void handle_accepted_callback(const std::shared_ptr<NavGoalGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing accepted goal");
        execute_goal(goal_handle);
    }

     // main function to execute the function of the action server
     // loops through the map path
     // updates the map and initial position of the robot
     // sends these goals to nav2 
    void execute_goal(const std::shared_ptr<NavGoalGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        static size_t idx = 0;

        while (idx < map_path.size() - 1) {
            goal_point.from_map = map_path[idx];
            goal_point.to_map = map_path[idx + 1];

            auto waypoint = get_waypoint(db_, goal_point.from_map, goal_point.to_map);
            goal_point.wh_x = waypoint[0];
            goal_point.wh_y = waypoint[1];
            goal_point.wh_theta = 0.0;  
            
            if (idx == 0) {
                // load first map and initial pose
                std::string new_map = map_yaml_path + goal_point.from_map;
                change_map_and_reset_pose(new_map, 0.0, 1.5);
            }

            RCLCPP_INFO(this->get_logger(), "Current Map: %s", goal_point.from_map.c_str());
            RCLCPP_INFO(this->get_logger(), "Current Goal (x=%.2f, y=%.2f)", 
                        goal_point.wh_x, goal_point.wh_y);

            ++idx;

            // send goal to Nav2
            send_nav2_goal(goal_point);

            // load next map and set initial pose
            std::string new_map = map_yaml_path + goal_point.to_map;
            auto initial_pose = get_waypoint(db_, goal_point.to_map, goal_point.from_map);
            change_map_and_reset_pose(new_map, initial_pose[0], initial_pose[1]);
        }

        RCLCPP_INFO(this->get_logger(), "Final Map: %s", final_goal_point.from_map.c_str());
        RCLCPP_INFO(this->get_logger(), "Final Goal (x=%.2f, y=%.2f)", 
                    final_goal_point.wh_x, final_goal_point.wh_y);

        send_nav2_goal(final_goal_point);

        // success
        auto result = std::make_shared<NavGoal::Result>();
        result->status = "SUCCESS";
        goal_handle->succeed(result);
    }

     // Sends navigation goal to NavigateToPose action server and waits until done
     // feedback callback needs to be integrated
    void send_nav2_goal(struct wormhole_waypoints nav2goal) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";  
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.pose.position.x = nav2goal.wh_x;
        goal_msg.pose.pose.position.y = nav2goal.wh_y;
        goal_msg.pose.pose.orientation.w = 1.0;

        bool goal_finished = false;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [&goal_finished](const GoalHandleNavigateToPose::WrappedResult & result) {
                goal_finished = true;
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded!");
                else
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal failed");
            };

        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while (!goal_finished && rclcpp::ok()) {}
    }

    // loads new map by taking in the yaml file location and intializes the pose in the new map frame
    void change_map_and_reset_pose(const std::string &map_file, float x, float y) {
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_file + ".yaml";
        map_client_->async_send_request(request);

        RCLCPP_INFO(get_logger(), "Loaded new map: %s", map_file.c_str());

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.orientation.w = 1.0;
        initial_pose_pub_->publish(pose_msg);
    }

    rclcpp_action::Server<NavGoal>::SharedPtr nav_goal_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::shared_ptr<NavGoalGoalHandle> goal_handle_;
    sqlite3* db_;
    std::vector<std::string> map_path;
    struct wormhole_waypoints goal_point;
    struct wormhole_waypoints final_goal_point;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    std::string map_yaml_path;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavGoalServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
