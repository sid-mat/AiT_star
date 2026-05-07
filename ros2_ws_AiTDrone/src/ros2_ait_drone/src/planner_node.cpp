/**
 * planner_node.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * ROS2 node wrapping OMPL for 3D drone path planning.
 * Supports AIT*, RRT*, and Informed RRT* selectable at runtime.
 *
 * Topics in:
 *   /goal_pose          (geometry_msgs/PoseStamped)  - planning goal
 *   /drone_pose         (geometry_msgs/PoseStamped)  - current drone position
 *   /dynamic_obstacles  (geometry_msgs/PoseArray)    - obstacle centres;
 *                          position = centre, orientation.w = radius
 *   /switch_planner     (std_msgs/String)            - "ait" | "rrt" | "informed"
 *
 * Topics out:
 *   /planned_path       (nav_msgs/Path)              - path for follower node
 *   /planning_viz       (visualization_msgs/MarkerArray) - tree + path + labels
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <mutex>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// ── Static obstacle (axis-aligned box) ────────────────────────────────────────
struct StaticBox {
    double cx, cy, cz;   // centre
    double hx, hy, hz;   // half-extents
};

// ── Dynamic obstacle (sphere) ─────────────────────────────────────────────────
struct DynSphere {
    double x, y, z, radius;
};

// ── Planner colour palette ────────────────────────────────────────────────────
struct RGB { float r, g, b; };
static const std::map<std::string, RGB> COLORS = {
    {"ait",      {0.65f, 0.15f, 1.00f}},   // purple
    {"rrt",      {0.15f, 0.60f, 1.00f}},   // blue
    {"informed", {1.00f, 0.60f, 0.10f}},   // orange
};

// ─────────────────────────────────────────────────────────────────────────────

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : Node("ait_planner_node")
    {
        // Parameters
        declare_parameter("drone_radius",  0.35);
        declare_parameter("planning_time", 2.0);
        declare_parameter("replan_hz",     2.0);
        declare_parameter("planner",       std::string("ait"));
        declare_parameter("x_min", -10.0); declare_parameter("x_max", 10.0);
        declare_parameter("y_min", -10.0); declare_parameter("y_max", 10.0);
        declare_parameter("z_min",   0.5); declare_parameter("z_max",  8.0);

        drone_radius_  = get_parameter("drone_radius").as_double();
        planning_time_ = get_parameter("planning_time").as_double();
        active_planner_= get_parameter("planner").as_string();

        // Workspace bounds
        double xlo = get_parameter("x_min").as_double();
        double xhi = get_parameter("x_max").as_double();
        double ylo = get_parameter("y_min").as_double();
        double yhi = get_parameter("y_max").as_double();
        double zlo = get_parameter("z_min").as_double();
        double zhi = get_parameter("z_max").as_double();

        // Static obstacles in the scene (match Webots world)
        static_obstacles_ = {
            {  0.0,  4.0, 2.0,   0.3, 0.3, 2.0 },   // vertical wall shard
            { -4.0,  0.0, 1.75,  0.3, 2.5, 1.75},   // L-wall
            {  5.0, -4.0, 2.5,   0.3, 3.0, 2.5 },   // right barrier
            {  2.0,  2.0, 1.0,   1.5, 0.3, 1.0 },   // horizontal beam
        };

        // OMPL state space R3
        // space_ = std::make_shared<ob::RealVectorStateSpace>(3);
        // ob::RealVectorBounds bounds(3);
        auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, xlo); bounds.setHigh(0, xhi);
        bounds.setLow(1, ylo); bounds.setHigh(1, yhi);
        bounds.setLow(2, zlo); bounds.setHigh(2, zhi);
        rvss->setBounds(bounds);     // ← typed pointer, compiles fine
        space_ = rvss;               // ← assign back to base class member

        // ROS2
        using std::placeholders::_1;
        goal_sub_  = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PlannerNode::onGoal, this, _1));
        drone_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone_pose", 10, std::bind(&PlannerNode::onDrone, this, _1));
        obs_sub_   = create_subscription<geometry_msgs::msg::PoseArray>(
            "/dynamic_obstacles", 10, std::bind(&PlannerNode::onObstacles, this, _1));
        sw_sub_    = create_subscription<std_msgs::msg::String>(
            "/switch_planner", 10, std::bind(&PlannerNode::onSwitch, this, _1));

        path_pub_  = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        viz_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("/planning_viz", 10);

        double period_ms = 1000.0 / get_parameter("replan_hz").as_double();
        replan_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&PlannerNode::checkValidity, this));

        RCLCPP_INFO(get_logger(),
            "AIT* Planner ready. Active: %s | Planning time: %.1fs",
            active_planner_.c_str(), planning_time_);
    }

private:
    // ── State ──────────────────────────────────────────────────────────────────
    double drone_radius_, planning_time_;
    std::string active_planner_;

    double dx_{-8.0}, dy_{-8.0}, dz_{1.5};   // drone pose
    double gx_{ 8.0}, gy_{ 8.0}, gz_{4.0};   // goal pose
    bool   has_goal_{false};

    std::vector<DynSphere>  dyn_obs_;
    std::vector<StaticBox>  static_obstacles_;
    std::mutex obs_mutex_;

    nav_msgs::msg::Path current_path_;
    ob::StateSpacePtr   space_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, drone_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr   obs_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           sw_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr                                     replan_timer_;

    // ── Collision checking ─────────────────────────────────────────────────────
    bool isValid(const ob::State* state) const
    {
        const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
        double x = s->values[0], y = s->values[1], z = s->values[2];

        // Static box obstacles
        for (const auto& box : static_obstacles_) {
            if (std::abs(x - box.cx) < box.hx + drone_radius_ &&
                std::abs(y - box.cy) < box.hy + drone_radius_ &&
                std::abs(z - box.cz) < box.hz + drone_radius_)
                return false;
        }

        // Dynamic sphere obstacles
        std::lock_guard<std::mutex> lock(const_cast<PlannerNode*>(this)->obs_mutex_);
        for (const auto& obs : dyn_obs_) {
            double d = std::sqrt((x-obs.x)*(x-obs.x) +
                                 (y-obs.y)*(y-obs.y) +
                                 (z-obs.z)*(z-obs.z));
            if (d < obs.radius + drone_radius_) return false;
        }
        return true;
    }

    // ── Planning ───────────────────────────────────────────────────────────────
    void plan()
    {
        auto si = std::make_shared<ob::SpaceInformation>(space_);
        si->setStateValidityChecker(
            [this](const ob::State* s){ return isValid(s); });
        si->setup();

        auto pdef = std::make_shared<ob::ProblemDefinition>(si);

        ob::ScopedState<> start(space_);
        start[0] = dx_; start[1] = dy_; start[2] = dz_;
        ob::ScopedState<> goal(space_);
        goal[0]  = gx_; goal[1]  = gy_; goal[2]  = gz_;

        pdef->setStartAndGoalStates(start, goal, 0.4);
        pdef->setOptimizationObjective(
            std::make_shared<ob::PathLengthOptimizationObjective>(si));

        // Select planner
        ob::PlannerPtr planner;
        if (active_planner_ == "rrt") {
            auto p = std::make_shared<og::RRTstar>(si);
            p->setRange(1.5);
            planner = p;
        } else if (active_planner_ == "informed") {
            auto p = std::make_shared<og::InformedRRTstar>(si);
            p->setRange(1.5);
            planner = p;
        } else {
            planner = std::make_shared<og::AITstar>(si);
        }

        planner->setProblemDefinition(pdef);
        planner->setup();

        auto t0 = std::chrono::steady_clock::now();
        ob::PlannerStatus solved = planner->solve(planning_time_);
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        if (solved) {
            auto* pg = pdef->getSolutionPath()->as<og::PathGeometric>();
            pg->interpolate(60);

            double cost = pg->length();
            RCLCPP_INFO(get_logger(), "[%s] Path found | cost=%.3f | time=%.2fs | vertices=%zu",
                active_planner_.c_str(), cost, elapsed, pg->getStateCount());

            publishPath(pg);
            publishViz(planner, si, pg);
        } else {
            RCLCPP_WARN(get_logger(), "[%s] No solution in %.1fs",
                active_planner_.c_str(), elapsed);
        }
    }

    // ── Publishers ─────────────────────────────────────────────────────────────
    void publishPath(og::PathGeometric* pg)
    {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "world";
        msg.header.stamp    = now();

        for (size_t i = 0; i < pg->getStateCount(); ++i) {
            const auto* s = pg->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = s->values[0];
            ps.pose.position.y = s->values[1];
            ps.pose.position.z = s->values[2];
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        current_path_ = msg;
        path_pub_->publish(msg);
    }

    void publishViz(ob::PlannerPtr& planner,
                    ob::SpaceInformationPtr& si,
                    og::PathGeometric* pg)
    {
        visualization_msgs::msg::MarkerArray ma;
        auto col = COLORS.count(active_planner_) ?
                   COLORS.at(active_planner_) : RGB{1,1,1};

        // ── Tree edges ─────────────────────────────────────────────────────────
        ob::PlannerData pdata(si);
        planner->getPlannerData(pdata);

        visualization_msgs::msg::Marker edges;
        edges.header.frame_id = "world";
        edges.header.stamp    = now();
        edges.ns   = "tree";
        edges.id   = 0;
        edges.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges.action = visualization_msgs::msg::Marker::ADD;
        edges.scale.x = 0.03;
        edges.color.r = col.r; edges.color.g = col.g;
        edges.color.b = col.b; edges.color.a = 0.45f;
        edges.lifetime = rclcpp::Duration(0, 0);

        for (unsigned i = 0; i < pdata.numVertices(); ++i) {
            std::vector<unsigned> children;
            pdata.getEdges(i, children);
            const auto* from =
                pdata.getVertex(i).getState()
                     ->as<ob::RealVectorStateSpace::StateType>();
            for (auto j : children) {
                const auto* to =
                    pdata.getVertex(j).getState()
                         ->as<ob::RealVectorStateSpace::StateType>();
                geometry_msgs::msg::Point p1, p2;
                p1.x = from->values[0]; p1.y = from->values[1]; p1.z = from->values[2];
                p2.x = to->values[0];   p2.y = to->values[1];   p2.z = to->values[2];
                edges.points.push_back(p1);
                edges.points.push_back(p2);
            }
        }
        ma.markers.push_back(edges);

        // ── Solution path ──────────────────────────────────────────────────────
        visualization_msgs::msg::Marker path_line;
        path_line.header.frame_id = "world";
        path_line.header.stamp    = now();
        path_line.ns   = "solution";
        path_line.id   = 1;
        path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_line.action = visualization_msgs::msg::Marker::ADD;
        path_line.scale.x = 0.14;
        path_line.color.r = 1.0f; path_line.color.g = 1.0f;
        path_line.color.b = 1.0f; path_line.color.a = 1.0f;

        for (size_t i = 0; i < pg->getStateCount(); ++i) {
            const auto* s = pg->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            geometry_msgs::msg::Point p;
            p.x = s->values[0]; p.y = s->values[1]; p.z = s->values[2];
            path_line.points.push_back(p);
        }
        ma.markers.push_back(path_line);

        // ── Start / goal spheres ───────────────────────────────────────────────
        auto sphere = [&](int id, double x, double y, double z,
                          float r, float g, float b, double scale = 0.5) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world"; m.header.stamp = now();
            m.ns = "waypoints"; m.id = id;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = scale;
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
            return m;
        };
        ma.markers.push_back(sphere(10, dx_, dy_, dz_, 0.1f, 1.0f, 0.1f));  // start: green
        ma.markers.push_back(sphere(11, gx_, gy_, gz_, 1.0f, 0.2f, 0.2f));  // goal:  red

        // ── Static obstacle boxes ──────────────────────────────────────────────
        int box_id = 20;
        for (const auto& box : static_obstacles_) {
            visualization_msgs::msg::Marker b;
            b.header.frame_id = "world"; b.header.stamp = now();
            b.ns = "static_obs"; b.id = box_id++;
            b.type = visualization_msgs::msg::Marker::CUBE;
            b.action = visualization_msgs::msg::Marker::ADD;
            b.pose.position.x = box.cx; b.pose.position.y = box.cy; b.pose.position.z = box.cz;
            b.pose.orientation.w = 1.0;
            b.scale.x = box.hx * 2; b.scale.y = box.hy * 2; b.scale.z = box.hz * 2;
            b.color.r = 0.6f; b.color.g = 0.6f; b.color.b = 0.6f; b.color.a = 0.8f;
            ma.markers.push_back(b);
        }

        // ── Dynamic obstacle spheres ───────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            int dyn_id = 50;
            for (const auto& obs : dyn_obs_) {
                ma.markers.push_back(
                    sphere(dyn_id++, obs.x, obs.y, obs.z,
                           1.0f, 0.15f, 0.15f, obs.radius * 2.0));
            }
        }

        viz_pub_->publish(ma);
    }

    // ── Path validity check (triggers replanning) ─────────────────────────────
    void checkValidity()
    {
        if (!has_goal_ || current_path_.poses.empty()) return;

        auto si = std::make_shared<ob::SpaceInformation>(space_);
        si->setStateValidityChecker(
            [this](const ob::State* s){ return isValid(s); });

        for (const auto& ps : current_path_.poses) {
            ob::ScopedState<> state(space_);
            state[0] = ps.pose.position.x;
            state[1] = ps.pose.position.y;
            state[2] = ps.pose.position.z;
            if (!si->isValid(state.get())) {
                RCLCPP_WARN(get_logger(),
                    "Dynamic obstacle invalidated path — replanning with %s",
                    active_planner_.c_str());
                current_path_.poses.clear();
                plan();
                return;
            }
        }
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        gx_ = msg->pose.position.x;
        gy_ = msg->pose.position.y;
        gz_ = msg->pose.position.z;
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "Goal set: (%.1f, %.1f, %.1f)", gx_, gy_, gz_);
        plan();
    }

    void onDrone(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        dx_ = msg->pose.position.x;
        dy_ = msg->pose.position.y;
        dz_ = msg->pose.position.z;
    }

    void onObstacles(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        dyn_obs_.clear();
        for (const auto& p : msg->poses) {
            // Convention: position = centre, orientation.w = radius
            dyn_obs_.push_back({p.position.x, p.position.y, p.position.z,
                                p.orientation.w > 0 ? p.orientation.w : 1.2});
        }
    }

    void onSwitch(const std_msgs::msg::String::SharedPtr msg) {
        active_planner_ = msg->data;
        RCLCPP_INFO(get_logger(), "Planner switched to: %s", active_planner_.c_str());
        if (has_goal_) {
            current_path_.poses.clear();
            plan();
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
