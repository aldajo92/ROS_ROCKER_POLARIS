#ifndef PATH_TRACKING_CONTROLLER_H
#define PATH_TRACKING_CONTROLLER_H

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <utility>
#include <cmath>
#include <functional>
#include <chrono>

using ControllerActionCallback = std::function<void(double, double)>;

class AbstractPathTrackingController
{
public:
    virtual ~AbstractPathTrackingController() = default;

    virtual void setPath(const nav_msgs::Path &path) = 0;
    virtual void setOdom(const nav_msgs::Odometry &odom) = 0;

    virtual void computeControlCommand() = 0;
    virtual void stopRobot() = 0;

    virtual bool isReachGoal() const = 0;
    virtual bool isOdomReceived() const = 0;
    virtual bool isPathReceived() const = 0;

protected:
    ControllerActionCallback controller_action_callback_;
};

class PathTrackingController : public AbstractPathTrackingController
{
public:
    PathTrackingController(
        double lookahead_distance,
        double max_linear_speed,
        double max_angular_speed,
        double goal_tolerance,
        ControllerActionCallback controller_action_callback);

    void setPath(const nav_msgs::Path &path);
    void setOdom(const nav_msgs::Odometry &odom);

    void computeControlCommand();
    void stopRobot();

    std::pair<double, int> calculateMinDistanceInPath(
        const std::vector<double> &position,
        const nav_msgs::Path &path);

    std::vector<double> computeControlAction(
        const std::vector<double> &position,
        const std::vector<double> &lookahead_point);

    bool isReachGoal() const;

    bool isOdomReceived() const;
    bool isPathReceived() const;

    double getYawFromQuaternion(const geometry_msgs::Quaternion &q);

private:
    nav_msgs::Path path_;
    nav_msgs::Odometry current_odometry_;
    double lookahead_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;

    ControllerActionCallback controller_action_callback_;

    bool path_received_;
    bool odom_received_;

    geometry_msgs::Point starting_point_;
    bool starting_point_set_;

    std::chrono::duration<double> log_interval_;
    std::chrono::time_point<std::chrono::system_clock> last_log_time_;
};

#endif // PATH_TRACKING_CONTROLLER_H
