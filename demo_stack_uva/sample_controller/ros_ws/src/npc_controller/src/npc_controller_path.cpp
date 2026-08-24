#include "npc_controller.hpp"

#include <cmath>
#include <libalglib/interpolation.h>

namespace controller
{

    void ControllerNode::pure_pursuit()
    {
        /**
         * @brief This function computes the steering angle using the pure pursuit algorithm
         * @return The steering angle in radians
         */

        // Check to see if we have enough localization data.
        if (!wheel_speed_received || !position_received || !path_loaded) {return;}

        PathPoint current_position;
        current_position.x = vehicle_state_.x;
        current_position.y = vehicle_state_.y;
        current_position.z = vehicle_state_.z;
        current_position.yaw = vehicle_state_.yaw;

        int start_index = calculate_base_projections(*current_path_, current_position);

        double current_velocity = vehicle_state_.vx;
        double lookahead = std::max(min_lookahead_dist_, std::min(lookahead_gain_ * current_velocity, max_lookahead_dist_));

        PathPoint target_position = pure_pursuit_target_point(*current_path_, start_index, current_position, lookahead);

        double pursuit_vector_dx = target_position.x - current_position.x;
        double pursuit_vector_dy = target_position.y - current_position.y;

        double alpha = atan2(pursuit_vector_dy, pursuit_vector_dx) - vehicle_state_.yaw;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));
        double delta = atan((2 * wheelbase_ * sin(alpha)) / lookahead);

        // Update the steering angle.
        pure_pursuit_steering_angle = delta;

        debug_msg_.pp_current_x = current_position.x;
        debug_msg_.pp_current_y = current_position.y;
        debug_msg_.pp_current_yaw = current_position.yaw;
        debug_msg_.pp_target_x = target_position.x;
        debug_msg_.pp_target_y = target_position.y;
        debug_msg_.pp_lookahead = lookahead;
        debug_msg_.pp_alpha = alpha;
        debug_msg_.pp_delta = delta;

        // Enhanced logging for deviations and crashes
        static int log_counter = 0;
        log_counter++;

        // Log critical data every 10 cycles (~1Hz at 10Hz control loop)
        if (log_counter % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[CONTROL] s=%.1f opt_s=%.1f dist=%.3fm err=%.3fm | v=%.1f mph delta=%.3f rad lookahead=%.1f | state=%d",
                center_line_s_, optimal_line_s_, optimal_line_distance_, optimal_line_signed_error_,
                current_velocity * 2.237, delta, lookahead, static_cast<int>(lap_state_inputs_.lap_state));
        }

        // Alert if deviation is excessive (> 1.5 meters from optimal line)
        if (optimal_line_distance_ > 1.5) {
            RCLCPP_WARN(this->get_logger(),
                "[DEVIATION WARNING] Large deviation from optimal line: %.2f m at s=%.1f | signed_err=%.3f | pos=(%.2f, %.2f) | target=(%.2f, %.2f)",
                optimal_line_distance_, optimal_line_s_, optimal_line_signed_error_,
                current_position.x, current_position.y, target_position.x, target_position.y);
        }

        // Alert if steering angle exceeds safe threshold (> 0.3 radians = ~17 degrees)
        if (std::abs(delta) > 0.3) {
            RCLCPP_WARN(this->get_logger(),
                "[STEERING ALERT] High steering angle: %.3f rad (%.1f deg) | alpha=%.3f | dist=%.2f m | lookahead=%.1f m",
                delta, delta * 57.2958, alpha, optimal_line_distance_, lookahead);
        }

        // Visualize the pure pursuit base and target points
        base_point_msg_.header.stamp = this->now();
        base_point_msg_.header.frame_id = "map";
        base_point_msg_.point.x = current_path_->points[start_index].x;
        base_point_msg_.point.y = current_path_->points[start_index].y;

        target_point_msg_.header.stamp = this->now();
        target_point_msg_.header.frame_id = "map";
        target_point_msg_.point.x = target_position.x;
        target_point_msg_.point.y = target_position.y;

        base_point_pub_->publish(base_point_msg_);
        target_point_pub_->publish(target_point_msg_);
    }

    int ControllerNode::calculate_base_projections(const Path &path, const PathPoint &current_position)
    {
        // Calculate start index. Find the closest point to us.
        int start_index = 0;
        double best_distance = 1000000;
        for (unsigned int i = 0; i < path.points.size(); i++)
        {
            double dx = path.points[i].x - current_position.x;
            double dy = path.points[i].y - current_position.y;
            double distance = sqrt(dx * dx + dy * dy);
            if (distance < best_distance)
            {
                best_distance = distance;
                start_index = i;
            }
        }
        return start_index;
    }

    PathPoint ControllerNode::pure_pursuit_target_point(const Path &path, int start_index, const PathPoint &position, double lookahead) const
    {
        int min_ind = 0;
        double min_diff = 1000000.0;
        // todo: make better w/ interpolation between indices
        // the path.points.size()/2 is to make sure we don't accidentally look behind ourselves
        for (unsigned int i = start_index; i < path.points.size() / 2 + start_index; i++)
        {
            double dx = (path.points[i % path.points.size()].x - position.x);
            double dy = (path.points[i % path.points.size()].y - position.y);
            // Finds the point that is along the circle at a distance `lookahead` away.
            double diff = abs(lookahead - sqrt(dx * dx + dy * dy));
            if (diff < min_diff)
            {
                min_diff = diff;
                min_ind = i;
            }
        }
        return path.points[min_ind % path.points.size()];
    }

    Path ControllerNode::load_path(std::string filename)
    {
        std::vector<std::vector<double>> path_data = read_csv(filename, 4, ",");

        alglib::spline1dinterpolant x_interpolator, y_interpolator;

        // Calculate Arc Length
        std::vector<double> arc_length;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
        arc_length.push_back(0.0);
        x.push_back(path_data[0][0]);
        y.push_back(path_data[0][1]);

        for (unsigned int i = 1; i < path_data[0].size(); i++)
        {
            x.push_back(path_data[0][i]);
            y.push_back(path_data[1][i]);
            double dx = path_data[0][i] - path_data[0][i - 1];
            double dy = path_data[1][i] - path_data[1][i - 1];
            double ds = std::sqrt(dx * dx + dy * dy);
            arc_length.push_back(arc_length[i - 1] + ds);
        }

        // Generate Splines
        alglib::real_1d_array xs, ys, s;
        xs.setcontent(x.size(), x.data());
        ys.setcontent(y.size(), y.data());
        s.setcontent(arc_length.size(), arc_length.data());

        alglib::spline1dbuildlinear(s, xs, x_interpolator);
        alglib::spline1dbuildlinear(s, ys, y_interpolator);

        double xp, yp, dxp, dyp, ddxp, ddyp;
        std::vector<double> s_sample;
        double track_length = arc_length[arc_length.size() - 1];
        Path path;
        for (double i = 0; i < track_length; i++)
        {
            alglib::spline1ddiff(x_interpolator, i, xp, dxp, ddxp);
            alglib::spline1ddiff(y_interpolator, i, yp, dyp, ddyp);
            PathPoint point;
            point.s = i;
            point.x = xp;
            point.y = yp;
            point.z = 0.0;
            point.yaw = std::atan2(dyp, dxp);
            path.points.push_back(point);
        }

        return path;
    }

} // namespace controller