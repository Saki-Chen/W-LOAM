#pragma once
#include <vector>
#include <string> 
#include <cmath>
namespace odometry
{
    using WheelCount = int64_t;

    class LookupTable
    {
    public:
        LookupTable(const int n, const double *break_points, const double *values);

        double Lookup(double x) const;

    private:
        std::vector<double> _break_points, _values;
    }; // class LookupTable

    class WheelOdometry
    {

    public:
        WheelOdometry(const std::string &json_params_file_path);

        ~WheelOdometry()
        {
            delete _vehicle_param.lookup_curvature_forward;
            delete _vehicle_param.lookup_curvature_backward;
        }

        bool isNewPose() const
        {
            return _is_new_pose;
        }

        bool isNewTwist() const
        {
            return _is_new_twist;
        }

        void updateStampManual(const uint64_t& stamp)
        {
            _stamp = stamp;
            _is_new_pose = true;
        }

        void getPose(double &x, double &y, double &yaw, uint64_t& stamp)
        {
            x = _x;
            y = _y;
            yaw = _yaw;
            stamp = _stamp;
            _is_new_pose = false;
        }

        void getPose(double &x, double &y, double &yaw, double &distance, uint64_t& stamp)
        {
            x = _x;
            y = _y;
            yaw = _yaw;
            distance = _distance;
            stamp = _stamp;
            _is_new_pose = false;
        }

        void getTwist(double &local_v, double &local_yaw_rate, uint64_t &stamp)
        {
            local_v = _local_v;
            local_yaw_rate = _local_yaw_rate;
            stamp = _stamp_mid;
            _is_new_twist = false;
        }

        void updateSteeringAngle(const double steering_angle)
        {
            _chasis_status.steering_angle = steering_angle;
            _status_flag |= STEERING_ANGLE_READY;
        }

        void updateDirection(const bool is_forward)
        {
            _chasis_status.is_forward = is_forward;
            _status_flag |= IS_FORWARD_READY;
        }

        bool isReady() const { return _status_flag == ALL_READY; }

        void updateWheelingCount(WheelCount FL, WheelCount FR, WheelCount RL, WheelCount RR, const uint64_t stamp);

        void setSlidingWindow(double duration) {_sliding_time = duration;}

        void getAngularResolution(double& front_res, double& rear_res) const
        {
            front_res = M_PI * 2 / _vehicle_param.front_wheel_count_full;
            rear_res = M_PI * 2 / _vehicle_param.rear_wheel_count_full;
        }

    private:
        double _x = 0;
        double _y = 0;
        double _yaw = 0;
        double _distance = 0;
        double _local_v = 0;
        double _local_yaw_rate = 0;
        struct
        {
            WheelCount wheeling_count_curr[4];
            WheelCount wheeling_count_last[4];
            double steering_angle;
            bool is_forward;
        } _chasis_status;
        enum
        {
            ALL_READY = 0b111,
            WHEELING_COUNT_READY = 0b001,
            STEERING_ANGLE_READY = 0b010,
            IS_FORWARD_READY = 0b100,
        };
        uint8_t _status_flag = 0;

        struct
        {
            double wheel_base = -1;
            double front_wheel_radius;
            double rear_wheel_radius;
            double front_wheel_track;
            double rear_wheel_track;
            double count_period;
            double enable_vec_fl_fr_rl_rr[4];
            int front_wheel_count_full;
            int rear_wheel_count_full;
            LookupTable *lookup_curvature_forward = nullptr;
            LookupTable *lookup_curvature_backward = nullptr;
            bool check() const
            {
                return wheel_base > 0 && front_wheel_track > 0 && rear_wheel_track > 0 && front_wheel_radius > 0 && rear_wheel_radius > 0 && front_wheel_count_full > 0 && rear_wheel_count_full > 0 && count_period > 0;
            }
        } _vehicle_param;

        // time stamp in nanosecond
        uint64_t _stamp = 0;
        uint64_t _stamp_mid = 0;

        // sliding window size for speed and yaw_rate estimation
        double _sliding_time = 0.2; // second

        bool _is_new_pose = false;
        bool _is_new_twist = false;
        
    }; // class WheelOdometry
} // namespace odometry
