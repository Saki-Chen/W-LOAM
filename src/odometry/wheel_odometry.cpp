#include "wheel_odometry.h"

#include <queue>
#include <jsoncpp/json/json.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <algorithm>

namespace odometry
{
    LookupTable::LookupTable(const int n, const double *break_points, const double *values)
    {
        _break_points.resize(n);
        _values.resize(n);
        for (int i = 0; i < n; ++i)
        {
            _break_points[i] = *break_points++;
            _values[i] = *values++;
        }
    }

    double LookupTable::Lookup(double x) const
    {
        auto loc = std::lower_bound(_break_points.begin(), _break_points.end(), x) - _break_points.begin();
        if (loc == 0)
            loc = 1;
        if (loc == _break_points.size())
            --loc;

        const auto x1 = _break_points[loc - 1];
        const auto y1 = _values[loc - 1];
        const auto x2 = _break_points[loc];
        const auto y2 = _values[loc];

        return (y1 * (x2 - x) + y2 * (x - x1)) / (x2 - x1);
    }

    WheelOdometry::WheelOdometry(const std::string &json_params_file_path)
    {
        Json::Value params;
        Json::Reader param_reader;
        std::ifstream fs(json_params_file_path);
        if (!param_reader.parse(fs, params, false))
        {
            std::cerr << "params file opening error in WheelOdometry" << std::endl;
            fs.close();
            exit(1);
        }
        fs.close();
        _vehicle_param.wheel_base = params.get("wheel_base", -1).asDouble();
        _vehicle_param.front_wheel_count_full = params.get("front_wheel_count_full", -1).asInt();
        _vehicle_param.rear_wheel_count_full = params.get("rear_wheel_count_full", -1).asInt();
        _vehicle_param.count_period = params.get("count_period", -1).asDouble();
        _vehicle_param.front_wheel_radius = params.get("front_wheel_radius", -1).asDouble();
        _vehicle_param.rear_wheel_radius = params.get("rear_wheel_radius", -1).asDouble();
        _vehicle_param.front_wheel_track = params.get("front_wheel_track", -1).asDouble();
        _vehicle_param.rear_wheel_track = params.get("rear_wheel_track", -1).asDouble();
        const auto enable_vec_fl_fr_rl_rr = params.get("enable_vec_fl_fr_rl_rr", Json::Value::null);
        const auto steering_angle_forward = params.get("steering_angle_forward", Json::Value::null);
        const auto curvature_forward = params.get("curvature_forward", Json::Value::null);
        const auto steering_angle_backward = params.get("steering_angle_backward", Json::Value::null);
        const auto curvature_backward = params.get("curvature_backward", Json::Value::null);
        if (!_vehicle_param.check() || steering_angle_forward.size() != curvature_forward.size() || curvature_forward.size() < 2 || steering_angle_backward.size() != curvature_backward.size() || curvature_backward.size() < 2 || enable_vec_fl_fr_rl_rr.size() != 4)
        {
            std::cerr << "Vehicle Params Error" << std::endl;
            exit(1);
        }

        for (int i = 0; i < 4; ++i)
        {
            _vehicle_param.enable_vec_fl_fr_rl_rr[i] = enable_vec_fl_fr_rl_rr[i].asDouble();
        }

        std::vector<double> c_f(curvature_forward.size());
        std::vector<double> s_f(steering_angle_forward.size());
        std::vector<double> c_b(curvature_backward.size());
        std::vector<double> s_b(steering_angle_backward.size());

        for (int i = 0; i < c_f.size(); ++i)
        {
            s_f[i] = steering_angle_forward[i].asDouble();
            c_f[i] = curvature_forward[i].asDouble();
        }

        for (int i = 0; i < c_b.size(); ++i)
        {
            s_b[i] = steering_angle_backward[i].asDouble();
            c_b[i] = curvature_backward[i].asDouble();
        }

        _vehicle_param.lookup_curvature_forward = new LookupTable(c_f.size(), s_f.data(), c_f.data());
        _vehicle_param.lookup_curvature_backward = new LookupTable(c_b.size(), s_b.data(), c_b.data());
    }

    void WheelOdometry::updateWheelingCount(WheelCount FL, WheelCount FR, WheelCount RL, WheelCount RR, const uint64_t stamp)
    {
        if (!(_status_flag & WHEELING_COUNT_READY))
        {
            _status_flag |= WHEELING_COUNT_READY;
            _chasis_status.wheeling_count_last[0] = FL;
            _chasis_status.wheeling_count_last[1] = FR;
            _chasis_status.wheeling_count_last[2] = RL;
            _chasis_status.wheeling_count_last[3] = RR;
            _stamp = stamp;
            _stamp_mid = stamp;
            return;
        }
        _chasis_status.wheeling_count_curr[0] = FL;
        _chasis_status.wheeling_count_curr[1] = FR;
        _chasis_status.wheeling_count_curr[2] = RL;
        _chasis_status.wheeling_count_curr[3] = RR;

        if (_status_flag != ALL_READY)
        {
            return;
        }
        Eigen::Map<Eigen::Array<WheelCount, 4, 1>> curr_count{_chasis_status.wheeling_count_curr};
        Eigen::Map<Eigen::Array<WheelCount, 4, 1>> last_count{_chasis_status.wheeling_count_last};

        Eigen::Array<double, 4, 1> count_diff = (curr_count - last_count).cast<double>();

        double dt = (stamp - _stamp) * 1e-9;

        if((count_diff.sum() == 0) && (dt < 0.1)) return;

        const Eigen::Map<Eigen::Array<double, 4, 1>> enable_vec{&_vehicle_param.enable_vec_fl_fr_rl_rr[0]};

        const LookupTable *ltable = _vehicle_param.lookup_curvature_backward;
        if (_chasis_status.is_forward)
        {
            ltable = _vehicle_param.lookup_curvature_forward;
        }

        const auto curvatue = ltable->Lookup(_chasis_status.steering_angle);
        const auto half_Df_c = 0.5 * _vehicle_param.front_wheel_track * curvatue;
        const auto half_Dr_c = 0.5 * _vehicle_param.rear_wheel_track * curvatue;
        const auto wheelbase_c = _vehicle_param.wheel_base * curvatue;
        const auto coeff_front = 2 * M_PI / _vehicle_param.front_wheel_count_full * _vehicle_param.front_wheel_radius;
        const auto coeff_rear = 2 * M_PI / _vehicle_param.rear_wheel_count_full * _vehicle_param.rear_wheel_radius;


        for (int i = 0; i < 4; ++i)
        {
            if (count_diff[i] > _vehicle_param.count_period / 2)
                count_diff[i] -= _vehicle_param.count_period;
            else if (count_diff[i] < -_vehicle_param.count_period / 2)
                count_diff[i] += _vehicle_param.count_period;
        }

        // FL FR RL RR
        const Eigen::Array<double, 4, 1> factor{coeff_front / Eigen::Vector2d{1.0 - half_Df_c, wheelbase_c}.norm(),
                                                coeff_front / Eigen::Vector2d{1.0 + half_Df_c, wheelbase_c}.norm(),
                                                coeff_rear / (1.0 - half_Dr_c),
                                                coeff_rear / (1.0 + half_Dr_c)};

        double ds = (enable_vec * count_diff * factor).sum() / enable_vec.sum();

        if (!_chasis_status.is_forward)
            ds *= -1;

        const double dyaw = ds * curvatue;
        _x += ds * std::cos(_yaw);
        _y += ds * std::sin(_yaw);
        _yaw += dyaw;
        _distance += std::abs(ds);


        static std::queue<std::array<double, 3>> diffrencial;
        static double integrated_yaw = 0, integrated_s = 0, integrated_t = 0;

        diffrencial.push({dyaw, ds ,dt});
        integrated_yaw += dyaw;
        integrated_s += ds;
        integrated_t += dt;
        if(integrated_t > _sliding_time)
        {
            _local_yaw_rate = integrated_yaw / integrated_t;
            _local_v = integrated_s / integrated_t;
            _stamp_mid = stamp - 5e8 * integrated_t;
            _is_new_twist = true;
            while(integrated_t > _sliding_time)
            {
                const auto old_data = diffrencial.front();
                diffrencial.pop();
                integrated_yaw -= old_data[0];
                integrated_s -= old_data[1];
                integrated_t -= old_data[2];
            }
        }
        // _local_yaw_rate = dyaw / dt;
        // _local_v = ds / dt;
        // _stamp_mid = (stamp - _stamp) / 2 + _stamp;

        last_count = curr_count;
        _stamp = stamp;
        _is_new_pose = true;
    }
} // namespace odometry