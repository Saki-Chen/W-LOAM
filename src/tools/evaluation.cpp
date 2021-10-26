#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <map>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>

void load_tum(const std::string& path, std::vector<tf::StampedTransform>& out)
{
    std::ifstream in(path);
    std::string line;
    std::stringstream ss;
    double t, x, y, z, qx, qy, qz, qw;
    tf::StampedTransform T;
    while(std::getline(in, line))
    {
        if(line == "") continue;
        ss.clear();
        ss.str(line);
        ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw;
        // z=0;
        T.stamp_.fromSec(t);
        T.setOrigin(tf::Vector3(x, y, z));
        T.setRotation(tf::Quaternion(qx, qy, qz, qw));
        // double roll ,pitch, yaw;
        // tf::Matrix3x3(T.getRotation()).getRPY(roll, pitch, yaw);
        // T.setRotation(tf::Quaternion(yaw,0,0));
        out.push_back(T);
    }
    std::sort(out.begin(), out.end(), [](const tf::StampedTransform& u, const tf::StampedTransform& v){return u.stamp_ <= v.stamp_;});
    std::cout << "load " << out.size() << " poses from " << path << "\n";
}

bool inter_transform(const ros::Time& stamp, const std::vector<tf::StampedTransform>& sequence, const double dt_tolerance, tf::StampedTransform& out)
{
    auto p = std::upper_bound(sequence.begin(), sequence.end(), stamp, [](const ros::Time& stamp, const tf::StampedTransform& T){return stamp < T.stamp_;});
    if(p == sequence.end() || p == sequence.begin()) return false;
    const auto& pre = *(p - 1);
    const auto& next = *p;
    const double dt_pre = (stamp - pre.stamp_).toSec();
    const double dt_next = (next.stamp_ - stamp).toSec();
    if(dt_pre + dt_next > dt_tolerance) return false;
    out.stamp_ = stamp;
    const double f = dt_pre / (dt_pre + dt_next);
    out.setOrigin(pre.getOrigin() + f * (next.getOrigin() - pre.getOrigin()));
    out.setRotation(pre.getRotation().slerp(next.getRotation(), f));
    return true;
}

void calc_rpe(const tf::StampedTransform& ref_i, const tf::StampedTransform& ref_j, const tf::StampedTransform& odom_i, const tf::StampedTransform& odom_j, tf::StampedTransform& out)
{
    out.setData(ref_j.inverse() * ref_i * odom_i.inverse() * odom_j);
}

bool calc_drift(const std::vector<tf::StampedTransform>& odom, const std::vector<tf::StampedTransform>& ref, const double dt_tolerance, tf::StampedTransform& out)
{
    tf::StampedTransform start_pose_odom, end_pose_odom, start_pose_ref, end_pose_ref;
    auto p = odom.begin();
    while (p != odom.end())
    {
        if(inter_transform(p->stamp_, ref, dt_tolerance, start_pose_ref))
        {
            start_pose_odom = *p;
            break;
        }
        ++p;
    }

    if(p == odom.end()) return false;
    
    auto rp = odom.rbegin();
    while (rp != odom.rend())
    {
        if(inter_transform(rp->stamp_, ref, dt_tolerance, end_pose_ref))
        {
            end_pose_odom = *rp;
            break;
        }
        ++rp;
    }

    if(rp == odom.rend()) return false;

    if(start_pose_odom.stamp_ >= end_pose_odom.stamp_) return false;

    std::cout << std::setprecision(3);
    std::cout << "ref time range: [" << start_pose_ref.stamp_.toSec() << ", " << end_pose_ref.stamp_.toSec() << "] " << " total: " << (end_pose_ref.stamp_ - start_pose_ref.stamp_).toSec() << " second\n";
    std::cout << "odom time range: [" << start_pose_odom.stamp_.toSec() << ", " << end_pose_odom.stamp_.toSec() << "]" << " total: " << (end_pose_odom.stamp_ - start_pose_odom.stamp_).toSec() << " second\n";
    std::cout << std::setprecision(6);
    std::cout << "start ref: [" << start_pose_ref.getOrigin().x() << ", " << start_pose_ref.getOrigin().y() << ", "<< start_pose_ref.getOrigin().z() << "]\n"; 
    std::cout << "end ref: [" << end_pose_ref.getOrigin().x() << ", " << end_pose_ref.getOrigin().y() << ", "<< end_pose_ref.getOrigin().z() << "]\n"; 
    std::cout << "start odom: [" << start_pose_odom.getOrigin().x() << ", " << start_pose_odom.getOrigin().y() << ", "<< start_pose_odom.getOrigin().z() << "]\n"; 
    std::cout << "end odom: [" << end_pose_odom.getOrigin().x() << ", " << end_pose_odom.getOrigin().y() << ", "<< end_pose_odom.getOrigin().z() << "]\n"; 

    calc_rpe(start_pose_ref, end_pose_ref, start_pose_odom, end_pose_odom, out);

    return true;

}

struct DriftInfo
{
    double distance;
    double seg_length;
    tf::StampedTransform rpe;

    DriftInfo(){}
    DriftInfo(const double dist, const double seg_len, const tf::StampedTransform& T) : distance(dist), seg_length(seg_len), rpe(T){}
}; // struct DriftInfo

void calc_path_distance(const std::vector<tf::StampedTransform>& odom, std::vector<double>& distance)
{
    const auto size = odom.size();
    distance.resize(size, 0);
    for(int i = 1; i < size; ++i)
    {
        distance[i] = distance[i-1] + (odom[i].getOrigin() - odom[i-1].getOrigin()).length();
    }

    std::cout << "total distance: " <<distance.back() << " m\n";

}

tf::Transform convert_to_2d(const tf::Transform& T_3d)
{
    double roll, pitch, yaw;
    tf::Transform T_2d;
    tf::Matrix3x3(T_3d.getRotation()).getRPY(roll, pitch, yaw);
    T_2d.setRotation(tf::createQuaternionFromRPY(0, 0, yaw));
    T_2d.setOrigin(tf::Vector3(T_3d.getOrigin().x(), T_3d.getOrigin().y(), 0));
    return T_2d;
}

void calc_ate(const std::vector<tf::StampedTransform>& odom, const std::vector<tf::StampedTransform>& ref, const double dt_tolerance)
{
    std::vector<tf::StampedTransform> odom_sub, ref_sub;
    tf::StampedTransform T_correspond;
    for(const auto& T : odom)
    {
        if(!inter_transform(T.stamp_, ref, dt_tolerance, T_correspond)) continue;
        odom_sub.push_back(T);
        ref_sub.push_back(T_correspond);
    }
    const auto size = odom_sub.size();
    std::cout << "find " << size << " corresponding poses\n";
    Eigen::Matrix<float, 3, Eigen::Dynamic> src(3, size);
    Eigen::Matrix<float, 3, Eigen::Dynamic> dist(3, size);
    for(int i = 0; i < size; ++i)
    {
        src(0,i) = odom_sub[i].getOrigin().x();
        src(1,i) = odom_sub[i].getOrigin().y();
        src(2,i) = odom_sub[i].getOrigin().z();
        dist(0,i) = ref_sub[i].getOrigin().x();
        dist(1,i) = ref_sub[i].getOrigin().y();
        dist(2,i) = ref_sub[i].getOrigin().z();
    }

    tf::Transform transform;
    tf::poseEigenToTF(Eigen::Isometry3f(Eigen::umeyama(src, dist, false)).cast<double>(), transform);
    tf::Quaternion q(transform.getRotation());

    std::cout << "aligned translation: [" << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << "]\n"
              << "aligned rotation: [" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "]\n";
    
    double dist_err = 0, dist_err_2d = 0;
    double rotation_err = 0, rotation_err_2d = 0;
    for(int i = 0; i < size; ++i)
    {
        const auto rel_pose = ref_sub[i].inverse() * transform * odom_sub[i];
        dist_err += rel_pose.getOrigin().length();
        rotation_err += rel_pose.getRotation().getAngle();

        const auto rel_pose_2d = convert_to_2d(ref_sub[i]).inverse() * convert_to_2d(transform * odom_sub[i]);
        dist_err_2d += rel_pose_2d.getOrigin().length();
        rotation_err_2d += rel_pose_2d.getRotation().getAngle();
    }
    dist_err /= size;
    rotation_err = 180 / M_PI * rotation_err / size;
    dist_err_2d /= size;
    rotation_err_2d = 180 / M_PI * rotation_err_2d / size;

    std::cout << "ATE: " << dist_err << " m\n";
    std::cout << "ARE: " << rotation_err <<" deg\n";
    std::cout << "ATE(2D): " << dist_err_2d << " m\n";
    std::cout << "ARE(2D): " << rotation_err_2d << " deg\n";
}

void calc_rpe_by_segment(const std::vector<tf::StampedTransform>& odom, const std::vector<double>& distance,const std::vector<tf::StampedTransform>& ref, const double dt_tolerance, const double seg_length, std::vector<DriftInfo>& out)
{
    const auto size = odom.size();

    const auto find_odom = [&](const double dist)->size_t
    {
        auto p = std::lower_bound(distance.begin(), distance.end(), dist);
        return p - distance.begin();
    };

    tf::StampedTransform start_pose_odom, end_pose_odom, start_pose_ref, end_pose_ref, rpe;
    
    for(int i = 0; i < size; ++i)
    {
        if(!inter_transform(odom[i].stamp_, ref, dt_tolerance, start_pose_ref)) continue;
        const auto next_i = find_odom(distance[i] + seg_length);
        if(next_i == size) continue;
        if(!inter_transform(odom[next_i].stamp_, ref, dt_tolerance, end_pose_ref)) continue;
        start_pose_odom = odom[i];
        end_pose_odom = odom[next_i];
        calc_rpe(start_pose_ref, end_pose_ref, start_pose_odom, end_pose_odom, rpe);
        out.emplace_back(distance[i], distance[next_i] - distance[i], rpe);   
    }
}

void calc_average_drift_rate(const std::vector<DriftInfo>& drift , double& translation_drift, double& rotation_drift)
{
    translation_drift = -1;
    if(drift.empty()) return;
    translation_drift = 0;
    rotation_drift = 0;
    for(const auto& info : drift)
    {
        translation_drift += info.rpe.getOrigin().length() / info.seg_length;
        rotation_drift +=  180.0 / M_PI *  info.rpe.getRotation().getAngle() / info.seg_length;
    }
    translation_drift /= drift.size();
    rotation_drift  = rotation_drift / drift.size();
}

int main(int argc, char** argv)
{
    const double inter_dt_threshold = 1;
    tf::StampedTransform drift;
    std::vector<tf::StampedTransform> ref, odom;
    load_tum(argv[1], ref);
    load_tum(argv[2], odom);
    std::vector<double> distance;
    calc_path_distance(odom, distance);
    std::cout << "================= total drift ===================\n";
    std::cout << std::fixed;
    if(calc_drift(odom, ref, inter_dt_threshold, drift))
    {
        std::cout << std::setprecision(6);
        std::cout << "delta t(m):[" << drift.getOrigin().x() << ", " << drift.getOrigin().y() << ", " << drift.getOrigin().z() << "]\n";
        double roll ,pitch, yaw;
        tf::Matrix3x3(drift.getRotation()).getRPY(roll, pitch, yaw);
        std::cout << "delta R(rpy,deg):[" << 180.0 / M_PI * roll << ", " << 180.0 / M_PI * pitch << ", " << 180.0 / M_PI * yaw << "]" << std::endl; 
    }
    else
    {
        std::cout << "failed..." << std::endl;
    }
    std::cout << "===================== APE =======================\n";
    calc_ate(odom, ref, inter_dt_threshold);

    std::cout << "===================== RPE =======================\n";
    std::vector<double> seg_len{100, 200, 300, 400, 500, 600, 700, 800, 900};
    std::vector<DriftInfo> drift_info_total, drift_info_seg;
    double translation_drift_rate, rotation_drift_rate;
    std::vector<double> result;

    for(const auto len : seg_len)
    {
        drift_info_seg.clear();
        calc_rpe_by_segment(odom, distance, ref, inter_dt_threshold, len, drift_info_seg);
        calc_average_drift_rate(drift_info_seg, translation_drift_rate, rotation_drift_rate);
        if(translation_drift_rate >= 0)
        {
            drift_info_total.insert(drift_info_total.end(), drift_info_seg.begin(), drift_info_seg.end());
            std::cout << "avg_drift_rate(" << int(len) <<"m, " << drift_info_seg.size() << " poses): " 
                      << translation_drift_rate * 100.0 << " %, " << rotation_drift_rate * 100 << " deg/100m\n";
            result.push_back(len);
            result.push_back(translation_drift_rate);
            result.push_back(rotation_drift_rate);
        }
        else
        {
            std::cout << "failed..." << std::endl;
            break;
        }
    }

    calc_average_drift_rate(drift_info_total, translation_drift_rate, rotation_drift_rate);

    if (translation_drift_rate >= 0)
    {
        drift_info_total.insert(drift_info_total.end(), drift_info_seg.begin(), drift_info_seg.end());
        std::cout << "avg_drift_rate(average, " << drift_info_total.size() << " poses): "
                  << translation_drift_rate * 100 <<"%, " << rotation_drift_rate * 100 <<" deg/100m" << std::endl;;
    }
    else
    {
        std::cout << "failed..." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "=================== RPE table ===================\n";
    std::cout << "output as table:\n" << std::setw(5) << "len" << "   trans(%)" << "   rot(deg/100m)\n";
    std::cout << std::setprecision(5);
    std::cout << std::setw(5) << 0 << "    " << translation_drift_rate * 100 << "    " << rotation_drift_rate * 100 << "\n";
    for(int i = 0; i < result.size(); i+=3)
    {
        std::cout << std::setw(5) <<int(result[i]) << "    " << result[i+1] * 100 << "    " << result[i+2] * 100 << "\n";
    }
}