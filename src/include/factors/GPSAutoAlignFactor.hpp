#pragma once
#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam
{

    class GPSAutoAlignFactor2D : public NoiseModelFactor2<Pose2, Pose3>
    {
        using Base = NoiseModelFactor2<X1, X2>;
        using This = GPSAutoAlignFactor2D;
        using GPSMeasure = Point2;

    public:
        using AlignT = X1;
        GPSAutoAlignFactor2D(Key key1, Key key2, const GPSMeasure &gps_measure, const Eigen::Isometry3d&  T_X_gps,const SharedNoiseModel &model)
            : Base(model, key1, key2), _gps_measure(gps_measure), _T_X_gps(T_X_gps.matrix()){}

        virtual ~GPSAutoAlignFactor2D() {}

        // (T_earth_map * T_map_X * T_X_gps).translation - gps_measure
        Vector evaluateError(const Pose2 &T_earth_map, const Pose3 &pose,
                             boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none) const
        {
            Matrix36 H3;
            Vector2 error = (X2(T_earth_map).transformPoseFrom(pose.transformPoseFrom(_T_X_gps, H2), H1)).translation(H1 || H2 ? &H3 : 0).topRows(2) - _gps_measure;
            if (H1)
            {
                *H1 = H3.topRows(2) * *H1 * (Matrix63() << 0.0, 0.0, 0.0,
                                                           0.0, 0.0, 0.0,
                                                           0.0, 0.0, 1.0,
                                                           1.0, 0.0, 0.0,
                                                           0.0, 1.0, 0.0,
                                                           0.0, 0.0, 0.0).finished();
            }
            if (H2)
            {
                *H2 = H3.topRows(2) * *H2;
            }

            return error;
        }

        virtual NonlinearFactor::shared_ptr clone() const
        {
            return boost::static_pointer_cast<NonlinearFactor>(
                NonlinearFactor::shared_ptr(new This(*this)));
        }

        virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const
        {
            const This *e = dynamic_cast<const This *>(&expected);
            return e != nullptr && Base::equals(*e, tol) && traits<GPSMeasure>::Equals(_gps_measure, e->_gps_measure, tol)
                                && traits<Pose3>::Equals(_T_X_gps, e->_T_X_gps);
        }

        virtual void print(const std::string &s = "",
                           const KeyFormatter &keyFormatter = DefaultKeyFormatter) const
        {
            cout << s << ":\nGPSAutoAlignFactor2D on (" << keyFormatter(key1())
                 << ", " << keyFormatter(key2()) << ")\n"
                 << "  GPS Measurement: " << _gps_measure.transpose() << "\n"
                 << "  T_X_gps: " << _T_X_gps.matrix() << "\n";
            noiseModel_->print("  noise model: ");
        }

    private:
        GPSMeasure _gps_measure;
        Pose3 _T_X_gps;
    }; // class GPSAutoAlignFactor2D

    class GPSAutoAlignFactor3D : public NoiseModelFactor2<Pose3, Pose3>
    {
        using Base = NoiseModelFactor2<X1, X2>;
        using This = GPSAutoAlignFactor3D;
        using GPSMeasure = Point3;

    public:
        using AlignT = X1;
        GPSAutoAlignFactor3D(Key key1, Key key2, const GPSMeasure &gps_measure, const Eigen::Isometry3d& T_X_gps, const SharedNoiseModel &model)
            : Base(model, key1, key2), _gps_measure(gps_measure), _T_X_gps(T_X_gps.matrix()){}

        virtual ~GPSAutoAlignFactor3D() {}

        // (T_earth_map * T_map_X * T_X_gps).translation - gps_measure
        Vector evaluateError(const X1 &T_earth_map, const X2 &pose,
                             boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none) const
        {
            Matrix36 H3;
            Vector3 error = T_earth_map.transformPoseFrom(pose.transformPoseFrom(_T_X_gps, H2), H1).translation(H1 || H2 ? &H3 : 0) - _gps_measure;
            // Vector3 error = T_earth_map.transformPoseFrom(pose.transformPoseFrom(_T_X_gps, H2), H1).translation(H1 || H2 ? &H3 : 0) - gtsam::Vector3(_gps_measure[0], _gps_measure[1], 0);
            if (H1)
            {
                *H1 = H3 * *H1;
                // *H1 = H3 * *H1;

            }
            if (H2)
            {
                *H2 = H3 * *H2;
                // *H2 = H3 * *H2;
            }

            return error;
        }

        virtual NonlinearFactor::shared_ptr clone() const
        {
            return boost::static_pointer_cast<NonlinearFactor>(
                NonlinearFactor::shared_ptr(new This(*this)));
        }

        virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const
        {
            const This *e = dynamic_cast<const This *>(&expected);
            return e != nullptr && Base::equals(*e, tol) && traits<GPSMeasure>::Equals(_gps_measure, e->_gps_measure, tol)
                                && traits<Pose3>::Equals(_T_X_gps, e->_T_X_gps);
        }

        virtual void print(const std::string &s = "",
                           const KeyFormatter &keyFormatter = DefaultKeyFormatter) const
        {
            cout << s << ":\nGPSAutoAlignFactor3D on (" << keyFormatter(key1())
                 << ", " << keyFormatter(key2()) << ")\n"
                 << "  GPS Measurement: " << _gps_measure.transpose() << "\n"
                 << "  T_X_gps: " << _T_X_gps.matrix() << "\n";
            noiseModel_->print("  noise model: ");
        }

    private:
        GPSMeasure _gps_measure;
        Pose3 _T_X_gps;
    }; // class GPSAutoAlignFactor3D

} // namespace gtsam