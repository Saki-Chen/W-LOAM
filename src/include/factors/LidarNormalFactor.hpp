#pragma once
#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam
{
    class LidarNormalFactor : public NoiseModelFactor2<Pose3, Pose3>
    {
        using X = Pose3;
        using Base = NoiseModelFactor2<Pose3, Pose3>;
        using This = LidarNormalFactor;

    public:
        LidarNormalFactor(Key key1, Key key2, const Point3 &unit1, const Point3 &unit2, const SharedNoiseModel &model)
            : Base(model, key1, key2), u1_(unit1.normalized()), u2_(unit2.normalized())
        {
        }

        virtual ~LidarNormalFactor() {}

        /*
            d(R*u)/dT =  R*[[-u]x 03x3]
            
            err = R1*u1 x R2*u2
            derr/dT1 =  [R2*u2]x * R1*[[u1]x 03x3]
            derr/dT2 = -[R1*u1]x * R2*[[u2]x 03x3]
        */
        Vector evaluateError(const X &pose1, const X &pose2,
                             boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none) const
        {
            const auto &rot_u1 = pose1.rotation().matrix() * u1_;
            const auto &rot_u2 = pose2.rotation().matrix() * u2_;
            if (H1)
                *H1 = skewSymmetric(rot_u2[0], rot_u2[1], rot_u2[2]) * (Matrix36() << pose1.rotation().matrix() * skewSymmetric(u1_[0], u1_[1], u1_[2]), Z_3x3).finished();

            if(H2)
                *H2 = skewSymmetric(rot_u1[0], rot_u1[1], rot_u1[2]) * (Matrix36() << pose2.rotation().matrix() * skewSymmetric(u2_[0], u2_[1], u2_[2]), Z_3x3).finished();

            return rot_u1.cross(rot_u2);
        }

        virtual NonlinearFactor::shared_ptr clone() const
        {
            return boost::static_pointer_cast<NonlinearFactor>(
                NonlinearFactor::shared_ptr(new This(*this)));
        }

        // virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const
        // {
        //   const This *e = dynamic_cast<const This *>(&expected);
        //   return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(p1_, e->p1_, tol) &&
        //          traits<Point3>::Equals(p2_, e->p2_, tol) && traits<Point3>::Equals(u_, e->u_, tol);
        // }

        // virtual void print(const std::string &s = "",
        //                    const KeyFormatter &keyFormatter = DefaultKeyFormatter) const
        // {
        //   cout << s << ":\nLidarPlaneFactor1 on (" << keyFormatter(key()) <<  ")\n"
        //        << "  Plane Point: " << p1_.transpose() << "\n"
        //        << "  Plane norm Axis: " << u_.transpose() << "\n"
        //        << "  Match Point: " << p2_.transpose() << "\n";
        //   noiseModel_->print("  noise model: ");
        // }

    private:
        Point3 u1_, u2_;
    }; // class LidarNormalFactor
}