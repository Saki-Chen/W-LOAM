#pragma once
#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam
{
  class LidarPlaneFactor2 : public NoiseModelFactor2<Pose3, Pose3>
  {

    using X = Pose3;
    using Base = NoiseModelFactor2<Pose3, Pose3>;
    using This = LidarPlaneFactor2;

  public:
    LidarPlaneFactor2(Key key1, Key key2, const Point3 &point1, const Point3 &unit, const Point3 &point2, const SharedNoiseModel &model)
        : Base(model, key1, key2), p1_(point1), p2_(point2), u_(unit.normalized())
    {
    }

    virtual ~LidarPlaneFactor2() {}

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    plane_err = (p2w-p1).dot(u)
              = (T2*p2 - p1).dot(u)
    dplane_err/dT2 = u'*d(T2*p2)/dT2 = u'*R2*[[-p2]x I3x3]
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    plane_err = (p2w-p1w).dot(uw)
              = (T2*p2 - T1*p1).dot(R1*u)
    dplane_err/dT1 = (T2*p2 - T1*p1)'* d(R1*u)/dT1 - (R1*u)'*d(T1*p1)/dT1
    dplane_err/dT2 = (R1*u)'*d(T2*p2)/dT2
    */

    Vector evaluateError(const X &pose1, const X &pose2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const
    {
      const auto &rotation1 = pose1.rotation().matrix();
      const auto &rotation2 = pose2.rotation().matrix();
      const auto p12w = pose2.transformFrom(p2_) - pose1.transformFrom(p1_);
      const auto uw = rotation1 * u_;
      if (H1)
        *H1 = p12w.transpose() * rotation1 * (Matrix36() << skewSymmetric(-u_[0], -u_[1], -u_[2]), Z_3x3).finished()
            - uw.transpose() * rotation1 * (Matrix36() << skewSymmetric(-p1_[0], -p1_[1], -p1_[2]), I_3x3).finished();
      if (H2)
        *H2 = uw.transpose() * rotation2 * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
      return Vector1(p12w.dot(uw));
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
      return boost::static_pointer_cast<NonlinearFactor>(
          NonlinearFactor::shared_ptr(new This(*this)));
    }

    virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const
    {
      const This *e = dynamic_cast<const This *>(&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(p1_, e->p1_, tol) &&
             traits<Point3>::Equals(p2_, e->p2_, tol) && traits<Point3>::Equals(u_, e->u_, tol);
    }

    virtual void print(const std::string &s = "",
                       const KeyFormatter &keyFormatter = DefaultKeyFormatter) const
    {
      cout << s << ":\nLidarPlaneFactor2 on (" << keyFormatter(key1())
           << ", " << keyFormatter(key2()) << ")\n"
           << "  Plane Point: " << p1_.transpose() << "\n"
           << "  Plane norm Axis: " << u_.transpose() << "\n"
           << "  Match Point: " << p2_.transpose() << "\n";
      noiseModel_->print("  noise model: ");
    }

  private:
    Point3 p1_, p2_, u_;
  }; // class LidarPlaneFactor2

  class LidarPlaneFactor1 : public NoiseModelFactor1<Pose3>
  {

    using X = Pose3;
    using Base = NoiseModelFactor1<Pose3>;
    using This = LidarPlaneFactor1;

  public:
    LidarPlaneFactor1(Key key, const Point3 &point1, const Point3 &unit, const Point3 &point2, const SharedNoiseModel &model)
        : Base(model, key), p1_(point1), p2_(point2), u_(unit.normalized())
    {
    }

    virtual ~LidarPlaneFactor1() {}

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    plane_err = (p2w-p1).dot(u)
              = (T2*p2 - p1).dot(u)
    dplane_err/dT2 = u'*d(T2*p2)/dT2 = u'*R2*[[-p2]x I3x3]
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    plane_err = (p2w-p1w).dot(uw)
              = (T2*p2 - T1*p1).dot(R1*u)
    dplane_err/dT1 = (T2*p2 - T1*p1)'* d(R1*u)/dT1 - (R1*u)'*d(T1*p1)/dT1
    dplane_err/dT2 = (R1*u)'*d(T2*p2)/dT2
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    plane_err = (p2w-p1).dot(u)
              = (T2*p2 - p1).dot(u)
    dplane_err/dT2 = u'*d(T2*p2)/dT2 = u'*R2*[[-p2]x I3x3]
    */

    Vector evaluateError(const X &pose,
                         boost::optional<Matrix&> H = boost::none) const
    {
      const auto &rotation = pose.rotation();
      if (H)
        *H = u_.transpose() * rotation.matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
      return Vector1((pose.transformFrom(p2_) - p1_).dot(u_));
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
      return boost::static_pointer_cast<NonlinearFactor>(
          NonlinearFactor::shared_ptr(new This(*this)));
    }

    virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const
    {
      const This *e = dynamic_cast<const This *>(&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(p1_, e->p1_, tol) &&
             traits<Point3>::Equals(p2_, e->p2_, tol) && traits<Point3>::Equals(u_, e->u_, tol);
    }

    virtual void print(const std::string &s = "",
                       const KeyFormatter &keyFormatter = DefaultKeyFormatter) const
    {
      cout << s << ":\nLidarPlaneFactor1 on (" << keyFormatter(key()) <<  ")\n"
           << "  Plane Point: " << p1_.transpose() << "\n"
           << "  Plane norm Axis: " << u_.transpose() << "\n"
           << "  Match Point: " << p2_.transpose() << "\n";
      noiseModel_->print("  noise model: ");
    }

  private:
    Point3 p1_, p2_, u_;
  }; // class LidarPlaneFactor1

} // namespace gtsam
