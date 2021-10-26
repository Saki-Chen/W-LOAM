#pragma once
#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam
{
  class LidarEdgeFactor2 : public NoiseModelFactor2<Pose3, Pose3>
  {

    using X = Pose3;
    using Base = NoiseModelFactor2<Pose3, Pose3>;
    using This = LidarEdgeFactor2;

  public:
    LidarEdgeFactor2(Key key1, Key key2, const Point3 &point1, const Point3 &unit, const Point3 &point2, const SharedNoiseModel &model)
        : Base(model, key1, key2), p1_(point1), p2_(point2), u_(unit.normalized())
    {
    }

    virtual ~LidarEdgeFactor2() {}

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1).cross(u)
            = (T2*p2 - p1).cross(u)
    dline_err/dT2 = (-u)x * d(T2*p2)/dT2 = (-u)x * R2 * [[-p]x I3X3]
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1w).cross(uw)
            = (T2*p2 - T1*p1).cross(R1*u)
    dline_err/dT1 = (T2*p2 - T1*p1)x * d(R1*u)/dT1 + (R1*u)x * d(T1*p1)/dT1
    dline_err/dT2 = -(R1*u)x * d(T2*p2)/dT2
    */

    Vector evaluateError(const X &pose1, const X &pose2,
                         boost::optional<Matrix &> H1 = boost::none,
                         boost::optional<Matrix &> H2 = boost::none) const
    {
      const auto &rotation1 = pose1.rotation().matrix();
      const auto &rotation2 = pose2.rotation().matrix();
      const auto p12w = pose2.transformFrom(p2_) - pose1.transformFrom(p1_);
      const auto uw = rotation1 * u_;
      if (H1)
        *H1 = skewSymmetric(p12w[0], p12w[1], p12w[2]) * rotation1 * (Matrix36() << skewSymmetric(-u_[0], -u_[1], -u_[2]), Z_3x3).finished() 
            + skewSymmetric(uw[0], uw[1], uw[2]) * rotation1 * (Matrix36() << skewSymmetric(-p1_[0], -p1_[1], -p1_[2]), I_3x3).finished();
      if (H2)
        *H2 = skewSymmetric(-uw[0], -uw[1], -uw[2]) * rotation2 * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
      return p12w.cross(uw);
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
      cout << s << ":\nLidarEdgeFactor2 on (" << keyFormatter(key1())
           << ", " << keyFormatter(key2()) << ")\n"
           << "  Edge Point: " << p1_.transpose() << "\n"
           << "  Edge Axis: " << u_.transpose() << "\n"
           << "  Match Point: " << p2_.transpose() << "\n";
      noiseModel_->print("  noise model: ");
    }

  private:
    Point3 p1_, p2_, u_;

  }; // class LidarEdgeFactor2

  class LidarEdgeFactor1 : public NoiseModelFactor1<Pose3>
  {

    using X = Pose3;
    using Base = NoiseModelFactor1<Pose3>;
    using This = LidarEdgeFactor1;

  public:
    LidarEdgeFactor1(Key key, const Point3 &point1, const Point3 &unit, const Point3 &point2, const SharedNoiseModel &model)
        : Base(model, key), p1_(point1), p2_(point2), u_(unit.normalized())
    {
    }

    virtual ~LidarEdgeFactor1() {}

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1).cross(u)
            = (T2*p2 - p1).cross(u)
    dline_err/dT2 = (-u)x * d(T2*p2)/dT2 = (-u)x * R2 * [[-p]x I3X3]
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1w).cross(uw)
            = (T2*p2 - T1*p1).cross(R1*u)
    dline_err/dT1 = (T2*p2 - T1*p1)x * d(R1*u)/dT1 + (R1*u)x * d(T1*p1)/dT1
    dline_err/dT2 = -(R1*u)x * d(T2*p2)/dT2
    */

    Vector evaluateError(const X &pose,
                         boost::optional<Matrix &> H = boost::none) const
    {
      const auto &rotation = pose.rotation();
      if (H)
        *H = skewSymmetric(-u_[0], -u_[1], -u_[2]) * rotation.matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
      return (pose.transformFrom(p2_) - p1_).cross(u_);
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
      cout << s << ":\nLidarEdgeFactor1 on (" << keyFormatter(key()) << ")\n"
           << "  Edge Point: " << p1_.transpose() << "\n"
           << "  Edge Axis: " << u_.transpose() << "\n"
           << "  Match Point: " << p2_.transpose() << "\n";
      noiseModel_->print("  noise model: ");
    }

  private:
    Point3 p1_, p2_, u_;

  }; // class LidarEdgeFactor1

  class LidarEdgeProjectedFactor1 : public NoiseModelFactor1<Pose3>
  {

    using X = Pose3;
    using Base = NoiseModelFactor1<Pose3>;
    using This = LidarEdgeProjectedFactor1;

  public:
    LidarEdgeProjectedFactor1(Key key, const Point3 &point1, const Point3 &unit, const Point3 &point2, const SharedNoiseModel &model)
        : Base(model, key), p1_(point1), p2_(point2), u_(unit.normalized())
    {
      // std::cout << "unit:" << u_.transpose()<<"\n";
    }

    virtual ~LidarEdgeProjectedFactor1() {}

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1).cross(u)
            = (T2*p2 - p1).cross(u)
    dline_err/dT2 = (-u)x * d(T2*p2)/dT2 = (-u)x * R2 * [[-p]x I3X3]
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1w).cross(uw)
            = (T2*p2 - T1*p1).cross(R1*u)
    dline_err/dT1 = (T2*p2 - T1*p1)x * d(R1*u)/dT1 + (R1*u)x * d(T1*p1)/dT1
    dline_err/dT2 = -(R1*u)x * d(T2*p2)/dT2
    */

    /*
    d(T^-1*p)/dT = [[T^-1*p]x -I3X3]
    d(R^-1*u)/dT = [[R^T*u]x   03x3]

    line_err = (p2 - p1).cross(u)
             = (p2 - T2^-1*p1).cross(R2^T*u1)

    point_err = (p2 - T2^-1*p1)[0,1] * R2^T*u1[2] = (p2 - c2)[0,1] * c1[2]

    dline_err/dT2 = [R2^-1*u1]x*d(T2^-1*p1)/dT2 + d(R2^-1*u1)/dT2
                  = [R2^T*u1]x*[[T2^-1*p1]x -I3X3] + [[R2^T*u1]x   03x3]

    c1 = R2^T*u1
    c2 = T2^-1*p1
    dpoint_err/dT2 = -dc2/dT2[0,1] * c1[2] + (p2 - c2)[0,1] * dc1/dT2[2]
                   = -[[c2]x -I3X3][0,1] * c1[2] +  (p2 - c2)[0,1]  *  [[c1]x  03x3][2]
    dc1/dT2 = [[R2^T*u1]x   03x3]
    dc2/dT2 = [[T2^-1*p1]x -I3X3]
          
   */

    Vector evaluateError(const X &pose,
                         boost::optional<Matrix &> H = boost::none) const
    {
      // const auto &rotation = pose.rotation();
      // if (H)
      // {
      //   *H = skewSymmetric(-u_[0], -u_[1], -u_[2]) * rotation.matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
      // H->col(0).setZero();
      // H->col(1).setZero();
      // H->col(5).setZero();
      // }
      // return (pose.transformFrom(p2_) - p1_).cross(u_);

      const auto c1 = pose.rotation().inverse() * u_;
      const auto c2 = pose.inverse().transformFrom(p1_);

      
      if (H)
      {
        // *H = skewSymmetric(c1[0], c1[1], c1[2]) * (Matrix36() << skewSymmetric(c2[0], c2[1], c2[2]), -I_3x3).finished()
            // + (Matrix36() << skewSymmetric(c1[0], c1[1], c1[2]), Z_3x3).finished();
        //  *H = (Matrix13() << -c1[1], +c1[0], 0.0).finished() * (Matrix36() << skewSymmetric(c2[0], c2[1], c2[2]), -I_3x3).finished() + (Matrix16() << -c1[1], +c1[0], 0.0, 0.0, 0.0, 0.0).finished();
        *H = (Matrix36() << (Matrix13() << -c1[1], +c1[0], 0.0).finished() * (Matrix36() << skewSymmetric(c2[0], c2[1], c2[2]), -I_3x3).finished() + (Matrix16() << -c1[1], +c1[0], 0.0, 0.0, 0.0, 0.0).finished(),
             -(Matrix36() << skewSymmetric(c2[0], c2[1], c2[2]), -I_3x3).finished().topRows(2) * c1[2] +  (p2_ - c2).topRows(2)  *  (Matrix16() << -c1[1], +c1[0], 0.0, 0.0, 0.0, 0.0).finished()).finished();
      }

      // return Vector1((p2_[0] - c2[0]) * c1[1] - (p2_[1] - c2[1]) * c1[0]);
      return (Vector3() << (p2_[0] - c2[0]) * c1[1] - (p2_[1] - c2[1]) * c1[0], (p2_ - c2).topRows(2) * c1[2]).finished();
      // return (p2_ - c2).cross(c1);
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
      cout << s << ":\nLidarEdgeProjectedFactor1 on (" << keyFormatter(key()) << ")\n"
           << "  Edge Point: " << p1_.transpose() << "\n"
           << "  Edge Axis: " << u_.transpose() << "\n"
           << "  Match Point: " << p2_.transpose() << "\n";
      noiseModel_->print("  noise model: ");
    }

  private:
    Point3 p1_, p2_, u_;

  }; // class LidarEdgeProjectedFactor1

class LidarEdgeProjectedFactor2 : public NoiseModelFactor2<Pose3, Pose3>
  {

    using X = Pose3;
    using Base = NoiseModelFactor2<Pose3, Pose3>;
    using This = LidarEdgeProjectedFactor2;

  public:
    LidarEdgeProjectedFactor2(Key key1, Key key2, const Point3 &point1, const Point3 &unit, const Point3 &point2, const SharedNoiseModel &model)
        : Base(model, key1, key2), p1_(point1), p2_(point2), u_(unit.normalized())
    {
    }

    virtual ~LidarEdgeProjectedFactor2() {}

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1).cross(u)
            = (T2*p2 - p1).cross(u)
    dline_err/dT2 = (-u)x * d(T2*p2)/dT2 = (-u)x * R2 * [[-p]x I3X3]
    */

    /*
    d(R*u)/dT =  R*[[-u]x 03x3]
    d(T*p)/dT =  R*[[-p]x I3x3]

    line_err = (p2w-p1w).cross(uw)
            = (T2*p2 - T1*p1).cross(R1*u)
    dline_err/dT1 = (T2*p2 - T1*p1)x * d(R1*u)/dT1 + (R1*u)x * d(T1*p1)/dT1
    dline_err/dT2 = -(R1*u)x * d(T2*p2)/dT2
    */

    /*
    d(T^-1*p)/dT = [[T^-1*p]x -I3X3]
    d(R^-1*u)/dT = [[R^T*u]x   03x3]

    line_err = (T1^-1*T2*p2 - p1).cross(u1)
    
    point_err = (T1^-1*T2*p2 - p1)[0,1] * u1[2]

    dline_err/dT1 = -[u1]x*d(T1^-1*T2*p2)/dT1
                  = -[u1]x*[[T1^-1*T2*p2]x -I3X3]
    dline_err/dT2 = -[u1]x*d(T1^-1*T2*p2)/dT2
                  = -[u1]x*T1^-1*R2*[[-p2]x I3x3]
                  = -[u1]x*R(T1^-1)*R2*[[-p2]x I3x3]
                  = -[u1]x*R1^-1*R2*[[-p2]x I3x3]
    dpoint_err/dT1 = u1[2]*d(T1^-1*T2*p2)/dT1[0,1]
                   = u1[2]*[[T1^-1*T2*p2]x -I3X3][0,1]
    dpoint_err/dT2 = u1[2]*d(T1^-1*T2*p2)/dT2[0,1]
                   = u1[2]*T1^-1*R2*[[-p2]x I3x3][0,1]
                   = u1[2]*R(T1^-1)*R2*[[-p2]x I3x3][0,1]
                   = u1[2]*R1^-1*R2*[[-p2]x I3x3][0,1]
   */

    Vector evaluateError(const X &pose1, const X &pose2,
                         boost::optional<Matrix &> H1 = boost::none,
                         boost::optional<Matrix &> H2 = boost::none) const
    {
      // const auto ux = skewSymmetric(u_[0], u_[1], u_[2]);
      const auto p2_transformed = (pose1.inverse()*pose2).transformFrom(p2_);

      if (H1)
      {
        // *H1 = -ux * (Matrix36() << skewSymmetric(p2_transformed[0], p2_transformed[1], p2_transformed[2]), -I_3x3).finished();
        // *H1 = -(Matrix13() << -u_[1], u_[0], 0.0).finished() * (Matrix36() << skewSymmetric(p2_transformed[0], p2_transformed[1], p2_transformed[2]), -I_3x3).finished();
        *H1 = (Matrix36() << -(Matrix13() << -u_[1], u_[0], 0.0).finished() * (Matrix36() << skewSymmetric(p2_transformed[0], p2_transformed[1], p2_transformed[2]), -I_3x3).finished(),
                             u_[2] * (Matrix36() << skewSymmetric(p2_transformed[0], p2_transformed[1], p2_transformed[2]), -I_3x3).finished().topRows(2)).finished();
          //  *H1 = u_[2] * (Matrix36() << skewSymmetric(p2_transformed[0], p2_transformed[1], p2_transformed[2]), -I_3x3).finished().topRows(2);


      }
      if (H2)
      {
        // *H2 = -ux * pose1.rotation().inverse().matrix() * pose2.rotation().matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
        // *H2 = -(Matrix13() << -u_[1], u_[0], 0.0).finished() * pose1.rotation().inverse().matrix() * pose2.rotation().matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished();
        *H2 = (Matrix36() << -(Matrix13() << -u_[1], u_[0], 0.0).finished() * pose1.rotation().inverse().matrix() * pose2.rotation().matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished(),
              u_[2] * (pose1.rotation().inverse().matrix() * pose2.rotation().matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished()).topRows(2)).finished();
        // *H2 = u_[2] * (pose1.rotation().inverse().matrix() * pose2.rotation().matrix() * (Matrix36() << skewSymmetric(-p2_[0], -p2_[1], -p2_[2]), I_3x3).finished()).topRows(2);
          
      }

      // return (p2_transformed - p1_).cross(u_);
      // return Vector1((p2_transformed[0] - p1_[0]) * u_[1] - (p2_transformed[1] - p1_[1]) * u_[0]);
      return (Vector3() << (p2_transformed[0] - p1_[0]) * u_[1] - (p2_transformed[1] - p1_[1]) * u_[0],
                           (p2_transformed - p1_).topRows(2)  * u_(2)).finished();  
      // return Vector2((p2_transformed - p1_).topRows(2)  * u_(2));
    
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
      cout << s << ":\nLidarEdgeProjectedFactor2 on (" << keyFormatter(key1())
           << ", " << keyFormatter(key2()) << ")\n"
           << "  Edge Point: " << p1_.transpose() << "\n"
           << "  Edge Axis: " << u_.transpose() << "\n"
           << "  Match Point: " << p2_.transpose() << "\n";
      noiseModel_->print("  noise model: ");
    }

  private:
    Point3 p1_, p2_, u_;

  }; // class LidarEdgeProjectedFactor2



} // namespace gtsam
