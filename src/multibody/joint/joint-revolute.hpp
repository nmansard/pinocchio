#ifndef __se3_joint_revolute_hpp__
#define __se3_joint_revolute_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"

namespace se3
{

  template<int axis> struct JointDataRevolute;
  template<int axis> struct JointModelRevolute;
  
  namespace revolute
  {
    template<int axis>
    struct CartesianVector3
    {
      double w; 
      CartesianVector3(const double & w) : w(w) {}
      CartesianVector3() : w(1) {}
      operator Eigen::Vector3d (); // { return Eigen::Vector3d(w,0,0); }
    };
    template<>CartesianVector3<0>::operator Eigen::Vector3d () { return Eigen::Vector3d(w,0,0); }
    template<>CartesianVector3<1>::operator Eigen::Vector3d () { return Eigen::Vector3d(0,w,0); }
    template<>CartesianVector3<2>::operator Eigen::Vector3d () { return Eigen::Vector3d(0,0,w); }
    Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<0> & wx)
    { return Eigen::Vector3d(w1[0]+wx.w,w1[1],w1[2]); }
    Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<1> & wy)
    { return Eigen::Vector3d(w1[0],w1[1]+wy.w,w1[2]); }
    Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<2> & wz)
    { return Eigen::Vector3d(w1[0],w1[1],w1[2]+wz.w); }
  } // namespace revolute

  template<int axis> 
  struct JointRevolute {
    struct BiasZero 
    {
      operator Motion () const { return Motion::Zero(); }
    };
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct MotionRevolute 
    {
      MotionRevolute()                   : w(NAN) {}
      MotionRevolute( const double & w ) : w(w)  {}
      double w;

      operator Motion() const
      { 
	return Motion(Motion::Vector3::Zero(),typename revolute::CartesianVector3<axis>(w));
      }
    }; // struct MotionRevolute

    friend const MotionRevolute& operator+ (const MotionRevolute& m, const BiasZero&) { return m; }
    friend Motion operator+( const MotionRevolute& m1, const Motion& m2)
    {
      return Motion( m2.linear(),m2.angular()+typename revolute::CartesianVector3<axis>(m1.w)); 
    }    
    struct ConstraintRevolute
    { 
      template<typename D>
      MotionRevolute operator*( const Eigen::MatrixBase<D> & v ) const { return MotionRevolute(v[0]); }

      const ConstraintRevolute & transpose() const { return *this; }
     //template<typename D> D operator*( const Force& f ) const
     Force::Vector3::ConstFixedSegmentReturnType<1>::Type
     operator*( const Force& f ) const
     { return f.angular().segment<1>(axis); }

      operator ConstraintXd () const
      {
	Eigen::Matrix<double,6,1> S;
	S << Eigen::Vector3d::Zero(), (Eigen::Vector3d)revolute::CartesianVector3<axis>();
	return ConstraintXd(S);
      }
    }; // struct ConstraintRevolute

    static Eigen::Matrix3d cartesianRotation(const double & angle); 
  };

  Motion operator^( const Motion& m1, const JointRevolute<0>::MotionRevolute& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(w,0,0) = ( 0,zw,-yw )
     * nu1^(0,wx) = ( 0,vz1 wx,-vy1 wx,    0,wz1 wx,-wy1 wx)
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion( Motion::Vector3(0,v[2]*wx,-v[1]*wx),
		   Motion::Vector3(0,w[2]*wx,-w[1]*wx) );
  }

  Motion operator^( const Motion& m1, const JointRevolute<1>::MotionRevolute& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,w,0) = ( -z,0,x )
     * nu1^(0,wx) = ( -vz1 wx,0,vx1 wx,    -wz1 wx,0,wx1 wx)
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion( Motion::Vector3(-v[2]*wx,0, v[0]*wx),
		   Motion::Vector3(-w[2]*wx,0, w[0]*wx) );
  }

  Motion operator^( const Motion& m1, const JointRevolute<2>::MotionRevolute& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,0,w) = ( y,-x,0 )
     * nu1^(0,wx) = ( vy1 wx,-vx1 wx,0,    wy1 wx,-wx1 wx,0 )
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion( Motion::Vector3(v[1]*wx,-v[0]*wx,0),
		   Motion::Vector3(w[1]*wx,-w[0]*wx,0) );
  }

  template<>
  Eigen::Matrix3d JointRevolute<0>::cartesianRotation(const double & angle) 
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	1,0,0,
	0,ca,-sa,
	0,sa,ca;
      return R3;
    }
  template<>
  Eigen::Matrix3d JointRevolute<1>::cartesianRotation(const double & angle)
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	ca, 0, sa,
	0 , 1,  0,
	-sa, 0,  ca;
      return R3;
    }
  template<>
  Eigen::Matrix3d JointRevolute<2>::cartesianRotation(const double & angle) 
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	ca,-sa,0,
	sa,ca,0,
	0,0,1;
      return R3;
    }



  template<int axis>
  struct traits< JointRevolute<axis> >
  {
    typedef JointDataRevolute<axis> JointData;
    typedef JointModelRevolute<axis> JointModel;
    typedef typename JointRevolute<axis>::ConstraintRevolute Constraint_t;
    typedef SE3 Transformation_t;
    typedef typename JointRevolute<axis>::MotionRevolute Motion_t;
    typedef typename JointRevolute<axis>::BiasZero Bias_t;
    enum {
      nq = 1,
      nv = 1
    };
  };

  template<int axis> struct traits< JointDataRevolute<axis> > { typedef JointRevolute<axis> Joint; };
  template<int axis> struct traits< JointModelRevolute<axis> > { typedef JointRevolute<axis> Joint; };

  template<int axis>
  struct JointDataRevolute : public JointDataBase< JointDataRevolute<axis> >
  {
    typedef JointRevolute<axis> Joint;
    SE3_JOINT_TYPEDEF;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    JointDataRevolute() : M(1)
    {
      M.translation(SE3::Vector3::Zero());
    }
  };

  template<int axis>
  struct JointModelRevolute : public JointModelBase< JointModelRevolute<axis> >
  {
    typedef JointRevolute<axis> Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRevolute>::idx_q;
    using JointModelBase<JointModelRevolute>::idx_v;
    using JointModelBase<JointModelRevolute>::setIndexes;
    
    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];
      data.M.rotation(JointRevolute<axis>::cartesianRotation(q));
    }

    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      data.M.rotation(JointRevolute<axis>::cartesianRotation(q));
      data.v.w = v;
    }


  };

  typedef JointDataRevolute<0> JointDataRX;
  typedef JointModelRevolute<0> JointModelRX;

  typedef JointDataRevolute<1> JointDataRY;
  typedef JointModelRevolute<1> JointModelRY;

  typedef JointDataRevolute<2> JointDataRZ;
  typedef JointModelRevolute<2> JointModelRZ;

} //namespace se3

#endif // ifndef __se3_joint_revolute_hpp__
