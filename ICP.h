#ifndef _ICP_
#define _ICP_


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <chrono>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <sophus/se3.hpp>


using namespace std;
using namespace cv;

struct ICPCeres
{
    ICPCeres ( Point3d uvw,Point3d xyz ) : _uvw(uvw),_xyz(xyz) {}
    // 残差的计算
    template <typename T>
    bool operator() (
            const T* _q,    // 模型参数
	    const T* _t,
            T* residual ) const     // 残差
    {
      
        Eigen::Quaternion<T> q_r{_q[3],_q[0],_q[1],_q[2]};
	Eigen::Matrix<T,3,1> point{T(_xyz.x),T(_xyz.y),T(_xyz.z)};
	Eigen::Matrix<T,3,1> tt{_t[0],_t[1],_t[2]};
	
// 	
 	Eigen::Matrix<T,3,1> sta=q_r*point+tt;
// 	
	residual[0] = T(_uvw.x)-(sta(0));
        residual[1] = T(_uvw.y)-(sta(1));
        residual[2] = T(_uvw.z)-(sta(2));
	
	
	
	
	
	
        //const Eigen::Matrix<T, 6, 1> param(parameters);
        //Sophus::SE3d tf = Sophus::SE3d::exp((param));
//         //T p[3];
/*        
        Eigen::Matrix<T,3,1> point;
        point<<T(_xyz.x),T(_xyz.y),T(_xyz.z);
       
        Eigen::Matrix<T,4,4> result=tf.matrix();*/
//         Eigen::Matrix<T,3,3> Rt;
//         Rt<<T(result(0,0)),T(result(0,1)),T(result(0,2)),T(result(1,0)),T(result(1,1)),T(result(1,2)),T(result(2,0)),T(result(2,1)),T(result(2,2));
//         Eigen::Matrix<T,3,1> tt{T(result(0,3)),T(result(1,3)),T(result(2,3))};
// 	
// 	Eigen::Matrix<T,3,1> sta=Rt*point+tt;
//         
//         residual[0] = T(_uvw.x)-T(sta(0));
//         residual[1] = T(_uvw.y)-T(sta(1));
//         residual[2] = T(_uvw.z)-T(sta(2));
        return true;
    }
    static ceres::CostFunction* Create(const Point3d uvw,const Point3d xyz) {
        return (new ceres::AutoDiffCostFunction<ICPCeres, 3, 4, 3>(
                new ICPCeres(uvw,xyz)));
    }
    const Point3f _uvw;
    const Point3f _xyz;
};

/*struct ICPceres{
  
  ICPceres(Eigen::Vector3d  s, Eigen::Vector3d  t) : s(s), t(t) {};

  template<typename T>
  bool operator()(const T* const parameters,
                  T* residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 6, 1>> param(parameters);
    Sophus::SE3d tf = Sophus::SE3d::exp(param);
   
    Eigen::Vector3d s_trans = tf * s;
    Eigen::Matrix4d result=tf.matrix();
    Eigen::Matrix3d Rt;
    Eigen::Vector3d tt;
    
    Rt<<result(0,0),result(0,1),result(0,2),
    result(1,0),result(1,1),result(1,2),
    result(2,0),result(2,1),result(2,2);
    tt<<result(0,3),result(1,3),result(2,3);
    Eigen::Vector3d s_trans = Rt * s+tt;
    
    residuals[0] = t(0,0) - s_trans(0,0);
    residuals[1] = t(1,0) - s_trans(1,0);
    residuals[2] = t(2,0) - s_trans(2,0);
   
    
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d  s,
                                     const Eigen::Vector3d t) {
    return new ceres::AutoDiffCostFunction<ICPceres, 3, 6>(
        new ICPceres(s, t));
  }

 protected:
   Eigen::Vector3d  s;
   Eigen::Vector3d  t;
};*/





namespace relative_positioning
{
  /**ICP SVD算法——方法来源于SLAM14讲*/
  void pose_estimation_3d3d(const vector<Point3d> &pts1,const vector<Point3d> &pts2,Eigen::Matrix3d &R,Eigen::Vector3d &t){
    Point3d p1, p2;     // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
       p1 += pts1[i];
       p2 += pts2[i];
    }
    p1 = Point3d(Vec3d(p1) / N);
    p2 = Point3d(Vec3d(p2) / N);
    vector<Point3d> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
       q1[i] = pts1[i] - p1;
       q2[i] = pts2[i] - p2;
    }

     // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    //cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    //cout << "U=" << U << endl;
    //cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    if (R_.determinant() < 0) {
       R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
    R <<
       R_(0, 0), R_(0, 1), R_(0, 2),
       R_(1, 0), R_(1, 1), R_(1, 2),
       R_(2, 0), R_(2, 1), R_(2, 2);
    t << t_(0, 0), t_(1, 0), t_(2, 0);
  }
  
  
  
  
  void pose_estimation_ICP_ceres(const vector<Point3d> &pts1,const vector<Point3d> &pts2,Eigen::Matrix3d &R, Eigen::Vector3d &t,const Eigen::Matrix<double, 6, 1> &initialvalue){
  
  
  ceres::Problem problem;
  Eigen::Matrix<double, 6, 1> tf_calculated_parameter=initialvalue;
 
  Eigen::Matrix4d result=Sophus::SE3d::exp(tf_calculated_parameter).matrix();
  //Eigen::Matrix3d R;
  R<<result(0,0),result(0,1),result(0,2),
    result(1,0),result(1,1),result(1,2),
    result(2,0),result(2,1),result(2,2);
    
  Eigen::Quaterniond q(R);
  double q1[4];
  q1[0]=q.x();
  q1[1]=q.y();
  q1[2]=q.z();
  q1[3]=q.w();
  
  double t1[3];
  t1[0]=result(0,3),t1[1]=result(1,3),t1[2]=result(2,3);
  
  
  
  for (int i = 0; i < pts2.size(); ++i) {
   /* Eigen::Vector3d vpts1,vpts2;
    vpts1<<pts1.at(i).x,pts1.at(i).y,pts1.at(i).z;
    vpts2<<pts2.at(i).x,pts2.at(i).y,pts2.at(i).z;*/
    
    ceres::CostFunction *cost_function = ICPCeres::Create(pts2[i], pts1[i]);
    problem.AddResidualBlock(cost_function, NULL, q1,t1);
  }
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout=true;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  
  std::cout << summary.BriefReport() << std::endl;
  
  
  Eigen::Quaterniond qr;
  qr.normalize();
  qr.x()=q1[0];
  qr.y()=q1[1];
  qr.z()=q1[2];
  qr.w()=q1[3];
  
  t<<t1[0],t1[1],t1[2];
  Sophus::SE3d SE3_qt(qr,t);
  
  Eigen::Matrix4d result1=SE3_qt.matrix();
  
  R<<result1(0,0),result1(0,1),result1(0,2),
    result1(1,0),result1(1,1),result1(1,2),
    result1(2,0),result1(2,1),result1(2,2);
  
  
  
  //R=qn.matrix();
  //t<<t1[0],t1[1],t1[2];
  
  
  
//   std::cout << "result:\n"
//             << Sophus::SE3d::exp(tf_calculated_parameter).matrix()<<std::endl;
//   
  std::cout << "R:\n"
            << R<<std::endl;
  std::cout << "t:\n"
            << t<<std::endl;
  
  
 }

}






#endif

