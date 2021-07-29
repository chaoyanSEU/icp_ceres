//
// Created by alterlimbo on 10/23/19.
//

#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
//#include "ceresicp.h"
#include "ICP.h"


int main(int argc, char **argv) {
  //std::vector<Eigen::Vector3d> target;
  std::vector<Point3d> target(9);
  target[0]=Point3d(-2.03435931, 5.72790387, -0.262000133);
  target[1]=Point3d(-3.8778978, 8.2628149, -0.217000133);
  target[2]=Point3d(-9.54013678, 19.5488507, 0.0469998671);
  target[3]=Point3d(52.6097968, 29.6025981, 0.761999867);
  target[4]=Point3d(80.3882647, 150.722268, 6.10199987);
  target[5]=Point3d(-221.015452, 126.763348, 8.72499987);
  target[6]=Point3d(-78.0543083, -128.085737, 4.93099987);
  target[7]=Point3d(-16.3343044, 165.462059, 3.09399987);
  target[8]=Point3d(-158.95493, 61.8613353, 13.8209999);
 
  
  Sophus::SE3d
      tf(Sophus::SO3d::exp(Eigen::Vector3d(0, 0, 0)), Eigen::Vector3d(2, 3, 4));
  std::vector<Point3d> source(9);
  source[0]=Point3d(-2.03524624524535,5.73008273049235,-0.176387381972605 );
  source[1]=Point3d(-3.87965409048327,8.26586712085141,-0.0685404769014402);
  source[2]=Point3d(-9.54479748450550,19.5559458358833,0.405035150211578);
  source[3]=Point3d(52.6287260973001,29.6174664193184,-0.540110433406383 );
  source[4]=Point3d(80.4105928709373,150.786167796010,4.60320155159029);
  source[5]=Point3d(-221.109450063087,126.799320794402,15.3608053050412 );
  source[6]=Point3d(-78.0765742134541,-128.140504567883,6.41012075521999 );
  source[7]=Point3d(-16.3506457593322,165.525661837821,4.33956337564149 );
  source[8]=Point3d(-159.020769889878,61.8758146649170,18.406748689435);
  
 /* Eigen::Matrix3d Rt;
  //Mat t1;
  Eigen::Vector3d tt;
  Eigen::Matrix4d result=tf.matrix();
   Rt<<result(0,0),result(0,1),result(0,2),
    result(1,0),result(1,1),result(1,2),
    result(2,0),result(2,1),result(2,2);
  tt<<result(0,3),result(1,3),result(2,3);
  /*std::cout << "R:\n"
            << Rt<<std::endl;
  std::cout << "t:\n"
            << tt<<std::endl;*/
  //std::vector<Eigen::Vector3d> sourceg;
  /*Eigen::Vector3d source1;
  Eigen::Vector3d p;
  for (int i = 0; i < target.size(); ++i) {
      source1<<target.at(i).x,target.at(i).y,target.at(i).z;
      p=(Rt*source1+tt);
   
      double x=p(0,0),y=p(1,0),z=p(2,0);
      source[i]=Point3f(x,y,z);
      //source[i]=p;
  }

  std::cout << "truth:\n" << tf.matrix() << std::endl;*/
  
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  //pose_estimation_ICP_ceres(target,source,R,t);
  
  Eigen::Matrix3d R1;
  //Mat t1;
  Eigen::Vector3d t1;
  
  /**	ICP_SVD求初值*/
  relative_positioning::pose_estimation_3d3d(target,source,R1,t1);
  /*std::cout << "R1:\n"
            << R1<<std::endl;
  std::cout << "t1:\n"
            << t1<<std::endl;*/
  
  /**将R，t写成李代数*/
  Sophus::SE3d SE3_Rt(R1,t1);	
  Eigen::Matrix<double,6,1> se3RT=SE3_Rt.log();
  
 /* std::cout << "SE3D:\n"
            << se3RT<<std::endl;*/
 
 /**	Ceres计算ICP*/
 relative_positioning::pose_estimation_ICP_ceres(target,source,R,t,se3RT);
	    
	    

  return 0;

}
