#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <iostream>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > vector_quad;
// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZ PointType;
// typedef pcl::PointXYZI PointType;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace mypcl
{
  struct pose
  {
    pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
         Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0),
         double _timestamp = 0.0):q(_q), t(_t), timestamp(_timestamp){}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    double timestamp;
  };

  // filePath 输入的 /home/pengll/hba_ws/src/HBA/data 路径
  // void loadPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num,
  //              std::string prefix = "") 
  // {
  //   std::stringstream ss;
  //   if(pcd_fill_num > 0)
  //     ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
  //   else
  //     ss << num;
  //   pcl::io::loadPCDFile(filePath + prefix + ss.str() + ".pcd", *pc);
  // }

  void loadPCD(std::string filePath, pcl::PointCloud<PointType>::Ptr& pc, double timestamp)
  {
    string pcdname = std::to_string(timestamp);
    //pcdname = pcdname.substr(0, 11);   // fast_livo前几个数据的pcd名字保存11位，后面的sbs保存全部
    string filename = filePath + "pcd/" + pcdname + ".pcd";
    if(std::filesystem::exists(filename)){
      if(pcl::io::loadPCDFile(filename, *pc)==-1){
        PCL_ERROR("Couldn't read file.\n");
      }
    }
    else{
      PCL_ERROR("Couldn't read file.\n");
    }
  }

  void savdPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num)
  {
    std::stringstream ss;
    if(pcd_fill_num > 0)
      ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
    else
      ss << num;
    pcl::io::savePCDFileBinary(filePath + ss.str() + ".pcd", *pc);
  }
  
  // 读取pose
  // std::vector<pose> read_pose(std::string filename,
  //                             Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
  //                             Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
  // {
  //   std::vector<pose> pose_vec;
  //   std::fstream file;
  //   file.open(filename);
  //   double tx, ty, tz, w, x, y, z;
  //   while(!file.eof())
  //   {
  //     file >> tx >> ty >> tz >> w >> x >> y >> z;
  //     Eigen::Quaterniond q(w, x, y, z);
  //     Eigen::Vector3d t(tx, ty, tz);
  //     pose_vec.push_back(pose(qe * q, qe * t + te));
  //   }
  //   file.close();
  //   return pose_vec;
  // }

  // 读取pose 记得时间戳 用于找到读取cloud
  std::vector<pose> read_pose(std::string filename,
                            Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
                            Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
  {
    std::vector<pose> pose_vec;
    std::fstream file;
    file.open(filename+"pose.txt");
    double timestamp, tx, ty, tz, w, x, y, z;
    while(!file.eof())
    {
      file >> timestamp >> tx >> ty >> tz >> x >> y >> z >> w;
      Eigen::Quaterniond q(w, x, y, z);
      Eigen::Vector3d t(tx, ty, tz);
      pose_vec.push_back(pose(qe * q, qe * t + te, timestamp));
    }
    file.close();
    return pose_vec;
  }

  void transform_pointcloud(pcl::PointCloud<PointType> const& pc_in,
                            pcl::PointCloud<PointType>& pt_out,
                            Eigen::Vector3d t,
                            Eigen::Quaterniond q)
  {
    size_t size = pc_in.points.size();
    pt_out.points.resize(size);
    for(size_t i = 0; i < size; i++)
    {
      Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
      Eigen::Vector3d pt_to;
      // if(pt_cur.norm()<0.3) continue;
      // pll我们输入的就是世界坐标系下的点云，所以不需要转
      // 原 pt_to = q * pt_cur + t;   // 从 visualize 可以看出
      pt_to = pt_cur;
      pt_out.points[i].x = pt_to.x();
      pt_out.points[i].y = pt_to.y();
      pt_out.points[i].z = pt_to.z();
      // pt_out.points[i].r = pc_in.points[i].r;
      // pt_out.points[i].g = pc_in.points[i].g;
      // pt_out.points[i].b = pc_in.points[i].b;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr append_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1,
                                                      pcl::PointCloud<pcl::PointXYZRGB> pc2)
  {
    size_t size1 = pc1->points.size();
    size_t size2 = pc2.points.size();
    pc1->points.resize(size1 + size2);
    for(size_t i = size1; i < size1 + size2; i++)
    {
      pc1->points[i].x = pc2.points[i-size1].x;
      pc1->points[i].y = pc2.points[i-size1].y;
      pc1->points[i].z = pc2.points[i-size1].z;
      pc1->points[i].r = pc2.points[i-size1].r;
      pc1->points[i].g = pc2.points[i-size1].g;
      pc1->points[i].b = pc2.points[i-size1].b;
      // pc1->points[i].intensity = pc2.points[i-size1].intensity;
    }
    return pc1;
  }

  pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
                                               pcl::PointCloud<PointType> pc2)
  {
    size_t size1 = pc1->points.size();
    size_t size2 = pc2.points.size();
    pc1->points.resize(size1 + size2);
    for(size_t i = size1; i < size1 + size2; i++)
    {
      pc1->points[i].x = pc2.points[i-size1].x;
      pc1->points[i].y = pc2.points[i-size1].y;
      pc1->points[i].z = pc2.points[i-size1].z;
      // pc1->points[i].r = pc2.points[i-size1].r;
      // pc1->points[i].g = pc2.points[i-size1].g;
      // pc1->points[i].b = pc2.points[i-size1].b;
      // pc1->points[i].intensity = pc2.points[i-size1].intensity;
    }
    return pc1;
  }

  double compute_inlier_ratio(std::vector<double> residuals, double ratio)
  {
    std::set<double> dis_vec;
    for(size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
      dis_vec.insert(fabs(residuals[3 * i + 0]) +
                     fabs(residuals[3 * i + 1]) + fabs(residuals[3 * i + 2]));

    return *(std::next(dis_vec.begin(), (int)((ratio) * dis_vec.size())));
  }

  void write_pose(std::vector<pose>& pose_vec, std::string path)
  {
    std::ofstream file;
    file.open(path + "pose.txt", std::ofstream::trunc);
    file.close();
    Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
    Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
    file.open(path + "pose.txt", std::ofstream::app);

    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      pose_vec[i].t << q0.inverse()*(pose_vec[i].t-t0);
      pose_vec[i].q.w() = (q0.inverse()*pose_vec[i].q).w();
      pose_vec[i].q.x() = (q0.inverse()*pose_vec[i].q).x();
      pose_vec[i].q.y() = (q0.inverse()*pose_vec[i].q).y();
      pose_vec[i].q.z() = (q0.inverse()*pose_vec[i].q).z();
      string pcdname = std::to_string(pose_vec[i].timestamp);
      //pcdname = pcdname.substr(0, 11);
      file //<< std::setprecision(12)
           << pcdname << " " 
           << pose_vec[i].t(0) << " "
           << pose_vec[i].t(1) << " "
           << pose_vec[i].t(2) << " "
           << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
           << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }

  void writeEVOPose(std::vector<double>& lidar_times, std::vector<pose>& pose_vec, std::string path)
  {
    std::ofstream file;
    file.open(path + "evo_pose.txt", std::ofstream::trunc);
    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      file << std::setprecision(18) << lidar_times[i] << " " << std::setprecision(6)
           << pose_vec[i].t(0) << " " << pose_vec[i].t(1) << " " << pose_vec[i].t(2) << " "
           << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
           << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }
}

#endif