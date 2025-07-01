// Copyright (c) 2023.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <iostream>
#include <filesystem> 
#include <cmath>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo_map_creator_interface/srv/map_request.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <boost/gil.hpp>
#include <boost/gil/io/dynamic_io_new.hpp>
#include <boost/gil/extension/io/png/old.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <octomap/octomap.h>

namespace gazebo
{

class GazeboMapCreator : public gazebo::SystemPlugin
{
  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Service<gazebo_map_creator_interface::srv::MapRequest>::SharedPtr  map_service_;
  physics::WorldPtr world_;

  // create entityIdLookup 
  std::unordered_map<std::string, int> classIdLookup_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  public: void Load(int argc, char ** argv)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);      
    } 
    
    ros_node_ = gazebo_ros::Node::Get();

    // Get a callback when a world is created
    this->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
      std::bind(&GazeboMapCreator::OnWorldCreated, this, std::placeholders::_1));
 
  }

  public: void OnWorldCreated(const std::string & _world_name)
  {
      world_created_event_.reset();
      world_ = gazebo::physics::get_world(_world_name);

      auto models = world_->Models();
      int labelCounter = 1;
      for (const auto& model : models) {
        std::string name = model->GetName();
        std::cout << "Model: " << name << std::endl;

        // For example, add all model names to the lookup with unique IDs
        classIdLookup_[name] = labelCounter++;
      }
            
      // Create service for map creation request
      map_service_ = ros_node_->create_service<gazebo_map_creator_interface::srv::MapRequest>(
      "/world/save_map",
      std::bind(
        &GazeboMapCreator::OnMapCreate, this,
        std::placeholders::_1, std::placeholders::_2));
  }
  
  public: void DisableRobotCollisions(const std::string& model_name)
  {
    physics::ModelPtr model = world_->ModelByName(model_name);
    if (!model)
    {
        std::cout << "Model " << model_name << " not found in the world!" << std::endl;
        return;
    }

    std::cout << "Disabling collisions for model: " << model_name << std::endl;

    for (auto link : model->GetLinks())
    {
        for (auto collision : link->GetCollisions())
        {
            collision->SetCollideBits(0);
            collision->SetCategoryBits(0);
        }
    }
  }

  void RestoreRobotCollisions(const std::string& model_name)
  {
      physics::ModelPtr model = world_->ModelByName(model_name);
      if (!model)
      {
          std::cout << "Model " << model_name << " not found in the world!" << std::endl;
          return;
      }

      std::cout << "Restoring collisions for model: " << model_name << std::endl;

      for (auto link : model->GetLinks())
      {
          for (auto collision : link->GetCollisions())
          {
              collision->SetCollideBits(GZ_ALL_COLLIDE);
              collision->SetCategoryBits(GZ_ALL_COLLIDE);
          }
      }
  }

public: void OnMapCreate(const std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Request> _req,
                         std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Response> _res )
{
  RCLCPP_INFO(ros_node_->get_logger(), "Received message");

  // Build label map filename first
  std::filesystem::path filePath(_req->filename);
  std::filesystem::path parentDir = filePath.parent_path();
  std::filesystem::path labelMapPath = parentDir / "label_map.txt";

  // Check if it exists
  if (std::filesystem::exists(labelMapPath)) {
      std::cout << "Label map already exists: " << labelMapPath << std::endl;
  } else {
      // If not, build and write the label map

      // Collect and sort
      std::vector<std::pair<int, std::string>> entries;
      for (const auto& kv : classIdLookup_) {
          entries.emplace_back(kv.second, kv.first);
      }
      std::sort(entries.begin(), entries.end());

      // Write to file
      std::ofstream label_map_file(labelMapPath);
      if (label_map_file.is_open()) {
          for (const auto& e : entries) {
              label_map_file << e.first << " " << e.second << "\n";
          }
          label_map_file.close();
          std::cout << "Saved label map to " << labelMapPath << std::endl;
      } else {
          std::cerr << "Failed to open label map file: " << labelMapPath << std::endl;
      }
  }

  // Pause simulation so robot doesn't fall
  world_->SetPaused(true);

  // Disable robot collisions so it doesn't occlude the scan
  DisableRobotCollisions("velodyne_sensor");

  // Lookup reference frame pose
  physics::ModelPtr ref_model = world_->ModelByName("velodyne_sensor");
  physics::LinkPtr ref_link = ref_model->GetLink("base_link");
  if (!ref_model)
  {
      RCLCPP_ERROR(ros_node_->get_logger(), "Reference frame model '%s' not found!", "velodyne_sensor");
      _res->success = false;
      world_->SetPaused(false);
      return;
  }
  ignition::math::Pose3d ref_pose = ref_link->WorldPose();

  // Compute bounding box sizes
  float size_x = _req->upperleft.x - _req->lowerright.x;
  float size_y = _req->lowerright.y - _req->upperleft.y;  // y negative
  float size_z = _req->upperleft.z - _req->lowerright.z;


  if (size_x <=0 || size_y >=0 || size_z <=0)
  {
      RCLCPP_ERROR(ros_node_->get_logger(), "Invalid coordinates");
      _res->success = false;
      world_->SetPaused(false);
      return;
  }

  int num_points_x = static_cast<int>(std::abs(size_x) / _req->resolution) + 1;
  int num_points_y = static_cast<int>(std::abs(size_y) / _req->resolution) + 1;
  int num_points_z = static_cast<int>(std::abs(size_z) / _req->resolution) + 1;

  float step_x = size_x / num_points_x;
  float step_y = size_y / num_points_y;
  float step_z = size_z / num_points_z;

  int dims = 6;

  if (_req->skip_vertical_scan)
  {
    num_points_z = 2;
    step_z = size_z;
    dims = 4;
  }

  // Ray setup
  gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::string entityName;
  ignition::math::Vector3d start, end;
  double dx, dy, dz, dist;
  cloud.width = num_points_x;
  cloud.height = num_points_y;

  struct PointMask {
      int x, y, z;
  };
  PointMask directions[6] = {
      {-1,0,0},
      {1,0,0},
      {0,1,0},
      {0,-1,0},
      {0,0,1},
      {0,0,-1}
  };

  // Store per-point label
  std::vector<int> labels;

  for (int x =0; x < num_points_x; ++x)
  {
    std::cout << "\rPercent complete: " << x * 100.0 / num_points_x << "%    " << std::flush;

    // Coordinates relative to reference frame
    double local_x = _req->lowerright.x + x * step_x;

    for (int y = 0; y < num_points_y; ++y)
    {
      double local_y = _req->upperleft.y + y * step_y;

      for (int z=0; z<num_points_z; ++z)
      {
        double local_z = _req->lowerright.z + z * step_z;
        //ignition::math::Vector3d rel(local_x, local_y, local_z);

        // Transform to world
        ignition::math::Vector3d cur(local_x, local_y, local_z); //ref_pose.Pos() + ref_pose.Rot().RotateVector(rel);       

        // For each direction
        for (int i = 0; i < dims; ++i)
        {
          dx = directions[i].x * step_x * _req->range_multiplier;
          dy = directions[i].y * step_y * _req->range_multiplier;
          dz = directions[i].z * step_z * _req->range_multiplier;
          ignition::math::Vector3d end_point = cur + ignition::math::Vector3d(dx, dy, dz);

          ray->SetPoints(cur, end_point);
          ray->GetIntersection(dist, entityName);
          if (!entityName.empty())
          {
            std::string model_name = entityName.substr(0, entityName.find("::"));
            auto it = classIdLookup_.find(model_name);
            uint32_t label = 0; // Default label for unknown
            if (it != classIdLookup_.end()) {
                label = it->second;
            }

            cloud.push_back(pcl::PointXYZ(cur.X(), cur.Y(), cur.Z()));
            labels.push_back(label);
            break;
          }
        }
      }
    }
  }

  std::cout << std::endl << "Completed calculations, writing to output" << std::endl;

  if (!_req->filename.empty() && cloud.size()>0)
  {
      pcl::io::savePCDFileASCII(_req->filename+".pcd", cloud);
      RCLCPP_INFO(ros_node_->get_logger(), "Saved pointcloud to: %s.pcd", _req->filename.c_str());

      std::ofstream labelFile(_req->filename + ".labels");
      for (const auto& l : labels) {
          labelFile << l << "\n";
      }
      labelFile.close();
  }

  RestoreRobotCollisions("velodyne_sensor");
  world_->SetPaused(false);
  _res->success = true;
}



  // public: void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view)
  // {
  //   // Write image to pgm file

  //   int h = view.height();
  //   int w = view.width();

  //   std::ofstream ofs;
  //   ofs.open(filename+".pgm");
  //   ofs << "P2" << '\n';          // grayscale
  //   ofs << w << ' ' << h << '\n'; // width and height
  //   ofs << 255 <<  '\n';          // max value
  //   for (int y = 0; y < h; ++y){
  //     for (int x = 0; x < w; ++x){
  //       // std::cout << (int)view(x, y)[0];
  //       ofs << (int)view(x, y)[0] << ' ';
  //     }
  //     ofs << '\n';
  //   }
  //   ofs.close();
  // }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboMapCreator)
}
