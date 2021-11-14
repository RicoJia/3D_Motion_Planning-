#ifndef __RRT_2D_HPP__
#define __RRT_2D_HPP__
#include "rico_map_data_structures.hpp"
#include <functional>
#include <climits>
#include <vector>
#include <random>

#define UNINITIALIZED INT_MIN

namespace rico_rrt_plugin{
  class Rrt2D
  {
    public:
      using FreeSpacePointCheck = std::function<bool(const Eigen::Vector2d& )>; 
      using FreeSpacePathCheck = std::function<bool(const Eigen::Vector2d&, const Eigen::Vector2d&)>; 
      Rrt2D (const FreeSpacePointCheck& is_free_space_cell, const FreeSpacePathCheck& is_free_space_path);
      ~Rrt2D (){
      }

      // lower_bound_corner_: (x_lower, y_lower)
      void set_bounds(const Eigen::Vector2d& lower_bound_corner, const Eigen::Vector2d& upper_bound_corner){
          lower_bound_corner_ = lower_bound_corner; 
          upper_bound_corner_ = upper_bound_corner; 
      }
    
      std::vector<Eigen::Vector2d> solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal); 
    private:
        Util::Node get_random_map_point();  
        const FreeSpacePointCheck& is_free_space_cell_ = nullptr; 
        const FreeSpacePathCheck& is_free_space_path_ = nullptr; 
        Eigen::Vector2d upper_bound_corner_ = {UNINITIALIZED, UNINITIALIZED}; 
        Eigen::Vector2d lower_bound_corner_ = {UNINITIALIZED, UNINITIALIZED}; 
  };

  std::vector<Eigen::Vector2d> Rrt2D::solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal){
      std::vector<Eigen::Vector2d> path; 
      return path; 
  }
  
  Rrt2D::Rrt2D (const FreeSpacePointCheck& is_free_space_cell, const FreeSpacePathCheck& is_free_space_path): 
      is_free_space_cell_(is_free_space_cell), is_free_space_path_(is_free_space_path)
  { 
  }

  Util::Node Rrt2D::get_random_map_point(){
      static std::random_device dev; 
      static std::mt19937 rng(dev()); 
      static std::uniform_real_distribution<double> x_dist(lower_bound_corner_(0), upper_bound_corner_(0)); 
      static std::uniform_real_distribution<double> y_dist (lower_bound_corner_(1), upper_bound_corner_(1)); 
      Util::Node node(Vector2d(x_dist(rng), y_dist(rng))); 
  }


}



#endif /* end of include guard: __RRT_2D_HPP__ */
