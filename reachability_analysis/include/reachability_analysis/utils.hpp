#ifndef UTILS_HPP
#define UTILS_HPP

#include<vector>
#include<string>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include "reachability_analysis/cell.hpp"
#include "reachability_analysis/grid.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

/*
This header contains functions:
grid_compare: comaprator function to check which grid has more reachable cells 
grid_builder: for building a grid at a certain depth, dimensions, and resolution
grid_checker: for each cell in grid, checks if the centre of each cell is reachable (without self collision), and builds occupancy grid
plotter: plots the obtained occupancy grid for optimal and some non-optimal base positons
*/

using namespace std;
namespace plt = matplotlibcpp;

namespace grid_utils{
    bool grid_compare(const Grid &a, const Grid &b){
        return a.reachable_area > b.reachable_area;
    }
    
    void grid_builder(vector<Grid> &grids, vector<double> grid_size){
    
        // builds different wall grids at different distances from the robot
    
        double depth = 0.15; // initial distance ; base itself is 10cm thick.
        double depth_resolution = 0.05; // checks at distance multiple of 5cm plus the initial depth
        
        while (depth < 1.4){ // 1.4 is more than max reach
            grids.push_back(Grid(grid_size[0],grid_size[1],depth));
            depth += depth_resolution;
        }
    }
    
    void grid_checker(vector<Grid> &grids, auto &node, const moveit::core::JointModelGroup* joint_model_group, moveit::core::RobotStatePtr &robot_state, planning_scene::PlanningScene &planning_scene){
        
        // check if end-eff can reach each cell's centre without self-collision
        
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        
        bool found_ik = false;
        
        double timeout = 0.005; //for solver : default setting;
        int reach_count = 0;    // number of cells reachable
    
        for (auto &grid : grids){
            reach_count = 0;
            RCLCPP_INFO(node->get_logger(), "Checking reachable cells. Distance from wall: %f" , grid.depth);
            for (int i = 0; i<grid.num_cell_y; i++){
                for (int j = 0; j< grid.num_cell_z; j++){
    
                    target_pose.translation() << grid.depth, grid.cell_grid[i][j].get_centre()[0], grid.cell_grid[i][j].get_centre()[1];
                
                    found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);
                    if (found_ik){
                        
                        // change the robot state to the newly found joint_pos ; 
                        robot_state->update();
                        planning_scene.setCurrentState(*robot_state);
    
                        // setting up collision 
                        collision_detection::CollisionRequest collision_request;
                        collision_detection::CollisionResult collision_result;
                        collision_request.max_contacts = 25;
                        
                        //checking for self collision
                        planning_scene.checkSelfCollision(collision_request, collision_result);
    
                        if (collision_result.collision){
                            // RCLCPP_INFO_STREAM(node->get_logger(), "Collision!! ");
                        }
                        else{   
                            grid.cell_grid[i][j].set_reachable();
                            reach_count++;
                        }
                    }
                }
            }
            RCLCPP_INFO(node->get_logger(), "Reachable cells: : %d , Distance from the wall: %f\n\n",reach_count, grid.depth);
            grid.update_reachable_area();
    }
    }
}


namespace plotter_utils{
    void plot_grids(vector<Grid>grids){
        int ncols, nrows;  // Grid size
        ncols = grids[0].num_cell_y;
        nrows = grids[0].num_cell_z;
        const int colors = 1;
        
        PyObject* mat;

        std::vector<double> xticks;
        std::vector<std::string> xticklabels;
        std::vector<double> yticks;
        std::vector<std::string> yticklabels;
        // // double step = grids[0].y_side / ncols;  // Step size to map grid to -1.5 to 1.5

        for (int i = 0; i <= ncols; i += ncols/6) {
            double x_value = -grids[0].y_side/2 + i * grids[0].y_side / ncols;
            xticks.push_back(i);  
            xticklabels.push_back(std::to_string(x_value).substr(0, 4));
        }

        for (int i = 0; i <= nrows; i += nrows/6 ){
            double y_value = i * grids[0].z_side / nrows;
            yticks.push_back(i);  
            yticklabels.push_back(std::to_string(grids[0].z_side - y_value).substr(0, 4));
            // std::reverse(yticklabels.begin(), yticklabels.end());
        }


        for (int grid = 0; grid<grids.size(); grid+=6){
            
            plt::figure(grid+1);
            
            vector<vector<int>> reachability_grid = grids[grid].get_reachability_grid();
            nrows = grids[grid].num_cell_y;
            ncols = grids[grid].num_cell_z;

            std::vector<float> z(ncols * nrows);
            for (int j=0; j<nrows; ++j) {
                for (int i=0; i<ncols; ++i) {
                    z.at(ncols * j + i) = reachability_grid[i][j]; //needs flattening
                }
            }

            const float* zptr = &(z[0]);

            if (grid==0){
                plt::title("Optimal Base Position: " + to_string(grids[grid].depth)+ " m. | Area : "+to_string(grids[grid].reachable_area) +" m^2");
            }
            else{
                plt::title("Base Position: " + to_string(grids[grid].depth)+ " m. | Area : "+to_string(grids[grid].reachable_area) +" m^2");
            }

            plt::imshow(zptr, nrows, ncols, colors, {}, &mat);
            plt::colorbar(mat);
            
            
            
            plt::xlabel("Y Position (m)");
            plt::ylabel("Z Position (m)");
            // updating the ticks to inc. actual wall dimensions 
            plt::xticks(xticks, xticklabels);
            plt::yticks(yticks, yticklabels);
        
            }

        plt::show();
        }
}


#endif 