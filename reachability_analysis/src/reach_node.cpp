
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>


#include "reachability_analysis/cell.hpp"
#include "reachability_analysis/grid.hpp"
#include "reachability_analysis/utils.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

using namespace std;

namespace plt = matplotlibcpp;


int main(int argc, char** argv){
    
    
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("reach_node", node_options); 
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    RCLCPP_INFO(node->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

    if (kinematic_model){
        RCLCPP_INFO(node->get_logger(), "Robot kinematic model loaded");
        }
    
    else{
        RCLCPP_ERROR(node->get_logger(), "Model not loaded!");
        }

    moveit::core::RobotStatePtr robot_state (new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();

    // getting the joint angles in the manipulator ur10 planning group
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ur10_planning_group");
    robot_state->enforceBounds();
    
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    

    vector<Grid> grids;

    grid_utils::grid_builder(grids, {3,3}); // build grid
    grid_utils::grid_checker(grids, node, joint_model_group, robot_state, planning_scene); //check reachability
    
    // printing it in the terminal as well
    for (auto & grid : grids){
        string out;
        out = grid.printOut();
        RCLCPP_INFO(node->get_logger(), "Depth: %f, Reachable cells: %d", grid.depth, grid.reachable_cell_num_);
        RCLCPP_INFO(node->get_logger(), "\n%s\n\n", out.c_str());  
    }

    std::sort(grids.begin(), grids.end(), grid_utils::grid_compare); // store in desc order per reachability area
    
    plotter_utils::plot_grids(grids); // plot some of it

    
   
    rclcpp::shutdown();

    return 0;
}

