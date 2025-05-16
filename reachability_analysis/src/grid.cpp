#include "reachability_analysis/grid.hpp"

using namespace std;

Grid::Grid() {}

Grid::Grid(double y_length, double z_length, double x) : y_side(y_length), z_side(z_length), depth(x) {
            
    // resolution :
    cell_resolution_ = 0.05; 
    num_cell_y = static_cast<int> (y_length / cell_resolution_);
    num_cell_z = static_cast<int> (z_length / cell_resolution_);
    cell_grid.resize(num_cell_y, std::vector<Cell>(num_cell_z));


    for (int i = 0; i<num_cell_y; i++){
        // vector<Cell> cell_row;
        for (int j = 0; j<num_cell_z; j++){
            cell_grid[i][j] = Cell( -y_length/2 + i*cell_resolution_ , z_length - j*cell_resolution_, cell_resolution_);

        }
    }
    reachable_cell_num_ = 0;
}


void Grid::update_reachable_area(){
    // stores the 
    for (int j = 0; j<num_cell_z; j++){
        for (int i = 0; i<num_cell_y; i++){
            if (cell_grid[j][i].is_reachable()){
                reachable_cell_num_++;
                reachable_area += cell_resolution_*cell_resolution_;
            }
        }
    }
}

vector<vector<int>> Grid::get_reachability_grid(void){

    // Checks which cell is reachable and which is not
    // returns a 2D std::vector : sort of occupancy grid
    
    vector<vector<int>> reachability_grid;
    for (int j = 0; j<num_cell_z; j++){
        vector<int> row;
        for (int i = 0; i<num_cell_y; i++){
            if (cell_grid[j][i].is_reachable()){
                row.push_back(1);
            }
            else{
                row.push_back(0);
            }
        }
        reachability_grid.push_back(row);
    }
    return reachability_grid;
}


string Grid::printOut(void){
    // string test = "";

    for (int j = 0; j<num_cell_z; j++){
        for (int i = 0; i<num_cell_y; i++){
            if (cell_grid[i][j].is_reachable()){
                grid_string_+= ". ";
            }
            else{
                grid_string_+= "X ";
            }
        }
        grid_string_+= "\n";
    }
    return grid_string_;
}