#ifndef GRID_HPP
#define GRID_HPP

#include<vector>
#include<string>
#include "reachability_analysis/cell.hpp"

/*
Grid: made up of cells
-> 


*/

using namespace std;

class Grid {


    private:
        
        double cell_resolution_;
        
        
        string grid_string_;

    public:
        vector<vector<Cell>> cell_grid;
        double depth;
        double y_side;
        double z_side;
        double reachable_area = 0;
        int reachable_cell_num_;
        int num_cell_y;
        int num_cell_z;

        Grid();
        Grid(double y_length, double z_length, double x);

        void update_reachable_area();
        vector<vector<int>> get_reachability_grid();
        string printOut();

};

#endif