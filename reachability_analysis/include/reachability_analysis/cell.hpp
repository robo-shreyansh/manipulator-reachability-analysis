#ifndef CELL_HPP
#define CELL_HPP

#include<vector>

/*
Cell: smallest square unit of the grid, initialzed with the corner point, and the side length

reachable_ : is the cell reachable by the manipulator (without self collision)
rest are self explanatory.
*/
using namespace std;

class Cell {

    private:
        double side_;
        double area_;

        double ul_corner_y_;
        double ul_corner_z_;

        double centre_y_;
        double centre_z_;
        
        bool reachable_;
        
    public:
        // int ind_y;
        // int ind_z;

        Cell();
        Cell(double y, double z, double cell_size);

        inline bool is_reachable() {return reachable_;}
        inline void set_reachable() {reachable_=true;}
        inline vector<double> get_centre() {return {centre_y_, centre_z_};}

};

#endif