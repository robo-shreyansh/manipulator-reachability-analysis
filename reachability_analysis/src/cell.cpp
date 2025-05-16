#include "reachability_analysis/cell.hpp"


Cell::Cell() {}

Cell::Cell(double y, double z, double cell_size ) : ul_corner_y_(y), ul_corner_z_(z), side_(cell_size) {

    centre_y_ = ul_corner_y_ + side_/2;
    centre_z_ = ul_corner_z_ - side_/2;

    reachable_ = false;
}


// bool Cell::is_reachable(){
//     return reachable_;
// }


// void Cell::set_reachable(){
//     reachable_ = 1;
// }


// vector<double> Cell::get_centre(){
//     vector<double> centre = {centre_y_, centre_z_};
//     return centre;
// }


