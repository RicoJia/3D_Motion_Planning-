#ifndef __RICO_MAP_DATA_STRUCTURES_HPP__
#define __RICO_MAP_DATA_STRUCTURES_HPP__

#include <Eigen/Dense>

using namespace Eigen; 

namespace Util  {
struct Node {
    Node(){}
    Node(const VectorXd state, Node* parent = nullptr): state_(state), parent_(parent){}
    VectorXd state_; 
    Node* parent_; 
};
}


#endif /* end of include guard: __RICO_MAP_DATA_STRUCTURES_HPP__ */
