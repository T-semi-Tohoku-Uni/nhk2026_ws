#include <string>
#include <fstream>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

int main()
{
    std::string urdf_file = ament_index_cpp::get_package_share_directory("nhk2026_sim") + "/urdf/ide_arm.xacro";
    std::ifstream file(urdf_file);
    KDL::Tree tree;
}