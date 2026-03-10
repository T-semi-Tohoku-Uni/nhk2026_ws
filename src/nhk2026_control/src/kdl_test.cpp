#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{
std::string load_xacro_as_urdf(const std::string & xacro_file)
{
    const std::string command = "xacro " + xacro_file;
    std::array<char, 4096> buffer{};
    std::string output;

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("Failed to run xacro.");
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        output += buffer.data();
    }

    if (pclose(pipe.release()) != 0) {
        throw std::runtime_error("xacro failed for file: " + xacro_file);
    }

    return output;
}
}  // namespace

int main()
{
    const std::string xacro_file =
        ament_index_cpp::get_package_share_directory("nhk2026_sim") + "/urdf/ide_arm.xacro";

    std::string urdf_xml;
    try {
        urdf_xml = load_xacro_as_urdf(xacro_file);
    } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    KDL::Tree tree;

    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        std::cerr << "Failed to parse URDF into KDL tree." << std::endl;
        return 1;
    }

    const std::string base_link = "arm_base";
    const std::string end_link = "tcp_link";

    KDL::Chain chain;
    if (!tree.getChain(base_link, end_link, chain)) {
        std::cerr << "Failed to extract KDL chain from "
                  << base_link << " to " << end_link << std::endl;
        return 1;
    }

    std::cout << "Chain joints: " << chain.getNrOfJoints() << std::endl;
    std::cout << "Chain segments: " << chain.getNrOfSegments() << std::endl;


    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // 現在の関節角の例
    KDL::JntArray q_current(chain.getNrOfJoints());
    q_current(0) = 0.4;
    q_current(1) = 0.1;
    q_current(2) = -0.2;

    KDL::Frame current_pose;
    int fk_status = fk_solver.JntToCart(q_current, current_pose);
    if (fk_status < 0) {
        std::cerr << "FK failed." << std::endl;
        return 1;
    }

    double roll, pitch, yaw;
    current_pose.M.GetRPY(roll, pitch, yaw);

    std::cout << "\nCurrent pose from FK:" << std::endl;
    std::cout << "  position = ["
              << current_pose.p.x() << ", "
              << current_pose.p.y() << ", "
              << current_pose.p.z() << "]" << std::endl;
    std::cout << "  rpy = ["
              << roll << ", "
              << pitch << ", "
              << yaw << "]" << std::endl;



    KDL::ChainIkSolverPos_LMA ik_solver(chain);

    // seed は現在角を入れるのが定石
    KDL::JntArray q_seed(chain.getNrOfJoints());
    q_seed(0) = 0.0;
    q_seed(1) = 0.0;
    q_seed(2) = 0.0;

    // 目標手先姿勢
    KDL::Vector target_pos(-0.182751, 0.624709, 0.425394);
    KDL::Rotation target_rot = KDL::Rotation::RPY(-2.94159, -0, 0);
    KDL::Frame target_pose(target_rot, target_pos);

    KDL::JntArray q_result(chain.getNrOfJoints());
    int ik_status = ik_solver.CartToJnt(q_seed, target_pose, q_result);

    if (ik_status < 0) {
        std::cerr << "IK failed." << std::endl;
        return 1;
    }

    std::cout << "\nIK result:" << std::endl;
    for (unsigned int i = 0; i < q_result.rows(); ++i) {
        std::cout << "  q[" << i << "] = " << q_result(i) << std::endl;
    }

}
