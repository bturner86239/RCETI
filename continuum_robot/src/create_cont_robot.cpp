#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string.h>

using namespace std;
#define NoOfSegments 3
#define NoOfDisks 10
#define Length 5
ofstream myfile;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("create_cont_robot");
    std::string path = ament_index_cpp::get_package_share_directory("continuum_robot") + "/urdf/robot_model.urdf";
    remove(path.c_str());
    myfile.open(path.c_str(), std::fstream::app);
    myfile << "<?xml version=\"1.0\"?>" << endl;
    myfile << "<robot name=\"continuum_robot\">" << endl;
    myfile << "<link name=\"base_link\"/>" << endl;

    for (int disk = 0; disk < NoOfDisks; disk++)
    {
        myfile << "<link name=\"S0L" << disk << "\">" << endl;
        myfile << "<visual>" << endl;
        myfile << "<geometry>" << endl;
        myfile << "<cylinder length=\"0.1\" radius=\"0.5\"/>" << endl;
        myfile << "<origin rpy=\"0 0 0\" xyz=\"0 0 " << (disk / (NoOfDisks - 1)) * Length << "\"/>" << endl;
        myfile << "</geometry>" << endl;
        myfile << "</visual>" << endl;
        myfile << "</link>" << endl;
        myfile << endl;
        myfile << "<joint name=\"S0J" << disk << "\" type=\"floating\">" << endl;
        myfile << "<parent link=\"base_link\"/>" << endl;
        myfile << "<child link=\"S0L" << disk << "\"/>" << endl;
        myfile << "</joint>" << endl;
        myfile << endl;
    }
    myfile << "</robot>" << endl;
    myfile.close();
    rclcpp::shutdown();
    return 0;
}
