#ifndef FK_IK_H
#define FK_IK_H



/***** Included Libs *****/

/** Standard **/
#include <fstream>
#include <exception>
#include <signal.h>
#include <string>
#include <iostream>
using std::ifstream;
using std::string;
using std::cout;
using std::endl;

/** Parsing **/
#include <boost/date_time.hpp>
#include <tinyxml.h>
#include <jsoncpp/json/json.h>

/** Basic ROS **/
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>

/** FK/IK/MP **/
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

/** Messages **/
// ROS Standard
#include <std_msgs/Float32MultiArray.h>
// Custom
#include <ur_fk_ik/FK.h>
#include <ur_fk_ik/IK.h>


/*************** class FK_IK_Service *********************************************************************************/

class FK_IK_Service{

/***** Public *****/ public:

ros::ServiceServer FKservice;
ros::ServiceServer IKservice;

FK_IK_Service( ros::NodeHandle& _nh );

bool FK_cb( ur_fk_ik::FK::Request& req, ur_fk_ik::FK::Response& rsp );
bool IK_cb( ur_fk_ik::IK::Request& req, ur_fk_ik::IK::Response& rsp );

bool init_services();

~FK_IK_Service();


/***** Protected *****/ protected:

/** ROS Node **/
ros::NodeHandle _nh; 

/** JSON Params **/
string FK_srv_topicName ,
       IK_srv_topicName ,
       URDF_param       ,
       SRDF_param       ,
       URDF_contents    ,
       SRDF_contents    ,
       base_link_name   , 
       end_link_name    , 
       _PKG_NAME        = "ur_fk_ik";
bool paramsOK;

/** Robot Description **/
KDL::Chain    chain; // ---- Kin chain
u_char /*--*/ N_joints; // # of joints in the kin chain
KDL::JntArray ll , // ------ lower joint limits
              ul , // ------ upper joint limits
              q  ; // ------ Current joint config

/** FK Solver **/
KDL::ChainFkSolverPos_recursive* fk_solver; //(chain); // Forward kin. solver

/** IK Solver and Params **/
TRAC_IK::TRAC_IK* tracik_solver = nullptr;
u_short /*----*/ N_IKsamples;
double /*-----*/ IK_timeout   , 
                 IK_epsilon   ,
                 IK_seed_fuzz ;

/** Internal Functions **/
bool setup_kin_chain();
bool fetch_params();
bool setup_FK();
bool setup_IK();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};

#endif