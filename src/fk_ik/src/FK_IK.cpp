#include "FK_IK.hpp"

/*************** class FK_IK_Service *********************************************************************************/

FK_IK_Service::FK_IK_Service( ros::NodeHandle& _nh ){
    ROS_INFO( "FK/IK Node starting ..." );
    _nh = _nh;
    
    // FIXME: INIT FUNCS GO HERE
    paramsOK = fetch_params(); // This must be done before setting up the arm!
    setup_kin_chain();
    if( paramsOK )  ROS_INFO( "Parameters located and loaded!" );  else  ROS_ERROR( "There was a PROBLEM loading parameters!" );
}

bool FK_IK_Service::setup_kin_chain(){
    // Init the IK solver and retrieve the kin chain from it

    // 0. Init
    string contents   , // Processed contents of the URDF
           param_name , // Param search result (not used, but req'd arg)
           urdf_param = "/robot_description";
    bool   valid      = 0;

    // 2. Search for parameter and fetch its contents
    // if( _nh.searchParam( urdf_param , param_name ) ){
    //     ROS_INFO("TEST: Found paramater");
    //     int i = 0;
    //     _nh.getParam( urdf_param , contents );
    // }else{
    //     // ROS_ERROR( ( "Could not find parameter!: " + urdf_param ).c_str() );
    //     ROS_ERROR( "Could not find parameter!: %s" , urdf_param.c_str() );
    //     return false;
    // }  

    // 3. Init solver and retrieve the kinematic chain
    tracik_solver = new TRAC_IK::TRAC_IK( base_link_name , end_link_name , urdf_param , IK_timeout , IK_epsilon );
    valid /*---*/ = tracik_solver->getKDLChain( chain );
    if( !valid ){  ROS_ERROR("There was no valid KDL chain found");  return false;  }

    // 4. Fetch the joint limits
    valid = tracik_solver->getKDLLimits( ll , ul );
    if( !valid ){  ROS_ERROR("There were no valid KDL joint limits found");  return false;  }

    // 5. Enforce that the joint limits are consistent with the number of joints
    N_joints = chain.getNrOfJoints();
    assert( N_joints == ll.data.size() );
    assert( N_joints == ul.data.size() );
    ROS_INFO( "SUCCESS: Loaded kinematic model with %d joints!" , N_joints );

    // 6. Init joints
    q = KDL::JntArray( N_joints );
    for( uint j = 0 ; j < N_joints ; j++ ){  q(j) = ( ll(j) + ul(j) ) / 2.0;  }

    return true;
}

bool FK_IK_Service::fetch_params(){
    return
        _nh.getParam( "base_link"   , base_link_name ) &&
        _nh.getParam( "end_link"    , end_link_name  ) &&
        _nh.getParam( "IK_timeout"  , IK_timeout     ) &&
        _nh.getParam( "IK_epsilon"  , IK_epsilon     )    ;
}

FK_IK_Service::~FK_IK_Service(){
    if( tracik_solver ) delete tracik_solver;
    if( fk_solver )     delete fk_solver;
}


/*************** MAIN NODE *******************************************************************************************/

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments
	srand( time( 0 ) ); // Random seed based on the current clock time
	
	/***** Preliminary { Setup , Instantiation , Planning  } *****/

    string NODE_NAME = "UR_FK_IK";
    float  RATE_HZ   = 100.0;
	
	// 0. Init ROS  &&  Register node
	ros::init( argc , argv , NODE_NAME );
	
	// 1. Fetch handle to this node
	ros::NodeHandle nodeHandle;
    FK_IK_Service nodeObj( nodeHandle );
	
	// 2. Init node rate
	ros::Rate heartbeat( RATE_HZ );
	

	/***** PRE-LOOP WORK *****/

	
	// N-1. Notify
	// ROS_INFO( string( string( "[" ) + NODE_NAME + "] Init OK and about to run ..." ).c_str() );
	ROS_INFO( "[%s] Init OK and about to run ..." , NODE_NAME.c_str() );

	
	// N. Main loop
	while( ros::ok() ){ // While neither node nor ROS has been shut down
		
		/***** LOOP WORK *****/


		break; // NOT EVEN ONCE
		
		
		/***** LOOP TIMEKEEPING *****/
		ros::spinOnce(); // - Process messages
		heartbeat.sleep(); // Sleep for remainder of period
	}
	
	// N+1. Notify  &&  Exit
	
	// ROS_INFO( string( string( "[" ) + NODE_NAME + "] Exit OK, Goodbye!" ).c_str() );
    ROS_INFO( "[%s] Exit OK, Goodbye!" , NODE_NAME.c_str() );
	
	return 0; // I guess everything turned out alright at the end!
}