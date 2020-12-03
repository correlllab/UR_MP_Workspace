#include "FK_IK.hpp"

static bool _DEBUG = 1;

/*************** class FK_IK_Service *********************************************************************************/


/********** Init **********/

FK_IK_Service::FK_IK_Service( ros::NodeHandle& _nh ){
    ROS_INFO( "FK/IK Node starting ..." );
    _nh = _nh;
    
    // FIXME: INIT FUNCS GO HERE
    paramsOK = fetch_params(); // This must be done before setting up the arm!
    setup_kin_chain();
    if( paramsOK )  ROS_INFO( "Parameters located and loaded!" );  else  ROS_ERROR( "There was a PROBLEM loading parameters!" );
    setup_FK();
    setup_IK();
    init_services();
}

bool FK_IK_Service::setup_kin_chain(){
    // Init the IK solver and retrieve the kin chain from it

    // 0. Init
    string contents   , // Processed contents of the URDF
           param_name , // Param search result (not used, but req'd arg)
           urdf_param = "/robot_description";
    bool   valid      = 0;

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

    if( _DEBUG ){
        u_int32_t Nseg = chain.getNrOfSegments();
        cout << "### Created a Kinematic Chain ###" << endl 
             << "Base Link: _ " << base_link_name   << endl
             << "End Link: __ " << end_link_name    << endl
             << "Description: " << urdf_param       << endl
             << "IK Timeout:  " << IK_timeout       << endl
             << "IK Max Err:  " << IK_epsilon       << endl
             << "IK Samples:  " << N_IKsamples      << endl
             << "IK Fuzz: ___ " << IK_seed_fuzz     << endl
             << "Joints: ____ " << (int) N_joints   << endl
             << "Segments: __ " << Nseg             << endl;
        KDL::Segment seg;
        KDL::Joint   jnt;
        for( u_int32_t i = 0 ; i < Nseg ; i++ ){
            seg = chain.getSegment( i );
            jnt = seg.getJoint();
            cout << '\t' << i << ": " << seg.getName() << ", Joint: " << jnt.getName() << " - " << jnt.getTypeName() << endl;
        }
        
    }

    return true;
}

bool FK_IK_Service::fetch_params(){
    int  smpl = 50;
    bool okay = _nh.getParam( "base_link"          , base_link_name   ) &&
                _nh.getParam( "end_link"           , end_link_name    ) &&
                _nh.getParam( "IK_timeout"         , IK_timeout       ) &&
                _nh.getParam( "IK_epsilon"         , IK_epsilon       ) &&
                _nh.getParam( "IK_samples"         , smpl             ) &&
                _nh.getParam( "IK_fuzz"            , IK_seed_fuzz     ) &&
                _nh.getParam( "UR_FKservice_TOPIC" , FK_srv_topicName ) &&
                _nh.getParam( "UR_IKservice_TOPIC" , IK_srv_topicName ) && 
                1;
    N_IKsamples = (u_short) smpl;
    return okay;
}


/********** FK / IK **********/

bool FK_IK_Service::setup_FK(){
    // Init the FK solver
    fk_solver = new KDL::ChainFkSolverPos_recursive( chain ); // Forward kin. solver
    return 1;
}

bool FK_IK_Service::setup_IK(){
    /* NOTHING TO DO HERE */
    return 1;
}

bool FK_IK_Service::check_q( const KDL::JntArray& q ){
    // Check that the currently-set joint config lies within the limits
    for( u_char i ; i < N_joints ; i++ ){
        if(  ( q(i) < ll(i) )  ||  ( q(i) > ul(i) )  ) 
            return false;
    }
    return true;
}

KDL::Frame FK_IK_Service::calc_FK( const boost::array<double,6>& jointConfig ){
    KDL::JntArray q = load_q( jointConfig );
    KDL::Frame    end_effector_pose;
    if( check_q( q ) ){
        if( fk_solver->JntToCart( q , end_effector_pose ) < 0 )  ROS_ERROR( "UNABLE to solve FK!" );
    }else{
        ROS_ERROR( "Received an FK request OUTSIDE of loaded robot's joint limits!" );
    }
    return end_effector_pose;
}


bool FK_IK_Service::FK_cb( ur_fk_ik::FK::Request& Req, ur_fk_ik::FK::Response& Rsp ){
    if( _DEBUG )  cout << "FK service invoked!" << endl;

    cout << Req << endl;
    cout << Req.q_joints << endl;
    cout << Req.q_joints[0] << endl;

    Rsp.poseMatx = KDL_frame_to_response_arr(  calc_FK( Req.q_joints )  );
    
    if( _DEBUG )  cout << "FK packed a response: " << Rsp << endl;

    return 1;
}

boost::array<double,7> FK_IK_Service::calc_IK( const boost::array<double,22>& bigIKarr ){
    
    boost::array<double,16> reqPose; // 4x4 homog pose
    boost::array<double, 6> reqSeed; // 1x6 joint seed (search begins here)
    
    for( u_char i = 0 ; i < 22 ; i++ ){
        if( i < 16 ){  reqPose[ i    ] = bigIKarr[i];  }
        else{          reqSeed[ i-16 ] = bigIKarr[i];  }
    }

    KDL::Frame    reqFrame = request_arr_to_KDL_frame( reqPose );
    KDL::JntArray seedArr  = request_arr_to_KDL_arr( reqSeed );
    KDL::JntArray result;
    int /*-----*/ valid = 0;

    if( _DEBUG )  cout << "Request vars loaded!" << endl;

    size_t i = 0;

    for( i = 0 ; i < N_IKsamples ; i++ ){
        valid = tracik_solver->CartToJnt( seedArr , reqFrame , result );
        // If the solution is valid, stop
        if( valid > 0 ){  
            break;
        // Else nudge seed and try again
        }else{
            fuzz_seed_array( seedArr , IK_seed_fuzz );  
        } 
    }

    if( _DEBUG )  cout << "Exit IK after " << i+1 << " tries" << endl;

    return IK_soln_to_IK_arr( result , valid );
}

bool FK_IK_Service::IK_cb( ur_fk_ik::IK::Request& Req , ur_fk_ik::IK::Response& Rsp ){

    if( _DEBUG )  cout << "Entered the IK callback!" << endl;

    Rsp.q_joints_valid = calc_IK( Req.pose_seed );

    // FIXME: A SECOND ATTEMPT AT A TIGHTER EPS

    bool valid         = Rsp.q_joints_valid[6] > 0;

    if( _DEBUG )  cout << "Valid solution?: " << yesno( valid ) << ", ";
    if( _DEBUG )  cout << "Solution Obtained: " << Rsp.q_joints_valid << endl;

    return 1;
}

/********** Services **********/

bool FK_IK_Service::init_services(){
    // string servName = "serv" ;
    FKservice = _nh.advertiseService( FK_srv_topicName , &FK_IK_Service::FK_cb , this );
    IKservice = _nh.advertiseService( IK_srv_topicName , &FK_IK_Service::IK_cb , this );
    return 1;
}


/********** Shutdown **********/

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


		// break; // NOT EVEN ONCE
		
		
		/***** LOOP TIMEKEEPING *****/
		ros::spinOnce(); // - Process messages
		heartbeat.sleep(); // Sleep for remainder of period
	}
	
	// N+1. Notify  &&  Exit
	
	// ROS_INFO( string( string( "[" ) + NODE_NAME + "] Exit OK, Goodbye!" ).c_str() );
    ROS_INFO( "[%s] Exit OK, Goodbye!" , NODE_NAME.c_str() );
	
	return 0; // I guess everything turned out alright at the end!
}