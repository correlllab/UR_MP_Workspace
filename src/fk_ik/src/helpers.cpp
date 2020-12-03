#include "helpers.hpp"

static bool _DEBUG = 0;


/******************** Utilities ********************/

double randrange( double rMin , double rMax ){
    // Get a pseudo-random number between `rMin` and `rMax`
    double f = (double) rand() / RAND_MAX;
    return rMin + f * (rMax - rMin);
}

string yesno( bool condition ){  return ( condition ? "YES" : "NO" );  }


/******************** Conversion ********************/

boost::array<double,16> KDL_frame_to_response_arr( const KDL::Frame& pose ){
    // Translate a KDL pose to a flattened pose
    boost::array<double,16> rtnArr;
    u_char k = 0;
    for( int i = 0 ; i < 4 ; i++ ){
        for( int j = 0 ; j < 4 ; j++ ){
            rtnArr[k] = pose( i , j );
            k++;
        }
    }
    return rtnArr;
}


/******************** Kinematics ********************/

KDL::JntArray load_q( const boost::array<double,6>& q_input ){
    // Convert a Boost array to KDL Joint Array

    KDL::JntArray q_return(6);

    if( _DEBUG ){
        cout << "Setting the joint model!" << endl;
        cout << "Got a vector with " << q_input.size() << " elements." << endl;
    }  

    for( u_char i = 0 ; i < 6 ; i++ ){  q_return(i) = q_input[i];  }

    return q_return;
}


void fuzz_seed_array( KDL::JntArray& seedArr , double fuzz_rad ){
    // Perturbate each element of `seedArr` by +/- `fuzz_rad`
    size_t len = seedArr.rows();
    fuzz_rad = abs( fuzz_rad );
    for( size_t i = 0 ; i < len ; i++ ){
        seedArr(i) = seedArr(i) + randrange( -fuzz_rad , +fuzz_rad );
    }
}


boost::array<double,7> IK_soln_to_IK_arr( const KDL::JntArray& jntArr , int valid ){
    // Pack the IK solution and its validity into a single boost array
    boost::array<double,7> rtnArr;
    for( size_t i = 0 ; i < 6 ; i++ ){  
        rtnArr[i] = jntArr(i);  
        if( _DEBUG )  cout << '\t' << i << endl;
    }
    rtnArr[6] = (double) valid;
    return rtnArr;
}

KDL::Frame request_arr_to_KDL_frame( const boost::array<double,16>& pose ){
    // Translate a flattened pose to a KDL pose
    size_t rotDex[9] = { 0, 1, 2,   4, 5, 6,  8, 9,10    }; // Rotation matrix
    size_t trnDex[3] = {          3,        7,        11 }; // Translation vector
    double rotTerms[9];
    double trnTerms[3];
    double rotTerm0 = pose[ rotDex[0] ];
    for( u_char i = 0 ; i < 9 ; i++ ){
        rotTerms[i] = pose[ rotDex[i] ];
        if( i < 3 )
            trnTerms[i] = pose[ trnDex[i] ];
    }
    return KDL::Frame( 
        KDL::Rotation( rotTerms[1] , rotTerms[1] , rotTerms[2] , rotTerms[3] , rotTerms[4] , rotTerms[5] , rotTerms[6] , rotTerms[7] , rotTerms[8] ) , 
        KDL::Vector( trnTerms[0] , trnTerms[1] , trnTerms[2] ) 
    );
}

KDL::JntArray request_arr_to_KDL_arr( const boost::array<double,6>& jntArr ){
    KDL::JntArray rtnArr(6);
    for( size_t i = 0 ; i < 6 ; i++ ){  rtnArr(i) = jntArr[i];  }
    return rtnArr;
}