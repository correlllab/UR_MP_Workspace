#ifndef HELPERS_H
#define HELPERS_H

/** Standard **/
#include <fstream>
#include <exception>
#include <signal.h>
#include <string>
#include <iostream>
using std::ifstream;
using std::ostream;
using std::string;
using std::cout;
using std::endl;

/** FK/IK/MP **/
#include <boost/array.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>


/******************** Utilities ********************/

double randrange( double rMin , double rMax ); // Get a pseudo-random number between `rMin` and `rMax`

string yesno( bool condition ); // Return "YES" if true, otherwise return "NO"

template<typename T, std::size_t N> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const boost::array<T,N>& vec ){ // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    size_t len = vec.size();
    os << "[ ";
    for (size_t i = 0; i < len; i++) {
        os << vec[i];
        if (i + 1 < len) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}


/******************** Conversion ********************/

boost::array<double,16> KDL_frame_to_response_arr( const KDL::Frame& pose ); // Translate a KDL pose to a flattened pose



/******************** Kinematics ********************/

KDL::JntArray load_q( const boost::array<double,6>& q_input ); // Convert a Boost array to KDL Joint Array

void fuzz_seed_array( KDL::JntArray& seedArr , double fuzz_rad ); // Perturbate each element of `seedArr` by +/- `fuzz_rad`

// IK_soln_to_IK_arr: Pack the IK solution and its validity into a single boost array
boost::array<double,7> IK_soln_to_IK_arr( const KDL::JntArray& jntArr , int valid ); 

KDL::Frame request_arr_to_KDL_frame( const boost::array<double,16>& pose ); // Translate a flattened pose to a KDL pose

KDL::JntArray request_arr_to_KDL_arr( const boost::array<double,6>& jntArr );

#endif