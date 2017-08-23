
//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of sot-hpp
// sot-hpp is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// sot-hpp is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-hpp  If not, see
// <http://www.gnu.org/licenses/>.
#include <jrl/mal/matrixabstractlayer.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <hpp/model/joint.hh>
#include <hpp/model/fwd.hh>
#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <sot/hpp/config.hh>
#include "path-sampler.hh"
#include "command.hh"
#include <boost/assign/list_inserter.hpp> // for 'push_back()'
#include <boost/assign/list_of.hpp>

#include <math.h>


  
/*setTimeStep*/


namespace dynamicgraph {
  namespace sot {
    namespace hpp {

      using dynamicgraph::Entity;
      using ::hpp::model::Device;
      using ::hpp::model::JointPtr_t;
      using ::hpp::model::DevicePtr_t;

      using ::hpp::core::PathVector;
      using ::hpp::core::SteeringMethodStraight;
      using ::hpp::core::Problem;
      using ::hpp::core::SteeringMethodPtr_t;
      using ::hpp::model::ConfigurationIn_t;
      using ::hpp::model::ConfigurationOut_t;
      using ::hpp::model::Configuration_t;
      using ::hpp::model::size_type;

      // Convert configuration from hpp::model::Configuration_t to
      // dynamicgraph::Vector
      void convert (ConfigurationIn_t hppConfig, Vector& sotConfig)
      {

        for (size_type i=0; i<hppConfig.size (); ++i) {
          sotConfig (i) = hppConfig [i];
        }

      }

      // Convert configuration from hpp::model::Configuration_t to
      // dynamicgraph::Vector
      void convert (const Vector& sotConfig, ConfigurationOut_t& hppConfig)
      {
           
        for (size_type i=0; i<hppConfig.size (); ++i) {
          hppConfig [i] = sotConfig (i);
        }
      }

      void convert ( Vector& sotConfig,Vector currentstate)
      {
       Vector arm;
       arm.resize(14);
       //sotConfig.extract(6,12,larm);
       arm = sotConfig;
       //sotConfig.extract(0,6,arm);
       sotConfig.resize(20);
       sotConfig = currentstate;
       for (int i=6; i<20; i++) {
            sotConfig(i) = arm(i-6);
       }

      }
      
      void posetoMatrix( Vector& ref, MatrixHomogeneous& tm)
      {
       MAL_S4x4_MATRIX_SET_IDENTITY(tm);
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,0,3) = ref (0);
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,1,3) = ref (1);
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,2,3) = ref (2);

       MAL_S4x4_MATRIX_ACCESS_I_J(tm,0,0) = 1-2*pow(ref(5),2)-2*pow(ref(6),2); 
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,0,1) = 2*ref(4)*ref(5)-2*ref(6)*ref(3); 
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,0,2) = 2*ref(4)*ref(6)+2*ref(5)*ref(3);       

       MAL_S4x4_MATRIX_ACCESS_I_J(tm,1,0) = 2*ref(4)*ref(5)+2*ref(6)*ref(3); 
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,1,1) = 1-2*pow(ref (4),2)-2*pow(ref (6),2); 
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,1,2) = 2*ref(5)*ref(6)-2*ref(4)*ref(3);   

       MAL_S4x4_MATRIX_ACCESS_I_J(tm,2,0) = 2*ref(4)*ref(6)-2*ref(5)*ref(3);
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,2,1) = 2*ref(5)*ref(6)+2*ref(4)*ref(3);  
       MAL_S4x4_MATRIX_ACCESS_I_J(tm,2,2) = 1-2*pow(ref (4),2)-2*pow(ref (5),2);   

      }

      PathSampler::PathSampler (const std::string& name) :
	Entity (name), configurationSOUT
	("PathSampler("+name+")::output(vector)::configuration"),
  jointPositionSIN(NULL,"PathSampler("+name+")::input(vector)::position"),
	robot_ (),problem_ (), path_ (), steeringMethod_ (), timeStep_ (),
	lastWaypoint_ (), state_ (NOT_STARTED), startTime_ (), samplingTime_ (),rootJointType_("planar")
      {
	using command::makeCommandVoid0;
        using command::makeCommandVoid1;
	using command::makeDirectSetter;
        using command::makeDirectGetter;
	// Initialize input signal
  signalRegistration(jointPositionSIN);
	signalRegistration (configurationSOUT);
        referenceSignalName = "l_wrist_1_joint";
	configurationSOUT.setFunction
	  (boost::bind (&PathSampler::computeConfiguration, this, _1, _2));

	std::string docstring;
	// Add command AddWaypoint
	docstring =
	  "\n"
	  "    Add a way point to the path\n"
	  "      input (vector) the waypoint: should be of the same dimension as\n"
	  "                     robot configuration\n"
	  "\n"
	  "      Path is a concatenation of straight interpolation paths as "
	  "defined\n"
	  "      in hpp::core::PathVector.\n";
	addCommand (std::string ("addWaypoint"),
		    new AddWaypoint (*this, docstring));

	docstring =
	  "\n"
	  "    Start sampling path\n";
	addCommand ("start", makeCommandVoid0(*this, &PathSampler::start,
					     docstring));

	docstring =
	  "\n"
	  "    Reset path\n";
	addCommand ("resetPath", makeCommandVoid0 (*this,
						   &PathSampler::resetPath,
						   docstring));

	docstring =
	  "\n"
	  "    Set timeStep\n"
	  "      Input: float\n";
	addCommand ("setTimeStep",
		    makeDirectSetter (*this, &timeStep_, docstring));

	docstring =
	  "\n"
	  "    Set timeStep\n"
	  "      Input: float\n";
	addCommand ("getState",
		    makeDirectGetter (*this, &state_, docstring));


	docstring =
	  "\n"
	  "    Load urdf robot model\n"
	  "      Input: string packageName: name of the ros package containing\n"
	  "               the urdf file\n"
	  "             string rootJointType: type of the root joint among "
	  "['freeflyer',\n"
	  "               'planar', 'anchor']\n"
	  "             string modelName: name of the urdf file\n"
	  "\n"
	  "     The url of the file is 'package://${packageName}/urdf/"
	  "modelName.urdf'\n";
	addCommand ("loadRobotModel",
		    dynamicgraph::command::makeCommandVoid3
		    (*this, &PathSampler::loadRobotModel, docstring));

	docstring =
	  "\n"
	  "    Create reference frame for joints\n"
	  "      Input: string name of the signal\n"
	  "             string rootJointType: type of the root joint among "
	  "\n";
	addCommand ("createJointReference",
		    dynamicgraph::command::makeCommandVoid1
		    (*this, &PathSampler::createReferenceJointSignal, docstring));
      }


      void PathSampler::loadRobotModel (const std::string& packageName,
					const std::string& rootJointType,
					const std::string& modelName)
      {

	robot_ = Device::create ("modelName");
	::hpp::model::urdf::loadRobotModel (robot_, rootJointType, packageName,
					    modelName, "", "");
	// Create a new empty path
	path_ = PathVector::create (robot_->configSize (),robot_->configSize ());

	//steeringMethod_ = SteeringMethodStraight::create(problem_);

	steeringMethod_ = SteeringMethodStraight::create(robot_);

      }

      void PathSampler::createReferenceJointSignal (
					const std::string& jointName )
      {
          dynamicgraph::Signal< MatrixHomogeneous,int > * sig
            = new dynamicgraph::Signal< MatrixHomogeneous,int >
            ("PathSampler("+name+")::output(matrixHomo)::"+jointName);

          (*sig).setFunction(boost::bind(&PathSampler::computePosition,this,jointName,_1,_2));

          referenceSignalName = jointName;
              
          //genericSignalRefs.push_back( sig );
          signalRegistration( *sig );

      }

MatrixHomogeneous& PathSampler::computePosition( const std::string& jointName, MatrixHomogeneous& res,
         int time)
{
    /*JointPtr_t joint = robot_->getJointByName (jointName);
	  if (!joint) {
      throw std::runtime_error ("Robot has no joint with name ");
	  }*/
	  //T = joint->currentTransformation ();
    Vector pose_reference;
	  pose_reference.resize(7);
	  pose_reference (0) = T.getTranslation () [0];
	  pose_reference (1) = T.getTranslation () [1];
	  pose_reference (2) = T.getTranslation () [2];
	  pose_reference (4) = T.getQuatRotation ().getX();
	  pose_reference (5) = T.getQuatRotation ().getY();
	  pose_reference (6) = T.getQuatRotation ().getZ();
	  pose_reference (3) = T.getQuatRotation ().getW();
    posetoMatrix(pose_reference,res);
	  return res;

}

/*############################################################################################################

void Robot::cmd_createJointPositionSignals( const std::string& SignalName,
				 const std::string& jointName )
{
  CjrlJoint* joint = getJointByName(jointName);
  if (!joint) {
    throw runtime_error ("Robot has no joint corresponding to " + jointName);
  }
  createEndeffJacobianSignal(std::string("J")+opPointName, joint);
  createPositionSignal(opPointName, joint);
}


dg::SignalTimeDependent< MatrixHomogeneous,int >& Dynamic::
createPositionSignal( const std::string& signame, CjrlJoint* aJoint)
{
  sotDEBUGIN(15);

  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig
    = new dg::SignalTimeDependent< MatrixHomogeneous,int >
    ( boost::bind(&Dynamic::computeGenericPosition,this,aJoint,_1,_2),
      ComputeJointPosition,
      "sotDynamic("+name+")::output(matrixHomo)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}

#
Transform Robot::getJointPosition(const char* jointName)
{
	  JointPtr_t joint = robot->getJointByName (jointName);
	  if (!joint) {
      throw runtime_error ("Robot has no joint corresponding to " + jointName);
	  }
	  const Transform3f& T = joint->currentTransformation ();
	  double* res = new Transform;
	  res [0] = T.getTranslation () [0];
	  res [1] = T.getTranslation () [1];
	  res [2] = T.getTranslation () [2];
	  res [3] = T.getQuatRotation () [0];
	  res [4] = T.getQuatRotation () [1];
	  res [5] = T.getQuatRotation () [2];
	  res [6] = T.getQuatRotation () [3];
	  return res;

}

    MAL_S4x4_MATRIX_ACCESS_I_J(m4,i,j)

    [qx,qy,qz,qw]=q
    M= array([[1-2*qy**2-2*qz**2,2*qx*qy-2*qz*qw,2*qx*qz+2*qy*qw,0],
        [2*qx*qy+2*qz*qw,1-2*qx**2-2*qz**2,2*qy*qz-2*qx*qw,0],
        [2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx**2-2*qy**2,0]])

*/

      void PathSampler::addWaypoint (const Vector& wp)
      {
	if (!robot_) {
	  throw std::runtime_error ("Robot is not initialized");
	}
	if (robot_->configSize () != (size_type) wp.size ()) {
	  std::ostringstream oss ("                     \n");
	  oss << "Dimension of robot (" << robot_->configSize () <<
	    ") and dimension of waypoint (" << wp.size () << ") differ.";
	  throw std::runtime_error (oss.str ().c_str ());
	}
	Configuration_t waypoint (wp.size ());
	convert (wp, waypoint);
        //std::cout << waypoint;
	if (lastWaypoint_.size () != 0 ){
	  // It is not the first way point, add a straight path
	  path_->appendPath ((*steeringMethod_) (lastWaypoint_, waypoint));
	}
	lastWaypoint_ = waypoint;
	if (state_ == RESET) {
          state_ = NOT_STARTED;
        }

      }

      void PathSampler::start ()
      {
	// if already started or finished, do nothing
	if (state_ == NOT_STARTED) {
	  startTime_ = 0;
    samplingTime_ = 0;
    configuration_t.resize(path_->outputSize ());
    convert ((*path_) (timeStep_), configuration_t);

    //convert(configuration_t);

	  state_ = SAMPLING;
	}
      }

      void PathSampler::resetPath ()
      {
	if (robot_) {
	   path_ = PathVector::create (robot_->configSize (),robot_->configSize ());
         //   path.
	}
	// Keep last waypoint
        lastWaypoint_.resize(0);
	state_ = RESET;
  
      }


      Vector& PathSampler::computeConfiguration
      (Vector& configuration, const int& time)
      {
      

    /*JointPtr_t joint = robot_->getJointByName (referenceSignalName);
	  if (!joint) {
      throw std::runtime_error ("Robot has no joint with name ");
	  }*/
	  //T = joint->currentTransformation ();
	if (!path_) {
	  throw std::runtime_error
	    ("Path is not initialized in entity PathSampler (" +
	     getName () + ")");
	}
	if (path_->length () == 0) {
	  if (lastWaypoint_.size () != robot_->configSize ()) {
	    throw std::runtime_error
	      ("Path length is 0 in entity PathSampler (" +
	       getName () + ") and waypoint size does not fit robot size");
	  }
    configuration = jointPositionSIN(time);

	  return configuration;
	}
	configuration.resize (path_->outputSize ());
	if (state_ == NOT_STARTED) {
    configuration = jointPositionSIN(time);
    //robot_->currentConfiguration ((*path_)(0) );
    //robot_->computeForwardKinematics();
    //T = joint->currentTransformation ();
    return configuration;
	}
	else if (state_ == FINISHED) {
	  convert ((*path_) (path_->length ()), configuration);
    //robot_->currentConfiguration ((*path_)(path_->length ()) );
    //robot_->computeForwardKinematics();
    //T = joint->currentTransformation ();
	}
	else if (state_ == SAMPLING) {
	  double t = timeStep_ * (samplingTime_ - startTime_);
	  if (t > path_->length ()) {
	    t = path_->length ();
	    state_ = FINISHED;
	  }
    /*
    Vector currentstate, desiredstate,tmp;
    currentstate.resize(33);
    desiredstate.resize(33);
    tmp.resize(33);
    currentstate = jointPositionSIN(time);
    currentstate.extract(6,33,tmp);
    configuration_t.extract(6,33,desiredstate);

    if ((tmp - desiredstate).norm() < 0.1){*/
    if (1){
    //robot_->currentConfiguration ((*path_)(t) );
    //robot_->computeForwardKinematics();
    convert ((*path_) (t), configuration);
    samplingTime_  = samplingTime_ + 1;
    //T = joint->currentTransformation ();
    }
    else {
    convert ((*path_) (0), configuration);
    }
	  
    //robot_->currentConfiguration ((*path_) (t));
    //configuration = jointPositionSIN(time);
    //return configuration;
	} 
    /*if(rootJointType_ == "planar") {
      convert (configuration);
    }*/
    Vector currentstate;
    currentstate.resize(34);
    currentstate = jointPositionSIN(time);
    convert (configuration,currentstate);
    configuration_t = configuration;
    return configuration;
	  //robot_->computeForwardKinematics ();
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (PathSampler, "PathSampler");
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
