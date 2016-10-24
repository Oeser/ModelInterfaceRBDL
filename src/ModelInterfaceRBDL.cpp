/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <ModelInterfaceRBDL/ModelInterfaceRBDL.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(model_interface_rbdl, XBot::ModelInterfaceRBDL, XBot::ModelInterface);

bool XBot::ModelInterfaceRBDL::init_model(const std::string& path_to_cfg)
{
    std::cout << "Initializing RBDL model!" << std::endl;
    
    // Init rbdl model with urdf
    if(!RigidBodyDynamics::Addons::URDFReadFromString(getUrdfString().c_str(), &_rbdl_model, isFloatingBase(), true)){
        std::cout << "ERROR in " << __func__ << ": RBDL model could not be initilized from given URDF string!" << std::endl;
        return false;
    }
    
    // Init configuration vectors
    _ndof = _rbdl_model.dof_count;
    _q.setZero(_ndof);
    _qdot.setZero(_ndof);
    _qddot.setZero(_ndof);
    _tau.setZero(_ndof);
    
    // Fill model-ordered joint id vector
    _model_ordered_joint_names.resize(_ndof);
    int joint_idx = 0;
    
    if( isFloatingBase() ){
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_1"; 
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_2"; 
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_3"; 
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_4"; 
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_5"; 
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_6"; 
    }
    
    for(int i = 0 ; i < _ndof; i++){
        
        
        std::string joint_name = getUrdf().getLink(_rbdl_model.GetBodyName(i+1))->parent_joint->name;
        int joint_model_id = _rbdl_model.mJoints[i+1].q_index;
        
        std::cout << "Joint name: " << joint_name << " RBDL ID: " << joint_model_id << std::endl;
        
        _model_ordered_joint_names[joint_model_id] = joint_name;
        

    }
    
    return true;
}

void XBot::ModelInterfaceRBDL::getCOM(KDL::Vector& com_position) const
{
    double mass;
    RigidBodyDynamics::Utils::CalcCenterOfMass(_rbdl_model, _q, _qdot, mass, _tmp_vector3d, nullptr, nullptr, false);
    tf::vectorEigenToKDL(_tmp_vector3d, com_position);
}

void XBot::ModelInterfaceRBDL::getCOMJacobian(KDL::Jacobian& J) const
{
    _tmp_jacobian6.setZero(6, _rbdl_model.dof_count);
    double mass;
    int body_id = 0;
    for( const RigidBodyDynamics::Body& body : _rbdl_model.mBodies ){
        _tmp_jacobian3.setZero(3, _rbdl_model.dof_count);
        RigidBodyDynamics::CalcPointJacobian(_rbdl_model, _q, body_id, body.mCenterOfMass, _tmp_jacobian3, false);
        _tmp_jacobian6.block(0,0,3,_rbdl_model.dof_count) += body.mMass * _tmp_jacobian3;
        mass += body.mMass;
    }
    _tmp_jacobian6 /= mass;
    J.data = _tmp_jacobian6;
    
}

bool XBot::ModelInterfaceRBDL::getPose(const std::string& source_frame, KDL::Frame& pose) const
{
    _tmp_vector3d.setZero();
    
    _tmp_matrix3d = RigidBodyDynamics::CalcBodyWorldOrientation(_rbdl_model, 
                                                                _q, 
                                                                linkId(source_frame), 
                                                                false);
    
    _tmp_vector3d = RigidBodyDynamics::CalcBodyToBaseCoordinates(_rbdl_model, 
                                                                 _q, 
                                                                 linkId(source_frame), 
                                                                 _tmp_vector3d, 
                                                                 false);
    
    rotationEigenToKDL(_tmp_matrix3d.transpose(), pose.M);
    tf::vectorEigenToKDL(_tmp_vector3d, pose.p);
    
    return true;
    
    // TBD check for link existence
}

int XBot::ModelInterfaceRBDL::linkId(const std::string& link_name) const
{
    return _rbdl_model.GetBodyId(link_name.c_str());
}

bool XBot::ModelInterfaceRBDL::update(bool update_position, bool update_velocity, bool update_desired_acceleration)
{
    bool success = true;
    success = success && getJointPosition(_q);
    success = success && getJointVelocity(_qdot);
    success = success && getJointEffort(_tau);
    // TBD what to do with acceleration??????
    RigidBodyDynamics::UpdateKinematics(_rbdl_model, _q, _qdot, _qddot);
    return success;
}

bool XBot::ModelInterfaceRBDL::getPointJacobian(const std::string& link_name, 
                                                const KDL::Vector& reference_point, 
                                                KDL::Jacobian& J) const
{
    _tmp_jacobian6.setZero(6, _rbdl_model.dof_count);
    tf::vectorKDLToEigen(reference_point, _tmp_vector3d);
    RigidBodyDynamics::CalcPointJacobian6D(_rbdl_model, _q, linkId(link_name), _tmp_vector3d, _tmp_jacobian6, false);
    J.data = _tmp_jacobian6;
}

void XBot::ModelInterfaceRBDL::getModelID(std::vector< std::string >& joint_name) const
{
    joint_name = _model_ordered_joint_names;
}

int XBot::ModelInterfaceRBDL::jointModelId(const std::string& joint_name) const
{
    return _rbdl_model.mJoints[linkId(this->getUrdf().getJoint(joint_name)->child_link_name)].q_index;  
}


bool XBot::ModelInterfaceRBDL::setFloatingBasePose(const KDL::Frame& floating_base_pose)
{
    return false;
}

void XBot::ModelInterfaceRBDL::getCOMVelocity(KDL::Vector& velocity) const
{
    double mass;
    RigidBodyDynamics::Utils::CalcCenterOfMass(_rbdl_model, _q, _qdot, mass, _tmp_vector3d, &_tmp_vector3d_1, nullptr, false);
    tf::vectorEigenToKDL(_tmp_vector3d_1, velocity);
}

void XBot::ModelInterfaceRBDL::getGravity(KDL::Vector& gravity) const
{
    tf::vectorEigenToKDL(_rbdl_model.gravity, gravity);
}

bool XBot::ModelInterfaceRBDL::getSpatialAcceleration(const std::string& link_name, KDL::Twist& acceleration) const
{
    return false;
}

bool XBot::ModelInterfaceRBDL::getSpatialVelocity(const std::string& link_name, KDL::Twist& velocity) const
{
    _tmp_vector3d.setZero();
    tf::twistEigenToKDL(RigidBodyDynamics::CalcPointVelocity6D(_rbdl_model, _q, _qdot, linkId(link_name), _tmp_vector3d), velocity);
    return true; // TBD check link exists
}


void XBot::ModelInterfaceRBDL::setGravity(const KDL::Vector& gravity)
{
    tf::vectorKDLToEigen(gravity, _rbdl_model.gravity);
}



