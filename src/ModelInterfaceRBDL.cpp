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

bool XBot::ModelInterfaceRBDL::getPose(const std::string& source_frame, 
                                       const std::string& target_frame, 
                                       KDL::Frame& pose) const
{
    return false;
}
