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

#ifndef XBOT_MODELINTERFACE_RBDL_H
#define XBOT_MODELINTERFACE_RBDL_H

#include <XBotInterface/ModelInterface.h>
#include <SharedLibraryClass.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


namespace XBot
{

class ModelInterfaceRBDL : public ModelInterface
{

public:
    
    virtual void getCOM(KDL::Vector& com_position) const;

    virtual void getCOMJacobian(KDL::Jacobian& J) const;

    virtual void getCOMVelocity(KDL::Vector& velocity) const;

    virtual void getGravity(KDL::Vector& gravity) const;

    virtual void getModelID(std::vector< std::string >& joint_name) const;

    virtual bool getPointJacobian(const std::string& link_name, const KDL::Vector& reference_point, KDL::Jacobian& J) const;

    virtual bool getPose(const std::string& source_frame, KDL::Frame& pose) const;

    virtual bool getSpatialAcceleration(const std::string& link_name, KDL::Twist& acceleration) const;

    virtual bool getSpatialVelocity(const std::string& link_name, KDL::Twist& velocity) const;

    virtual bool setFloatingBasePose(const KDL::Frame& floating_base_pose);

    virtual void setGravity(const KDL::Vector& gravity);

    virtual bool update(bool update_position = true, bool update_velocity = false, bool update_desired_acceleration = false);




protected:
    
    
    virtual bool init_model(const std::string& path_to_cfg) final;
    
private:
    
    mutable RigidBodyDynamics::Model _rbdl_model;
    
    Eigen::VectorXd _q, _qdot, _qddot, _tau;
    int _ndof;
    
    std::vector<std::string> _model_ordered_joint_names;
    
    mutable RigidBodyDynamics::Math::Vector3d _tmp_vector3d;
    mutable RigidBodyDynamics::Math::Matrix3d _tmp_matrix3d;
    mutable RigidBodyDynamics::Math::MatrixNd _tmp_jacobian3, _tmp_jacobian6;
    mutable KDL::Frame _tmp_kdl_frame;
    
    int linkId(const std::string& link_name) const;
    int jointModelId(const std::string& joint_name) const;
    
    

};


}
#endif