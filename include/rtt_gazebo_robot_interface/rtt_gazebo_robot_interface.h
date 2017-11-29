// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

#pragma once

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

// Gazebo headers
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// Eigen
#include <Eigen/Core>

class RttGazeboRobotInterface : public RTT::TaskContext
{
public:
    RttGazeboRobotInterface(const std::string& name);
    bool configureHook();
    void updateHook();
    void worldUpdateBegin();
    void worldUpdateEnd();
    int getNrOfDegreesOfFreedom();
    Eigen::Vector3d getGravity();
    void printState();
    bool setModelConfiguration(std::vector<std::string> joint_names,std::vector<double> joint_positions);
protected:
    // RTT interface
    RTT::InputPort<Eigen::VectorXd>   port_jnt_trq_in_;
    RTT::OutputPort<Eigen::VectorXd>  port_jnt_pos_out_;
    RTT::OutputPort<Eigen::VectorXd>  port_jnt_vel_out_;
    RTT::OutputPort<Eigen::VectorXd>  port_jnt_trq_out_;
    RTT::OutputPort<Eigen::Vector3d>  port_gravity_out_;
    
    // Gazebo 
    std::vector<std::string> joint_map_;
    gazebo::physics::ModelPtr gazebo_model_;
    gazebo::event::ConnectionPtr gazebo_world_end_connection_;
    gazebo::event::ConnectionPtr gazebo_world_begin_connection_;

    // Current state
    Eigen::VectorXd current_jnt_pos_;
    Eigen::VectorXd current_jnt_vel_;
    Eigen::VectorXd current_jnt_trq_;
    // Command
    Eigen::VectorXd jnt_trq_command_;
    
    Eigen::Vector3d gravity_;
    std::string model_name_;
};

