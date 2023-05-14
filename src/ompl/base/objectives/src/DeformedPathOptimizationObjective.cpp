/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Zhaoting Li*/

#include "ompl/base/objectives/DeformedPathOptimizationObjective.h"


ompl::base::DeformedPathOptimizationObjective::DeformedPathOptimizationObjective(const SpaceInformationPtr &si,
                                                                                     double pathLengthWeight)
  : OptimizationObjective(si), pathLengthWeight_(pathLengthWeight)
{
    description_ = "Deformed path length";

    // contact_detector_simplified.read_mesh_data();
    // contact_detector_simplified.create_vertex_neighbor_info();

    contact_detector_simplified.read_link_cluster_vertices_infos();

    Eigen::Vector3d pin1, pin2;
    pin1 << 0.5, -0.3, 0.425;
    pin2 << 0.5, 0.3, 0.425;
    // pin1 << 0.5, -0.3, 0.375;
    // pin2 << 0.5, 0.3, 0.375;

    contact_detector_simplified.set_elastic_band_pins(pin1, pin2);
}

void ompl::base::DeformedPathOptimizationObjective::update_planner_info(const double pin1_x, const double pin1_z)
{
    Eigen::Vector3d pin1, pin2;
    pin1 << pin1_x, -0.3, pin1_z;
    pin2 << pin1_x, 0.3, pin1_z;
    contact_detector_simplified.set_elastic_band_pins(pin1, pin2);
}

double ompl::base::DeformedPathOptimizationObjective::getPathLengthWeight() const
{
    return pathLengthWeight_;
}

ompl::base::Cost ompl::base::DeformedPathOptimizationObjective::stateCost(const State *) const
{   

    // std::cout<<"test DeformedPathOptimizationObjective stateCost" << std::endl;

    return Cost(1.0);
}


ompl::base::Cost ompl::base::DeformedPathOptimizationObjective::StateCost_deformedpath(const State *s) 
{   
    // std::cout << "stateCost: "<< Cost(1.0).value() << std::endl;

    
    // contact::Contact_detection_simplifiedModel contact_detector_simplified = contact::Contact_detection_simplifiedModel(mesh_path, robot_flag);
    // // contact_detector_simplified.init();
    // contact_detector_simplified.read_mesh_data();
    // contact_detector_simplified.create_vertex_neighbor_info();
    // std::cout<<"test StateCost_deformedpath" << std::endl;
    std::vector<double> state_vector;
    ompl::base::StateSpacePtr space = si_->getStateSpace();
    space->copyToReals(state_vector, s);

    Eigen::VectorXd robot_state = Eigen::Map<Eigen::VectorXd>(state_vector.data(), state_vector.size());

    Eigen::VectorXd robot_state_13(13);
    robot_state_13.setZero();
    robot_state_13.segment(6, 7) = robot_state;

    std::vector<Eigen::Vector3d> path_vertices;
    std::vector<int> path_vertices_id;
    contact_detector_simplified.find_contact_location_by_mode(robot_state_13, 1, path_vertices, path_vertices_id);   // 1 is the case where robot is below the band
    // contact_detector_simplified.find_contact_location_by_mode(robot_state_13, 0, path_vertices, path_vertices_id);  // 0 is the case where robot is above the band

    double cost_deformed = contact_detector_simplified.calculate_path_cost(path_vertices);


    std::cout<<"test DeformedPathOptimizationObjective: "<< cost_deformed << std::endl;
    // std::cout<<"sampled robot state: " << robot_state_13.transpose() << std::endl; 

    // return Cost(0.5 + std::exp(10 * cost_deformed));
    return Cost(0.1 + (100 * cost_deformed));
}

ompl::base::Cost ompl::base::DeformedPathOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    // Only accrue positive changes in cost
    double positiveCostAccrued = std::max(stateCost(s2).value() - stateCost(s1).value(), 0.0);
    // double positiveCostAccrued = std::max(StateCost_deformedpath(s2).value() - StateCost_deformedpath(s1).value(), 0.0);
    // std::cout << "motionCost: "<< Cost(positiveCostAccrued + pathLengthWeight_ * si_->distance(s1, s2)).value() << std::endl;
    return Cost(positiveCostAccrued + pathLengthWeight_ * si_->distance(s1, s2));
}
