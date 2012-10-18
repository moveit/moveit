/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <vector>

#include <moveit_visualization_ros/kinematics_group_visualization.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <collision_detection/collision_tools.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <random_numbers/random_numbers.h>

namespace moveit_visualization_ros
{

    static const ros::Duration NO_SOLUTION_UPDATE_TIMEOUT = ros::Duration(.25);

    KinematicsGroupVisualization::KinematicsGroupVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                               boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                                               boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                                                               const std::string& group_name,
                                                               const std::string& suffix_name,
                                                               const std_msgs::ColorRGBA& good_color,
                                                               const std_msgs::ColorRGBA& bad_color,
                                                               ros::Publisher& marker_publisher,
                                                               boost::shared_ptr<tf::TransformBroadcaster>& broadcaster) :
        group_name_(group_name),
        suffix_name_(suffix_name),
        regular_marker_name_(group_name+"_kinematics_"+suffix_name),
        use_good_bad_(false),
        regular_markers_hidden_(false),
        all_markers_hidden_(false),
        last_solution_good_(true),
        last_solution_changed_(false),
        good_color_(good_color),
        bad_color_(bad_color),
        stored_alpha_(good_color.a),
        planning_scene_(planning_scene),
        interactive_marker_server_(interactive_marker_server),
        state_(planning_scene_->getCurrentState()),
        marker_publisher_(marker_publisher),
        tf_broadcaster_(broadcaster),
        dof_marker_enabled_(true),
        interaction_enabled_(true),
        visible_(true)
    {
        const std::map<std::string, srdf::Model::Group>& group_map
            = planning_scene_->getKinematicModel()->getJointModelGroupConfigMap();

        if(group_map.find(group_name) == group_map.end()) {
            ROS_ERROR_STREAM("No group named " << group_name);
            return;
        }

        bool subgroups = false;
        const srdf::Model::Group& srdf_group = group_map.find(group_name)->second;
        if(srdf_group.subgroups_.size() > 0) {
            subgroups = true;
        } else if(srdf_group.chains_.size() == 0 ||
                  srdf_group.chains_[0].first.empty() ||
                  srdf_group.chains_[0].second.empty()) {
            ROS_ERROR_STREAM("Group name " << group_name << " has no or messed up chain definition");
            return;
        }

        planning_models::KinematicModel::SolverAllocatorFn kinematics_allocator = kinematic_model_loader->getKinematicsPluginLoader()->getLoaderFunction(kinematic_model_loader->getSRDF());

        if(subgroups) {
            std::map<std::string, kinematics::KinematicsBasePtr> solver_map;
            for(unsigned int i = 0; i < srdf_group.subgroups_.size(); i++) {
                const srdf::Model::Group& subgroup = group_map.at(srdf_group.subgroups_[i]);
                //TODO - deal with multi-chains etc
                if(subgroup.chains_.size() > 0)
                {
                    subgroup_chain_names_.push_back(subgroup.name_);
                }
                else
                {
                    ROS_WARN_STREAM("This visualizer does not support non-kinematic chain groups such as planning group: " << subgroup.name_);
                }
                if(!kinematic_model_loader->getKinematicsPluginLoader()->isGroupKnown(subgroup.name_)) {
                    ROS_WARN_STREAM("No loader for group " << subgroup.name_);
                    continue;
                }
                const planning_models::KinematicModel::JointModelGroup* jmg = planning_scene_->getKinematicModel()->getJointModelGroup(subgroup.name_);
                kinematics::KinematicsBasePtr slv = kinematics_allocator(jmg);
                if (slv)
                    solver_map[subgroup.name_] = slv;
                group_to_interactive_marker_names_[srdf_group.subgroups_[i]] = makeInteractiveMarkerName(srdf_group.subgroups_[i]);
                interactive_marker_to_group_names_[makeInteractiveMarkerName(srdf_group.subgroups_[i])] = srdf_group.subgroups_[i];
            }
            ik_solver_.reset(new kinematics_constraint_aware::KinematicsSolverConstraintAware(solver_map,
                                                                                              planning_scene_->getKinematicModel(),
                                                                                              group_name));
            std::map<std::string, geometry_msgs::Pose> poses;

            for(unsigned int i = 0; i < subgroup_chain_names_.size(); i++) {
                //TODO - deal with non-chain groups
                std::map<std::string, kinematics::KinematicsBasePtr>::iterator sit = solver_map.find(subgroup_chain_names_[i]);
                if (sit == solver_map.end())
                    continue;
                kinematics::KinematicsBasePtr& solver = sit->second;
                const planning_models::KinematicModel::LinkModel* lm = state_.getKinematicModel()->getLinkModel(solver->getTipFrame());
                if(lm == NULL) {
                    ROS_ERROR_STREAM("No link for tip frame " << solver->getTipFrame());
                }
                enable6DOFControls(false);
                for(std::map<std::string, std::string>::iterator it = group_to_interactive_marker_names_.begin();
                    it != group_to_interactive_marker_names_.end();
                    it++) {
                    default_menu_handler_.apply(*interactive_marker_server_, it->second);
                }
                interactive_marker_server_->applyChanges();

                geometry_msgs::Pose cur_pose;
                planning_models::msgFromPose(state_.getLinkState(solver->getTipFrame())->getGlobalLinkTransform(), cur_pose);
                poses[solver->getGroupName()] = cur_pose;
                ROS_DEBUG_STREAM("Adding pose for " << subgroup_chain_names_[i] << " "
                                 << cur_pose.position.x << " "
                                 << cur_pose.position.y << " "
                                 << cur_pose.position.z);
            }
            last_poses_ = poses;
            if (!poses.empty())
                updateEndEffectorState(poses.begin()->first, poses.begin()->second);
        } else {
            if(!kinematic_model_loader->getKinematicsPluginLoader()->isGroupKnown(group_name)) {
                ROS_WARN_STREAM("No loader for group " << group_name);
                return;
            }
            const planning_models::KinematicModel::JointModelGroup* jmg = planning_scene_->getKinematicModel()->getJointModelGroup(group_name);
            kinematics::KinematicsBasePtr result = kinematics_allocator(jmg);
            if (!result)
                return;

            ik_solver_.reset(new kinematics_constraint_aware::KinematicsSolverConstraintAware(result,
                                                                                              planning_scene_->getKinematicModel(),
                                                                                              group_name));

            group_to_interactive_marker_names_[group_name] = makeInteractiveMarkerName();
            interactive_marker_to_group_names_[makeInteractiveMarkerName()] = group_name;

            const planning_models::KinematicModel::LinkModel* lm = state_.getKinematicModel()->getLinkModel(result->getTipFrame());
            if(lm == NULL) {
                ROS_ERROR_STREAM("No link for tip frame " << result->getTipFrame());
            }
            enable6DOFControls(false);
            default_menu_handler_.apply(*interactive_marker_server_, group_to_interactive_marker_names_[group_name]);
            interactive_marker_server_->applyChanges();

            geometry_msgs::Pose cur_pose;
            planning_models::msgFromPose(state_.getLinkState(result->getTipFrame())->getGlobalLinkTransform(), cur_pose);
            updateEndEffectorState(group_name, cur_pose);
        }
    };

    void KinematicsGroupVisualization::hideRegularMarkers(){
        regular_markers_hidden_ = true;
        removeLastMarkers();
    }

    void KinematicsGroupVisualization::showRegularMarkers() {
        //if(!visible_) return;

        regular_markers_hidden_ = false;
        sendCurrentMarkers();
    }

    void KinematicsGroupVisualization::hideAllMarkers() {
        all_markers_hidden_ = true;
        for(std::map<std::string, std::string>::iterator it = group_to_interactive_marker_names_.begin();
            it != group_to_interactive_marker_names_.end();
            it++) {
            interactive_marker_server_->get(it->second, saved_markers_[it->second]);
        }
        removeLastMarkers();
        disable6DOFControls();
        for(std::map<std::string, std::string>::iterator it = group_to_interactive_marker_names_.begin();
            it != group_to_interactive_marker_names_.end();
            it++) {
            interactive_marker_server_->erase(it->second);
        }
        interactive_marker_server_->applyChanges();
    }

    void KinematicsGroupVisualization::showAllMarkers() {
        //if(!visible_) return;

        all_markers_hidden_ = false;
        for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
            last_marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
            last_marker_array_.markers[i].color.a = stored_alpha_;
        }
        enable6DOFControls();
        updateEndEffectorInteractiveMarker();
        for(std::map<std::string, geometry_msgs::Pose>::iterator it = last_poses_.begin();
            it != last_poses_.end();
            it++) {
            updateEndEffectorState(it->first, it->second);
        }
    }

    void KinematicsGroupVisualization::setMarkerAlpha(double a) {
        if(!regular_markers_hidden_) {
            for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
                last_marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
                last_marker_array_.markers[i].header.stamp = ros::Time::now();
                last_marker_array_.markers[i].color.a = a;
            }
            marker_publisher_.publish(last_marker_array_);
        }
        stored_alpha_ = a;
        std_msgs::ColorRGBA color;
        if(use_good_bad_) {
            if(last_solution_good_) {
                color = good_color_;
            } else {
                color = bad_color_;
            }
        } else {
            color = good_color_;
        }
        color.a = stored_alpha_;
        makeInteractiveControlMarkers(color,
                                      dof_marker_enabled_);
    }

    void KinematicsGroupVisualization::disable6DOFControls(bool load_saved) {
        for(std::map<std::string, std::string>::iterator it = group_to_interactive_marker_names_.begin();
            it != group_to_interactive_marker_names_.end();
            it++) {
            interactive_marker_server_->get(it->second, saved_markers_[it->second]);
        }
        dof_marker_enabled_ = false;
        std_msgs::ColorRGBA good_color = good_color_;
        good_color.a = stored_alpha_;
        makeInteractiveControlMarkers(good_color,
                                      false,
                                      load_saved);
    }

    void KinematicsGroupVisualization::enable6DOFControls(bool load_saved) {

        for(std::map<std::string, std::string>::iterator it = group_to_interactive_marker_names_.begin();
            it != group_to_interactive_marker_names_.end();
            it++) {
            interactive_marker_server_->get(it->second, saved_markers_[it->second]);
        }
        dof_marker_enabled_ = true;
        std_msgs::ColorRGBA color;
        if(use_good_bad_) {
            if(last_solution_good_) {
                color = good_color_;
            } else {
                color = bad_color_;
            }
        } else {
            color = good_color_;
        }
        color.a = stored_alpha_;
        makeInteractiveControlMarkers(color,
                                      true,
                                      load_saved);
    }

    void KinematicsGroupVisualization::addButtonClickCallback(const boost::function<void(void)>& button_click_callback) {
        button_click_callback_ = button_click_callback;
    }

    void KinematicsGroupVisualization::addMenuEntry(const std::string& name,
                                                    const boost::function<void(const std::string& name)>& callback) {
        interactive_markers::MenuHandler::EntryHandle eh
            = default_menu_handler_.insert(name,
                                           boost::bind(&KinematicsGroupVisualization::processInteractiveMenuFeedback, this, _1));

        menu_handle_to_string_map_[eh] = name;
        default_callback_map_[name] = callback;
        default_menu_handler_.reApply(*interactive_marker_server_);
        interactive_marker_server_->applyChanges();
    }

    void KinematicsGroupVisualization::updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene) {
        planning_scene_ = planning_scene;
        std::map<std::string, double> state_vals;
        state_.getStateValues(state_vals);
        state_ = planning_scene_->getCurrentState();
        state_.setStateValues(state_vals);
        if(!all_markers_hidden_ && !last_poses_.empty()) {
            updateEndEffectorState(last_poses_.begin()->first, last_poses_.begin()->second);
        }
    }

    /** Set a random valid state for this group.
     * Returns: true if it succeeded (in which case the planning scene is
     * updated and new markers are published), false otherwise. */
    bool KinematicsGroupVisualization::setRandomState(unsigned int max_tries) {

        random_numbers::RandomNumberGenerator rng;

        const planning_models::KinematicModel::JointModelGroup* jmg;
        jmg = planning_scene_->getKinematicModel()->getJointModelGroup(group_name_);
        if(!jmg)
        {
            ROS_ERROR_STREAM("No group named " << group_name_);
            return false;
        }
        std::vector<const planning_models::KinematicModel::JointModel*> joint_models = jmg->getJointModels();
        std::vector<std::string> joint_model_names = jmg->getJointModelNames();
        ROS_ASSERT(joint_models.size() == joint_model_names.size());
        std::vector<double> one_joint_values;
        std::vector<double> all_joint_values;
        planning_models::KinematicState ks(state_);

        planning_models::KinematicState::JointStateGroup* jsg = ks.getJointStateGroup(group_name_);
        for(unsigned int j=0;j<max_tries;j++)
        {
            all_joint_values.clear();
            for(unsigned int i=0; i<joint_models.size(); i++)
            {
                one_joint_values.clear();
                joint_models[i]->getRandomValues(rng, one_joint_values);
                ROS_ASSERT(one_joint_values.size() == 1);
                all_joint_values.push_back(one_joint_values[0]);
            }
            jsg->setStateValues(all_joint_values);
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.group_name = group_name_;
            planning_scene_->checkCollision(req, res, ks);
            if(!res.collision)
            {
                // Update the state
                planning_models::KinematicState::JointStateGroup* real_jsg = state_.getJointStateGroup(group_name_);
                real_jsg->setStateValues(all_joint_values);

                state_.updateLinkTransforms();

                sendCurrentMarkers();
                updateEndEffectorInteractiveMarker();
                return true;
            }
        }
        return false;
    }

    /** Set state for this group to be equal to the state in the planning scene. */
    void KinematicsGroupVisualization::resetState(void) {
        state_ = planning_scene_->getCurrentState();
        sendCurrentMarkers();
        updateEndEffectorInteractiveMarker();
    }

    void KinematicsGroupVisualization::setState(const planning_models::KinematicState& state) {
        state_ = state;
        sendCurrentMarkers();
        updateEndEffectorInteractiveMarker();
    }

    void KinematicsGroupVisualization::updateEndEffectorInteractiveMarker(void) {
        // Compute the new pose of the end-effector and force the associated
        // interactive marker there.
        if (!ik_solver_)
            return;
        const std::map<std::string, std::string>& tip_frame_map = ik_solver_->getTipFrames();
        for(std::map<std::string, std::string>::const_iterator it = tip_frame_map.begin();
            it != tip_frame_map.end();
            it++) {
            Eigen::Affine3d new_pose = state_.getLinkState(it->second)->getGlobalLinkTransform();
            geometry_msgs::Pose pm;
            planning_models::msgFromPose(new_pose, pm);
            last_poses_[it->first] = pm;
            new_pose = new_pose*relative_transforms_[it->first].inverse();
            geometry_msgs::Pose trans_pose;
            planning_models::msgFromPose(new_pose, trans_pose);
            interactive_marker_server_->setPose(group_to_interactive_marker_names_[it->first], trans_pose);
        }
        interactive_marker_server_->applyChanges();
    }

    void KinematicsGroupVisualization::processInteractiveMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        ROS_DEBUG_STREAM("got menu feedback");
        if(feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
            ROS_WARN_STREAM("Got something other than menu select on menu feedback function");
            return;
        }
        if(menu_handle_to_string_map_.find(feedback->menu_entry_id) ==
           menu_handle_to_string_map_.end()) {
            ROS_WARN_STREAM("No handle found " << feedback->menu_entry_id);
            return;
        }
        std::string name = menu_handle_to_string_map_[feedback->menu_entry_id];
        if(default_callback_map_.find(name) == default_callback_map_.end()) {
            ROS_WARN_STREAM("No callback associated with name " << name);
            return;
        }
        default_callback_map_[name](group_name_);
    }

    void KinematicsGroupVisualization::removeLastMarkers()
    {
        if(last_marker_array_.markers.empty()) return;

        for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
            last_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        marker_publisher_.publish(last_marker_array_);
        last_marker_array_.markers.clear();
    }

    void KinematicsGroupVisualization::sendCurrentMarkers()
    {
        if(!visible_) return;

        if(use_good_bad_ && last_solution_changed_) {
            enable6DOFControls(true);
        }
        if(last_solution_good_) {
            last_marker_array_.markers.clear();
            std_msgs::ColorRGBA col = good_color_;
            col.a = stored_alpha_;
            const std::vector<std::string>& link_names = ik_solver_->getLinkNames();
            state_.getRobotMarkers(col,
                                   regular_marker_name_,
                                   ros::Duration(0.0),
                                   last_marker_array_,
                                   link_names);
            if(all_markers_hidden_ || regular_markers_hidden_) return;
            marker_publisher_.publish(last_marker_array_);

            // aleeper: New stuff here
            std::vector<geometry_msgs::TransformStamped> transforms;
            for(size_t i = 0; i < last_marker_array_.markers.size(); i++)
            {
              visualization_msgs::Marker marker = last_marker_array_.markers[i];
              geometry_msgs::TransformStamped t;
              t.transform.translation.x = marker.pose.position.x;
              t.transform.translation.y = marker.pose.position.y;
              t.transform.translation.z = marker.pose.position.z;
              t.transform.rotation = marker.pose.orientation;
              t.header = marker.header;
              t.child_frame_id = group_name_ + "_" + suffix_name_ + "_" + link_names[i];
              transforms.push_back(t);
            }
            tf_broadcaster_->sendTransform(transforms);
        } else {
            removeLastMarkers();
            //last_marker_array_.markers.clear();
            // collision_detection::getCollisionMarkersFromContacts(last_marker_array_,
            //                                                      planning_scene_->getKinematicModel()->getModelFrame(),
            //                                                      ik_solver_->getLastInitialPoseCheckCollisionResult().contacts,
            //                                                      bad_color_,
            //                                                      ros::Duration(0.0));
            //marker_publisher_.publish(last_marker_array_);
        }
    }

    bool KinematicsGroupVisualization::validateEndEffectorState(const std::map<std::string, geometry_msgs::Pose>& poses,
                                                                sensor_msgs::JointState& sol,
                                                                moveit_msgs::MoveItErrorCodes& err)
    {
        ros::WallTime start = ros::WallTime::now();

        std::map<std::string, unsigned int> redundancies;

        const std::map<std::string, std::string>& tip_frame_map = ik_solver_->getTipFrames();
        const std::map<std::string, std::string>& base_frame_map = ik_solver_->getBaseFrames();
        std::map<std::string, geometry_msgs::Pose> conv_poses;
        for(std::map<std::string, std::string>::const_iterator it = tip_frame_map.begin();
            it != tip_frame_map.end();
            it++) {
            if(poses.find(it->first) == poses.end()) {
                ROS_WARN_STREAM("Must provide pose for every group, no pose for subgroup " << it->first);
                return false;
            }
            //assuming pose is in world frame for now
            Eigen::Affine3d cur;
            planning_models::poseFromMsg(poses.at(it->first), cur);

            state_.updateStateWithLinkAt(it->second, cur);

            ROS_DEBUG_STREAM("Cur pose of " << it->second << " in planning frame " << poses.at(it->first));

            //now need to get in base_frame
            Eigen::Affine3d base_in_world = state_.getLinkState(base_frame_map.at(it->first))->getGlobalLinkTransform();

            ROS_DEBUG_STREAM("Base " << base_frame_map.at(it->first) << " x y z " << base_in_world.translation().x() << " "
                             << base_in_world.translation().y() << " "
                             << base_in_world.translation().z() << " "
                             << Eigen::Quaterniond(base_in_world.rotation()).w());

            Eigen::Affine3d tip_in_base = base_in_world.inverse()*cur;

            ROS_DEBUG_STREAM("Tip " << tip_frame_map.at(it->first) << " x y z " << tip_in_base.translation().x() << " "
                             << tip_in_base.translation().y() << " "
                             << tip_in_base.translation().z());
            Eigen::Quaterniond rot_quat(tip_in_base.rotation());
            ROS_DEBUG_STREAM("Tip " << tip_frame_map.at(it->first) << " x y z w " << rot_quat.x() << " "
                             << rot_quat.y() << " "
                             << rot_quat.z() << " "
                             << rot_quat.w());

            geometry_msgs::Pose np;
            planning_models::msgFromPose(tip_in_base, np);

            conv_poses[it->first] = np;
            redundancies[it->first] = 2;
        }
        moveit_msgs::Constraints emp_constraints;
        bool result = true;
        if(conv_poses.size() == 1) {
            result = ik_solver_->findConstraintAwareSolution(conv_poses.begin()->second,
                                                             emp_constraints,
                                                             &state_,
                                                             planning_scene_,
                                                             sol,
                                                             err,
                                                             true);
        } else {
            result = ik_solver_->findConstraintAwareSolution(conv_poses,
                                                             redundancies,
                                                             emp_constraints,
                                                             &state_,
                                                             planning_scene_,
                                                             sol,
                                                             err,
                                                             true);
        }
        ROS_DEBUG_STREAM("Total time is " << (ros::WallTime::now()-start) << " result " << result);
        return result;
    }

    void KinematicsGroupVisualization::updateEndEffectorState(const std::string& group_name, const geometry_msgs::Pose& pose)
    {
        last_poses_[group_name] = pose;

        sensor_msgs::JointState sol;
        moveit_msgs::MoveItErrorCodes err;

        if(!last_solution_good_ && (ros::Time::now()-last_bad_validation_time_) < NO_SOLUTION_UPDATE_TIMEOUT) {
            ROS_DEBUG_STREAM("No update due to last bad validation");
            return;
        }

        if(validateEndEffectorState(last_poses_, sol, err)) {
            if(last_solution_good_ == false) {
                last_solution_changed_ = true;
            } else {
                last_solution_changed_ = false;
            }
            last_solution_good_ = true;
            state_.setStateValues(sol);
            if(state_changed_callback_) {
                state_changed_callback_(group_name_, state_);
            }
        } else {
            last_bad_validation_time_ = ros::Time::now();
            if(last_solution_good_ == true) {
                last_solution_changed_ = true;
            } else {
                last_solution_changed_ = false;
            }
            last_solution_good_ = false;
            ROS_DEBUG_STREAM("IK not ok " << err.val);
        }
        sendCurrentMarkers();
        // if(last_solution_changed_) {
        //   std_msgs::ColorRGBA col;
        //   if(last_solution_good_) {
        //     col = good_color_;
        //   } else {
        //     col = bad_color_;
        //   }
        //   makeInteractiveControlMarker(interactive_marker_name_,
        //                                state_.getLinkState(ik_solver_->getTipFrame())->getGlobalLinkTransform(),
        //                                col);
        // }
    }

    void KinematicsGroupVisualization::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        if(interactive_marker_to_group_names_.find(feedback->marker_name) == interactive_marker_to_group_names_.end()) {
            ROS_INFO_STREAM("Getting values for unknown group " << feedback->marker_name);
            return;
        }
        switch (feedback->event_type) {
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            std::string group_name = interactive_marker_to_group_names_[feedback->marker_name];
            Eigen::Affine3d new_pose;
            planning_models::poseFromMsg(feedback->pose, new_pose);
            new_pose = new_pose*relative_transforms_[group_name];
            geometry_msgs::Pose trans_pose;
            planning_models::msgFromPose(new_pose, trans_pose);
            updateEndEffectorState(group_name, trans_pose);
            //setting this so that we save it correctly
            interactive_marker_server_->setPose(feedback->marker_name, feedback->pose);
            interactive_marker_server_->applyChanges();
            last_6dof_marker_pose_.header = feedback->header;
            last_6dof_marker_pose_.pose = feedback->pose;
        }
        break;
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            if(button_click_callback_) {
                button_click_callback_();
            }
            break;
        default:
            ROS_DEBUG_STREAM("Getting event type " << (unsigned int)feedback->event_type);
            break;
        }
    };

    void KinematicsGroupVisualization::makeInteractiveControlMarkers(const std_msgs::ColorRGBA& color,
                                                                     bool add_6dof,
                                                                     bool load_saved)
    {
        if(!visible_) return;

        for(std::map<std::string, std::string>::iterator it = group_to_interactive_marker_names_.begin();
            it != group_to_interactive_marker_names_.end();
            it++)
        {
            visualization_msgs::InteractiveMarker marker;
            if(load_saved) {
                ROS_DEBUG_STREAM("Loading marker " << it->second);
                marker = saved_markers_[it->second];
                removeAxisControls(marker);
                recolorInteractiveMarker(marker, color);
            }
            else
            {
                ROS_DEBUG_STREAM("Creating marker " << it->second);
                std::vector<std::string> button_links = ik_solver_->getEndEffectorLinks().at(it->first);
                if(button_links.empty())
                {
                    button_links.push_back(ik_solver_->getLinkNames().back());
                }
                marker = makeMeshButtonFromLinks(it->second,
                                                 state_,
                                                 ik_solver_->getTipFrames().at(it->first),
                                                 button_links,
                                                 color,
                                                 .35,
                                                 true,
                                                 relative_transforms_[it->first]);
                // - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
                // aleeper: HACK to try out the new 3D cursor with force feedback...
                marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
                // - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
            }

            if(add_6dof) {
                add6DofControl(marker, false);
            }
            marker.description=it->second;

            if(!interaction_enabled_)
            {
                for(size_t i = 0; i < marker.controls.size(); ++i)
                    marker.controls[i].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
                marker.description = "";
            }
            // make6DOFMarker(group_name_+"_ik",
            //                ps,
            //                .3,
            //                false,
            //                false);

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
            // aleeper: HACK to try out the new 3D cursor with force feedback...
            std::string description = "";
            std::string prefix = group_name_ + "_" + "start_position" + "_";
            if(group_name_.find("right") != std::string::npos) description = "control_frame: " + prefix + "r_wrist_roll_link";
            if(group_name_.find("left") != std::string::npos) description =  "control_frame: " + prefix + "l_wrist_roll_link";
            for(size_t i = 0; i < marker.controls.size(); i++)
            {
              marker.controls[i].description = description;
            }
            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

            interactive_marker_server_->insert(marker);
            interactive_marker_server_->setCallback(marker.name,
                                                    boost::bind(&KinematicsGroupVisualization::processInteractiveMarkerFeedback, this, _1));
        }
        default_menu_handler_.reApply(*interactive_marker_server_);
        interactive_marker_server_->applyChanges();
    };

}


