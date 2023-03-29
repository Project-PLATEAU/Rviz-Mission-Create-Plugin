/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "waypoint_nav_tool.h"

//#include "waypoint_nav_frame.h"
//#include "waypoint_nav_frame.h"

#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

#include <QFileDialog>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace waypoint_nav_plugin {

    struct MissionKeywords {
        inline static const std::string kPosition = "position";
        inline static const std::string kPose = "pose";

    };

    WaypointFrame::WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode *> *map_ptr,
                                 interactive_markers::InteractiveMarkerServer *server, int *unique_ind, QWidget *parent,
                                 WaypointNavTool *wp_tool)
            : QWidget(parent), context_(context), ui_(new Ui::WaypointNavigationWidget()), sn_map_ptr_(map_ptr),
              unique_ind_(unique_ind), server_(server), frame_id_("map"), default_height_(0.0),
              selected_marker_name_(std::string(g_wp_name_prefix) + "1"), wp_nav_tool_(wp_tool) {
        scene_manager_ = context_->getSceneManager();

        // set up the GUI
        ui_->setupUi(this);

        wp_pub_ = nh_.advertise<mission_plan_msg::MissionPlan>("waypoints", 1);
        mission_path_pub_ = nh_.advertise<nav_msgs::Path>("mission_path", 1);
        timer = nh_.createTimer(ros::Duration(2), &WaypointFrame::pathFeedBack,this);

        //connect the Qt signals and slots
        connect(ui_->publish_wp_button, SIGNAL(clicked()), this, SLOT(publishButtonClicked()));
        connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this, SLOT(topicChanged()));
        connect(ui_->frame_line_edit, SIGNAL(editingFinished()), this, SLOT(frameChanged()));
        connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(heightChanged(double)));
        connect(ui_->clear_all_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));

        connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
        connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
        connect(ui_->z_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
        connect(ui_->yaw_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));

        connect(ui_->save_wp_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
        connect(ui_->load_wp_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));

    }

    WaypointFrame::~WaypointFrame() {
        delete ui_;
        sn_map_ptr_ = NULL;
    }

    void WaypointFrame::enable() {
        // activate the frame
        show();
    }

    void WaypointFrame::disable() {
        wp_pub_.shutdown();
        hide();
    }

    void WaypointFrame::saveButtonClicked() {
        QString filename =
                QFileDialog::getSaveFileName(0, tr("Save Mission"), "waypoints.json",
                                             tr("Mission Files (*.bag *.yaml *.json)"));

        if (filename.isEmpty()) {
            ROS_ERROR("No mission filename selected");
            return;
        }

        const std::string filename_str = filename.toStdString();
        ROS_INFO_STREAM("saving waypoints to " << filename_str);
        if (filename.endsWith(".json")) {
            saveToJson(filename_str);
        } else {
            ROS_INFO_STREAM("Invalid mission file format: " << filename_str);
        }
    }


    void WaypointFrame::saveToJson(const std::string &filename) {

        auto jsonObjects = json::array();
        std::map<int, Ogre::SceneNode *>::iterator sn_it;

        for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
            const Ogre::Vector3 position = sn_it->second->getPosition();
            const Ogre::Quaternion quat = sn_it->second->getOrientation();
            const Ogre::Any any_obj = sn_it->second->getUserAny();
            std::string mission_ty = any_obj.get<Ogre::String>();


            tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            std::vector<float> values = {position.x, position.y, position.z, static_cast<float>(roll),
                                         static_cast<float>( pitch), static_cast<float>(yaw)};

            json obj = json::object({{"MissionPoint", values},{"Type",mission_ty}});
            jsonObjects.push_back(obj);
        }


        std::string s = jsonObjects.dump(2);
        std::ofstream fout(filename);
        fout << s.c_str();


    }

    void WaypointFrame::loadButtonClicked() {
        const QString filename = QFileDialog::getOpenFileName(
                0, tr("Load Mission"), "~/", tr("Mission Files (*.bag *.yaml *.json)"));

        if (filename.isEmpty()) {
            ROS_ERROR("No mission file selected");
            return;
        }

        const std::string filename_str = filename.toStdString();
        ROS_INFO("loading waypoints from %s", filename_str.c_str());
        if (filename.endsWith(".json")) {
            loadFromJson(filename_str);
        } else {
            ROS_INFO_STREAM("Invalid mission file format: " << filename_str);
        }
    }


    void WaypointFrame::loadFromJson(const std::string &filename) {
        std::ifstream i(filename);
        json j_obj;
        i >> j_obj;


        for (auto &element: j_obj) {

            std::vector<double> mp = element.at("MissionPoint");

            std::string mt = element.at("Type");

//            for (const auto &item : mp) {
//                std::cout << item << "; ";
//            }


            Ogre::Vector3 position;
            position.x = mp[0];
            position.y = mp[1];
            position.z = mp[2];
            ROS_INFO_STREAM("Load waypoint whose x y z are: " << position.x << ", "
                                                             << position.y << ", "
                                                             << position.z<< " type:" << mt);

            tf2::Quaternion q;
            q.setRPY(mp[3], mp[4], mp[5]);
            q = q.normalize();

            Ogre::Quaternion quat;
            quat.x = q.x();
            quat.y = q.y();
            quat.z = q.z();
            quat.w = q.w();
            Ogre::String mission_type = mt;

            wp_nav_tool_->makeIm(position, quat, mission_type,
                                 ui_->sixDcheckBox->checkState() == Qt::Checked);

        }


        //publishButtonClicked();
    }

    void WaypointFrame::publishButtonClicked() {
        mission_plan_msg::MissionPlan missionPlan;

        std::map<int, Ogre::SceneNode *>::iterator sn_it;
        for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++) {
            Ogre::Vector3 position;
            position = sn_it->second->getPosition();

            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = position.x;
            pos.pose.position.y = position.y;
            pos.pose.position.z = position.z;

            Ogre::Quaternion quat;
            quat = sn_it->second->getOrientation();
            pos.pose.orientation.x = quat.x;
            pos.pose.orientation.y = quat.y;
            pos.pose.orientation.z = quat.z;
            pos.pose.orientation.w = quat.w;

            missionPlan.poses.push_back(pos);

            const Ogre::Any any_obj = sn_it->second->getUserAny();
            std::string mission_ty = any_obj.get<Ogre::String>();
            std_msgs::String mission_name;
            mission_name.data = mission_ty;
            ROS_INFO("in mission %s",mission_ty.c_str());
            missionPlan.mission_names.push_back(mission_name.data);
        }

        missionPlan.header.frame_id = frame_id_.toStdString();
        wp_pub_.publish(missionPlan);
    }

    void WaypointFrame::pathFeedBack(const ros::TimerEvent&) {
        nav_msgs::Path missionPath;

        std::map<int, Ogre::SceneNode *>::iterator sn_it;
        for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++) {
            Ogre::Vector3 position;
            position = sn_it->second->getPosition();

            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = position.x;
            pos.pose.position.y = position.y;
            pos.pose.position.z = position.z;

            Ogre::Quaternion quat;
            quat = sn_it->second->getOrientation();
            pos.pose.orientation.x = quat.x;
            pos.pose.orientation.y = quat.y;
            pos.pose.orientation.z = quat.z;
            pos.pose.orientation.w = quat.w;

            missionPath.poses.push_back(pos);
        }
        missionPath.header.frame_id = frame_id_.toStdString();
        mission_path_pub_.publish(missionPath);
    }

    void WaypointFrame::clearAllWaypoints() {
        //destroy the ogre scene nodes
        std::map<int, Ogre::SceneNode *>::iterator sn_it;
        for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++) {
            scene_manager_->destroySceneNode(sn_it->second);
        }

        //clear the waypoint map and reset index
        sn_map_ptr_->clear();
        *unique_ind_ = 0;

        //clear the interactive markers
        server_->clear();
        server_->applyChanges();
    }

    void WaypointFrame::heightChanged(double h) {
        boost::mutex::scoped_lock lock(frame_updates_mutex_);
        default_height_ = h;
    }

    void WaypointFrame::setSelectedMarkerName(std::string name) {
        selected_marker_name_ = name;
    }

    void WaypointFrame::poseChanged(double val) {
        auto sn_entry = sn_map_ptr_->end();
        try {
            const int selected_marker_idx =
                    std::stoi(selected_marker_name_.substr(strlen(g_wp_name_prefix)));
            sn_entry = sn_map_ptr_->find(selected_marker_idx);
        } catch (const std::logic_error &e) {
            ROS_ERROR_STREAM(e.what());
            return;
        }

        if (sn_entry == sn_map_ptr_->end())
            ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
        else {
            Ogre::Vector3 position;
            Ogre::Quaternion quat;
            getPose(position, quat);

            sn_entry->second->setPosition(position);
            sn_entry->second->setOrientation(quat);

            std::stringstream wp_name;
            wp_name << g_wp_name_prefix << sn_entry->first;
            std::string wp_name_str(wp_name.str());

            visualization_msgs::InteractiveMarker int_marker;
            if (server_->get(wp_name_str, int_marker)) {
                int_marker.pose.position.x = position.x;
                int_marker.pose.position.y = position.y;
                int_marker.pose.position.z = position.z;

                int_marker.pose.orientation.x = quat.x;
                int_marker.pose.orientation.y = quat.y;
                int_marker.pose.orientation.z = quat.z;
                int_marker.pose.orientation.w = quat.w;

                server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
            }
            server_->applyChanges();
        }
    }

    void WaypointFrame::frameChanged() {

        boost::mutex::scoped_lock lock(frame_updates_mutex_);
        QString new_frame = ui_->frame_line_edit->text();

        // Only take action if the frame has changed.
        if ((new_frame != frame_id_) && (new_frame != "")) {
            frame_id_ = new_frame;
            ROS_INFO("new frame: %s", frame_id_.toStdString().c_str());

            // update the frames for all interactive markers
            std::map<int, Ogre::SceneNode *>::iterator sn_it;
            for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++) {
                std::stringstream wp_name;
                wp_name << "waypoint" << sn_it->first;
                std::string wp_name_str(wp_name.str());

                visualization_msgs::InteractiveMarker int_marker;
                if (server_->get(wp_name_str, int_marker)) {
                    int_marker.header.frame_id = new_frame.toStdString();
                    server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
                }
            }
            server_->applyChanges();
        }
    }

    void WaypointFrame::topicChanged() {
        QString new_topic = ui_->topic_line_edit->text();

        // Only take action if the name has changed.
        if (new_topic != output_topic_) {
            wp_pub_.shutdown();
            output_topic_ = new_topic;

            if ((output_topic_ != "") && (output_topic_ != "/")) {
                wp_pub_ = nh_.advertise<mission_plan_msg::MissionPlan>(output_topic_.toStdString(), 1);
            }
        }
    }

    void WaypointFrame::setWpCount(int size) {
        std::ostringstream stringStream;
        stringStream << "Total Wp: " << size;
        std::string st = stringStream.str();

        boost::mutex::scoped_lock lock(frame_updates_mutex_);
        ui_->waypoint_count_label->setText(QString::fromStdString(st));
    }

    void WaypointFrame::setConfig(QString topic, QString frame, float height) {
        {
            boost::mutex::scoped_lock lock(frame_updates_mutex_);
            ui_->topic_line_edit->blockSignals(true);
            ui_->frame_line_edit->blockSignals(true);
            ui_->wp_height_doubleSpinBox->blockSignals(true);

            ui_->topic_line_edit->setText(topic);
            ui_->frame_line_edit->setText(frame);
            ui_->wp_height_doubleSpinBox->setValue(height);

            ui_->topic_line_edit->blockSignals(false);
            ui_->frame_line_edit->blockSignals(false);
            ui_->wp_height_doubleSpinBox->blockSignals(false);

        }
        topicChanged();
        frameChanged();
        heightChanged(height);
    }

    void WaypointFrame::getPose(Ogre::Vector3 &position, Ogre::Quaternion &quat) {
        {
            boost::mutex::scoped_lock lock(frame_updates_mutex_);
            position.x = ui_->x_doubleSpinBox->value();
            position.y = ui_->y_doubleSpinBox->value();
            position.z = ui_->z_doubleSpinBox->value();
            double yaw = ui_->yaw_doubleSpinBox->value();

            tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
            quat.x = qt.x();
            quat.y = qt.y();
            quat.z = qt.z();
            quat.w = qt.w();

        }
    }

    void WaypointFrame::setPose(const Ogre::Vector3 &position, const Ogre::Quaternion &quat) {
        {
            //boost::mutex::scoped_lock lock(frame_updates_mutex_);
            //block spinbox signals
            ui_->x_doubleSpinBox->blockSignals(true);
            ui_->y_doubleSpinBox->blockSignals(true);
            ui_->z_doubleSpinBox->blockSignals(true);
            ui_->yaw_doubleSpinBox->blockSignals(true);

            ui_->x_doubleSpinBox->setValue(position.x);
            ui_->y_doubleSpinBox->setValue(position.y);
            ui_->z_doubleSpinBox->setValue(position.z);

            tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
            ui_->yaw_doubleSpinBox->setValue(tf::getYaw(qt));

            //enable the signals
            ui_->x_doubleSpinBox->blockSignals(false);
            ui_->y_doubleSpinBox->blockSignals(false);
            ui_->z_doubleSpinBox->blockSignals(false);
            ui_->yaw_doubleSpinBox->blockSignals(false);

        }
    }

    void WaypointFrame::setWpLabel(Ogre::Vector3 position) {
        {
            //boost::mutex::scoped_lock lock(frame_updates_mutex_);
            std::ostringstream stringStream;
            stringStream.precision(2);
            stringStream << selected_marker_name_;
            //stringStream << " x: " << position.x << " y: " << position.y << " z: " << position.z;
            std::string label = stringStream.str();

            ui_->sel_wp_label->setText(QString::fromStdString(label));
        }
    }

    double WaypointFrame::getDefaultHeight() {
        boost::mutex::scoped_lock lock(frame_updates_mutex_);
        return default_height_;
    }

    QString WaypointFrame::getFrameId() {
        boost::mutex::scoped_lock lock(frame_updates_mutex_);
        return frame_id_;
    }

    QString WaypointFrame::getOutputTopic() {
        boost::mutex::scoped_lock lock(frame_updates_mutex_);
        return output_topic_;
    }
} // namespace
