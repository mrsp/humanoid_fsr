/*
 * humanoid_state_estimation - a complete state estimation scheme for humanoid robots
 *
 * Copyright 2017-2018 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
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

#ifndef HUMANOID_FSR_H
#define HUMANOID_FSR_H

// ROS Headers
#include <ros/ros.h>


#include <eigen3/Eigen/Dense>
// ROS Messages
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>



using namespace Eigen;
using namespace std;

class humanoid_fsr{
private:
	// ROS Standard Variables
	ros::NodeHandle n;

	ros::Publisher  LLeg_est_pub, RLeg_est_pub, COPL_pub, COPR_pub;
    ros::Subscriber lfsr1_sub, lfsr2_sub, lfsr3_sub, lfsr4_sub,
		rfsr1_sub, rfsr2_sub, rfsr3_sub, rfsr4_sub;
	
	double  fsr_freq, g;
	bool fsr_inc;
	
	//ROS Messages
	

	geometry_msgs::WrenchStamped Leg_Wrench_msg,  lfsr1_msg, lfsr2_msg,
	lfsr3_msg, lfsr4_msg, rfsr1_msg, rfsr2_msg, rfsr3_msg, rfsr4_msg;
   
	geometry_msgs::PointStamped COP_msg;

	// Helper
	bool is_connected_;


	string  lfoot_frame, rfoot_frame;
	
	tf::TransformListener Tfsr_listener;
	tf::StampedTransform Tfsr_tf;
	string lfsr1_frame, lfsr2_frame, lfsr3_frame, lfsr4_frame;
    string rfsr1_frame, rfsr2_frame, rfsr3_frame, rfsr4_frame;

    Vector3d lfsr1_pos, lfsr2_pos, lfsr3_pos, lfsr4_pos;
	Vector3d rfsr1_pos, rfsr2_pos, rfsr3_pos, rfsr4_pos;
	Vector3d copl, copr;


	Vector3d LLegGRF, RLegGRF;
  	Vector3d LLegGRT, RLegGRT;


	/** Real odometry Data **/
     string lfsr1_topic,lfsr2_topic,lfsr3_topic,lfsr4_topic,rfsr1_topic,rfsr2_topic,rfsr3_topic,rfsr4_topic;
	
	//Odometry, from supportleg to inertial, transformation from support leg to other leg
	 void subscribeToFSR();

	 void lfsr1Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void lfsr2Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void lfsr3Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void lfsr4Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void rfsr1Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void rfsr2Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void rfsr3Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	 void rfsr4Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	 void computeFT();
	 void computeCOP();
	 void getFSRPos();



	void publishGRF();

	void publishCOP();
	// Advertise to ROS Topics
	void advertise();
	void subscribe();

public:


	// Constructor/Destructor
	humanoid_fsr();

	~humanoid_fsr();

	// Connect/Disconnet to ALProxies
	bool connect(const ros::NodeHandle nh);

	void disconnect();



	// Parameter Server
	void loadparams();

	void run();

	bool connected();


};

#endif // HUMANOID_FSR_H
