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


#include <iostream>
#include "humanoid_fsr/humanoid_fsr.h"


humanoid_fsr::~humanoid_fsr() {
	if (is_connected_)
		disconnect();
}

void humanoid_fsr::disconnect() {
	if (!is_connected_)
		return;
	
	is_connected_ = false;
}



humanoid_fsr::humanoid_fsr()
{
	fsr_inc = false;
	copl = Vector3d::Zero();
	copr = Vector3d::Zero();
}
void humanoid_fsr::run() {
	static ros::Rate rate(2.5*fsr_freq);  //ROS Node Loop Rate
	while (ros::ok()) {
		if(fsr_inc){
			computeCOP();
			computeFT();
			publishCOP();
			publishGRF();
			fsr_inc = false;
		}
		ros::spinOnce();
		rate.sleep();
	}
}

bool humanoid_fsr::connect(const ros::NodeHandle nh) {
	// Initialize ROS nodes
	n = nh;
	// Load ROS Parameters
	loadparams();
	//Subscribe/Publish ROS Topics/Services
	subscribe();
	advertise();

	is_connected_ = true;

	ROS_INFO_STREAM("Humanoid State Estimator Initialized");

	return true;
}



bool humanoid_fsr::connected() {
	return is_connected_;
}

void humanoid_fsr::subscribe()
{
	subscribeToFSR();
	ros::Duration(1.0).sleep();
	getFSRPos();
	ros::Duration(1.0).sleep();
}



void humanoid_fsr::computeCOP() {
	// Computation of the CoP in the Local Coordinate Frame of the Foot
	copl(0) =  lfsr1_msg.wrench.force.z * lfsr1_pos(0) + lfsr2_msg.wrench.force.z * lfsr2_pos(0)+
	lfsr3_msg.wrench.force.z * lfsr3_pos(0) + lfsr4_msg.wrench.force.z * lfsr4_pos(0);

	copl(1) =  lfsr1_msg.wrench.force.z * lfsr1_pos(1) + lfsr2_msg.wrench.force.z * lfsr2_pos(1)+
	lfsr3_msg.wrench.force.z * lfsr3_pos(1) + lfsr4_msg.wrench.force.z * lfsr4_pos(1);
	copl(2) = lfsr1_pos(2);

	copr(0) =  rfsr1_msg.wrench.force.z * rfsr1_pos(0) + rfsr2_msg.wrench.force.z * rfsr2_pos(0)+
	rfsr3_msg.wrench.force.z * rfsr3_pos(0) + rfsr4_msg.wrench.force.z * rfsr4_pos(0);

	copr(1) =  rfsr1_msg.wrench.force.z * rfsr1_pos(1) + rfsr2_msg.wrench.force.z * rfsr2_pos(1)+
	rfsr3_msg.wrench.force.z * rfsr3_pos(1) + rfsr4_msg.wrench.force.z * rfsr4_pos(1);
	copr(2) = rfsr1_pos(2);
}

void humanoid_fsr::computeFT() {

	double fl = lfsr1_msg.wrench.force.z + lfsr2_msg.wrench.force.z + lfsr3_msg.wrench.force.z +  lfsr4_msg.wrench.force.z;
	double fr = rfsr1_msg.wrench.force.z + rfsr2_msg.wrench.force.z + rfsr3_msg.wrench.force.z +  rfsr4_msg.wrench.force.z;
	
	LLegGRF = Vector3d(0,0,fl*g);
	RLegGRF = Vector3d(0,0,fr*g);


	LLegGRT = copl.cross(LLegGRF);
	RLegGRT = copr.cross(RLegGRF);
}


void humanoid_fsr::loadparams() {

	ros::NodeHandle n_p("~");

	// Load Server Parameters
	n_p.param<std::string>("lfoot",lfoot_frame,"l_sole");
	n_p.param<std::string>("rfoot",rfoot_frame,"r_sole");

	n_p.param<double>("fsr_topic_freq",fsr_freq,100.0);
	n_p.param<double>("gravity",g,9.81);

	n_p.param<std::string>("lfsr1",lfsr1_frame,"LFsrFL_frame");
	n_p.param<std::string>("lfsr2",lfsr2_frame,"LFsrFR_frame");
	n_p.param<std::string>("lfsr3",lfsr3_frame,"LFsrRL_frame");
	n_p.param<std::string>("lfsr4",lfsr4_frame,"LFsrRR_frame");

	n_p.param<std::string>("rfsr1",rfsr1_frame,"RFsrFL_frame");
	n_p.param<std::string>("rfsr2",rfsr2_frame,"RFsrFR_frame");
	n_p.param<std::string>("rfsr3",rfsr3_frame,"RFsrRL_frame");
	n_p.param<std::string>("rfsr4",rfsr4_frame,"RFsrRR_frame");


	n_p.param<std::string>("lfsr1_topic",lfsr1_topic,"LfsrFL");
	n_p.param<std::string>("lfsr2_topic",lfsr2_topic,"LfsrFR");
	n_p.param<std::string>("lfsr3_topic",lfsr3_topic,"LfsrRL");
	n_p.param<std::string>("lfsr4_topic",lfsr4_topic,"LfsrRR");
	n_p.param<std::string>("rfsr1_topic",rfsr1_topic,"RfsrFL");
	n_p.param<std::string>("rfsr2_topic",rfsr2_topic,"RfsrFR");
	n_p.param<std::string>("rfsr3_topic",rfsr3_topic,"RfsrRL");
	n_p.param<std::string>("rfsr4_topic",rfsr4_topic,"RfsrRL");

}



void humanoid_fsr::subscribeToFSR()
{
	//Left Foot
	lfsr1_sub = n.subscribe(lfsr1_topic,1000,&humanoid_fsr::lfsr1Cb,this);
	lfsr2_sub = n.subscribe(lfsr2_topic,1000,&humanoid_fsr::lfsr2Cb,this);
	lfsr3_sub = n.subscribe(lfsr3_topic,1000,&humanoid_fsr::lfsr3Cb,this);
	lfsr4_sub = n.subscribe(lfsr4_topic,1000,&humanoid_fsr::lfsr4Cb,this);

	//Right Foot
	rfsr1_sub = n.subscribe(rfsr1_topic,1000,&humanoid_fsr::rfsr1Cb,this);
	rfsr2_sub = n.subscribe(rfsr2_topic,1000,&humanoid_fsr::rfsr2Cb,this);
	rfsr3_sub = n.subscribe(rfsr3_topic,1000,&humanoid_fsr::rfsr3Cb,this);
	rfsr4_sub = n.subscribe(rfsr4_topic,1000,&humanoid_fsr::rfsr4Cb,this);
}

void humanoid_fsr::lfsr1Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	lfsr1_msg = *msg;
	fsr_inc = true;
}
void humanoid_fsr::lfsr2Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	lfsr2_msg = *msg;
}
void humanoid_fsr::lfsr3Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	lfsr3_msg = *msg;
}
void humanoid_fsr::lfsr4Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	lfsr4_msg = *msg;
}
void humanoid_fsr::rfsr1Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rfsr1_msg = *msg;
}
void humanoid_fsr::rfsr2Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rfsr2_msg = *msg;
}
void humanoid_fsr::rfsr3Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rfsr3_msg = *msg;
}
void humanoid_fsr::rfsr4Cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rfsr4_msg = *msg;
}


void humanoid_fsr::getFSRPos()
{
	try{
	Tfsr_listener.lookupTransform(lfoot_frame, lfsr1_frame,  
				ros::Time(0), Tfsr_tf);
	lfsr1_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}

	try{
	Tfsr_listener.lookupTransform(lfoot_frame, lfsr2_frame,  
				ros::Time(0), Tfsr_tf);
	lfsr2_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}

	try{
	Tfsr_listener.lookupTransform(lfoot_frame, lfsr3_frame,  
				ros::Time(0), Tfsr_tf);
	lfsr3_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}

	try{
	Tfsr_listener.lookupTransform(lfoot_frame, lfsr4_frame,  
				ros::Time(0), Tfsr_tf);
	lfsr4_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}

	try{
	Tfsr_listener.lookupTransform(rfoot_frame, rfsr1_frame,  
				ros::Time(0), Tfsr_tf);
	rfsr1_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}


	try{
	Tfsr_listener.lookupTransform(rfoot_frame, rfsr2_frame,  
				ros::Time(0), Tfsr_tf);
	rfsr2_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}


	try{
	Tfsr_listener.lookupTransform(rfoot_frame, rfsr3_frame,  
				ros::Time(0), Tfsr_tf);
	rfsr3_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}

	try{
	Tfsr_listener.lookupTransform(rfoot_frame, rfsr4_frame,  
				ros::Time(0), Tfsr_tf);
	rfsr4_pos << Tfsr_tf.getOrigin().x(), Tfsr_tf.getOrigin().y() , Tfsr_tf.getOrigin().z();
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}
}



void humanoid_fsr::advertise() {

	RLeg_est_pub = n.advertise<geometry_msgs::WrenchStamped>("humanoid_estimator/RLeg/force_torque_states",1000);
	LLeg_est_pub = n.advertise<geometry_msgs::WrenchStamped>("humanoid_estimator/LLeg/force_torque_states",1000);

	COPL_pub = n.advertise<geometry_msgs::PointStamped>("humanoid_estimator/LLeg/COP",1000);
	COPR_pub = n.advertise<geometry_msgs::PointStamped>("humanoid_estimator/RLeg/COP",1000);

}
void humanoid_fsr::publishCOP() {
	COP_msg.point.x = copl(0);
	COP_msg.point.y = copl(1);
	COP_msg.point.z = copl(2);
	COP_msg.header.stamp = ros::Time::now();
	COPL_pub.publish(COP_msg);
	COP_msg.point.x = copr(0);
	COP_msg.point.y = copr(1);
	COP_msg.point.z = copr(2);
	COP_msg.header.stamp = ros::Time::now();
	COPR_pub.publish(COP_msg);
}

void humanoid_fsr::publishGRF() {
	Leg_Wrench_msg.wrench.force.x = LLegGRF(0);
	Leg_Wrench_msg.wrench.force.y = LLegGRF(1);
	Leg_Wrench_msg.wrench.force.z = LLegGRF(2);

	Leg_Wrench_msg.wrench.torque.x = LLegGRT(0);
	Leg_Wrench_msg.wrench.torque.y = LLegGRT(1);
	Leg_Wrench_msg.wrench.torque.z = LLegGRT(2);

	Leg_Wrench_msg.header.frame_id = lfoot_frame;
	Leg_Wrench_msg.header.stamp = ros::Time::now();
	LLeg_est_pub.publish(Leg_Wrench_msg);

	Leg_Wrench_msg.wrench.force.x = RLegGRF(0);
	Leg_Wrench_msg.wrench.force.y = RLegGRF(1);
	Leg_Wrench_msg.wrench.force.z = RLegGRF(2);
	Leg_Wrench_msg.wrench.torque.x = RLegGRT(0);
	Leg_Wrench_msg.wrench.torque.y = RLegGRT(1);
	Leg_Wrench_msg.wrench.torque.z = RLegGRT(2);

	Leg_Wrench_msg.header.frame_id = rfoot_frame;
	Leg_Wrench_msg.header.stamp = ros::Time::now();
	RLeg_est_pub.publish(Leg_Wrench_msg);

}
