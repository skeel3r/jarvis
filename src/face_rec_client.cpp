/* Copyright 2012 Pouyan Ziafati, University of Luxembourg and Utrecht University

* actionlib client example for the face_recognition simple actionlib server. The client subscribes to face_recognition::FRClientGoal messages. Each FRClientGoal message contains an order_id and an order_argument which specify a goal to be executed by the face_recognition server. After receiving a message, the client sends the corresponding goal to the server. By registering relevant call back functions, the client receives feedback and result information from the execution of goals in the server and prints such information on the terminal.

*Provided by modifying the ROS wiki tutorial: "actionlib_tutorials/Tutorials/Writing a Callback Based Simple Action Client (last edited 2010-09-13 17:32:34 by VijayPradeep)"

*License: Creative Commons Attribution 3.0.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <signal.h>
face_recognition::FaceRecognitionGoal goal; //Goal message
typedef actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> Client; //action lib client

class FaceClient{
	public:
		Client *ac = new Client("face_recognition", true);
		FaceClient();// : ac ("face_recognition", true)

		ros::NodeHandle n;
		ros::Subscriber sub ;
		ros::Publisher names_pub;

		void doneCb(const actionlib::SimpleClientGoalState& state,
			    const face_recognition::FaceRecognitionResultConstPtr& result);
		void activeCb(void);
		void feedbackCb(const face_recognition::FaceRecognitionFeedbackConstPtr& feedback);
		void frclientCallback(const face_recognition::FRClientGoalConstPtr& msg);
		void exit_handler(int s);
		private:
//			Client ac;//("face_recognition", true);
		// Called once when the goal completes

//		struct sigaction sigIntHandler;
//		sigIntHandler.sa_handler = exit_handler;
/*		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);
		//wait for the server
		ac->waitForServer();
*/
};

FaceClient::FaceClient(){
			names_pub = n.advertise<std_msgs::String>("names", 1);
			sub = n.subscribe("fr_order", 1, &FaceClient::frclientCallback, this);
			ROS_INFO("Waiting for action server to start.");
			ac->waitForServer();
			ROS_INFO("Action server started, sending goal.");
}

void FaceClient::doneCb(const actionlib::SimpleClientGoalState& state,
	    const face_recognition::FaceRecognitionResultConstPtr& result)
{
  ROS_INFO("Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());
  if(state.toString() != "SUCCEEDED") return;
  if( result->order_id==0)
    ROS_INFO("%s was recognized with confidence %f", result->names[0].c_str(),result->confidence[0]);
}

// Called once when the goal becomes active
void FaceClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void FaceClient::feedbackCb(const face_recognition::FaceRecognitionFeedbackConstPtr& feedback)
{
  std_msgs::String msg;
  std::stringstream ss;
  ROS_INFO("Received feedback from Goal [%d] ", feedback->order_id);
  if(feedback->order_id==1 )
    msg.data = feedback->names[0].c_str();
    names_pub.publish(msg);
    ROS_INFO("%s was recognized with confidence %f", feedback->names[0].c_str(),feedback->confidence[0]);
  if( feedback->order_id==2)
    ROS_INFO("A picture of %s was successfully added to the training images",feedback->names[0].c_str());
}
//called for every FRClientGoal message received by the client. Client processes each message and sends the corresponding goal to the server and registers feedback and result and status call back functions.
void FaceClient::frclientCallback(const face_recognition::FRClientGoalConstPtr& msg)
  {


     ROS_INFO("request for sending goal [%i] is received", msg->order_id);
     goal.order_id = msg->order_id;
     goal.order_argument = msg->order_argument;
     ac->sendGoal(goal, boost::bind(&FaceClient::doneCb, this, _1, _2), boost::bind(&FaceClient::activeCb, this), boost::bind(&FaceClient::feedbackCb, this, _1));

  }


//shut down
void FaceClient::exit_handler(int s)
{
  delete(ac);
  ros::shutdown();
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "face_recognition_client");
  FaceClient myclient;
  ros::spin();
  return 0;
}

