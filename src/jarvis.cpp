#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include <fstream>
#include <iostream>
#include <ctime>

using namespace ros;
using namespace std;
class PeopleListener{
	private:
		ofstream myfile;
		Publisher pub;
		Subscriber sub;
		void sub_callback(const std_msgs::String::ConstPtr& msg);
	public:
		NodeHandle nh;
		sound_play::SoundClient sc;
		PeopleListener();

};
PeopleListener::PeopleListener(){
	sub = nh.subscribe("names", 1000, &PeopleListener::sub_callback, this);
}
void PeopleListener::sub_callback(const std_msgs::String::ConstPtr& msg){
	string s,m;
	m =  msg->data.c_str();
	ROS_INFO("%s",msg->data.c_str());
	s = "Hello " + m;
	sc.say(s);
	sleep(2);
	time_t now = time(0);
	char* dt = ctime(&now);
	myfile.open("people.log", ios::app);
	myfile << m << " "<< dt << "\n";
	myfile.close();
}
int main(int argc, char**argv) {
	init(argc, argv, "jarvis");
	PeopleListener listener;
	spin();
}
