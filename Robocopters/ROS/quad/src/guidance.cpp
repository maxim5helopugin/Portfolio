// Guidance node
// Reads the target coordinates
// Decides between follow and attack
// sends vectors to the translator

#include "ros/ros.h"
#include "quad/Vector_of_movement.h"
#include "quad/Target_coordinates.h"
#include <sstream>
#include <iostream>
#include <cmath>


// Constans for now, later will be re-defined
#define HALFSIZE 50          //  viewfinder x dim
#define HALFWIDE 50          //  viewfinder y dim
#define RANGEY 40            //  camera y fov --- change for a more/less responsive
#define RANGEX 57            //  camera x fov --- change for a more/less responsive
#define PI 3.14159265/180.0
#define CONF 75.0             // confidence treshold
#define VELOCITY  1           // 1 m/s

int rate = 0, mode = 0;
double vector[3], oldvec[3], magnitude;
float angley, anglex, speed, coran;

// Pursuit vector generator
// takes x and y coordinates of the target
// translates them to the new vector, based on the old vector
void generate_pursuit_vector(int x, int y){
  angley = 1 * (HALFSIZE - x) * RANGEX/HALFSIZE;
  anglex = -1 * (HALFWIDE - y) * RANGEY/HALFWIDE;
  speed = 1 - ((std::abs(anglex) + std::abs(angley))/(RANGEY+RANGEX));
  magnitude = speed*speed*VELOCITY;
    
  coran = anglex*PI;

  vector[0] = oldvec[0]*cos(coran) + oldvec[1]*(-1*(sin(coran)));
  vector[1] = oldvec[0]*sin(coran) + oldvec[1]*cos(coran);

  vector[0] = ((int)(vector[0]*10000))/10000.0;
  vector[1] = ((int)(vector[1]*10000))/10000.0;
  vector[2] = angley/RANGEY;
  std::cout << "ANGLES : "<< vector[0] << " " << vector[1]; 
 }

//Follow vector, just a pursuit vector, but slower
// depends on the confidence
void generate_follow_vector(int x, int y, int conf){
	generate_pursuit_vector(x,y);
	magnitude = magnitude/100*(conf/2);
}

//Listener function,
// takes the message passed by the image processor
// decides based on the confidence the mode and proceeds
void target_Callback(const quad::Target_coordinates::ConstPtr& msg){
	if(msg->confidence > 0){
		mode = 1;
		if(msg->confidence > CONF)
  			generate_pursuit_vector(msg->x,msg->y);
  		else{
  			generate_follow_vector(msg->x, msg->y, msg->confidence);
  		}
  		std::cout << "\tx " << vector[0] << '\n'
  			<<"\ty " << vector[1] << '\n'
  			<<"\tz " << vector[2] << '\n'
  			<<"\ts " << magnitude << '\n';
  	}
  	else{
  		mode = 0;
  		vector[0] = oldvec[0];
  		vector[1] = oldvec[1];
  		vector[2] = oldvec[2];
 	}
}

//Process the message once, 
// save it in a local variable
void vectorCallback(const quad::Vector_of_movement::ConstPtr& msg){
  oldvec[0] = msg->x;
  oldvec[1] = msg->y;
  oldvec[2] = msg->z;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "talker_listener");

  if (argv[1] == NULL)
  	rate = 5;
  else
  	rate = atoi(argv[1]);

//Initialize the ros talkers and listeners
  ros::NodeHandle pub_vector, sub_vector, sub_target;
  ros::Publisher vector_pub = pub_vector.advertise<quad::Vector_of_movement>("vector_of_movement", 1);
  ros::Subscriber sub = sub_target.subscribe("target", 1, target_Callback);
  ros::Subscriber ex_vector = sub_vector.subscribe("previous_vector",1,vectorCallback);

  ros::Rate loop_rate(rate);

//Loop every 0.2 seconds and read/write the messages
  while (ros::ok()){
	   quad::Vector_of_movement msg;
	   msg.x = vector[0];
	   msg.y = vector[1];
	   msg.z = vector[2];
	   msg.flag = mode;
	   msg.speed = magnitude;
	   std::cout<<((mode==0)?"Search":"Pursuit")<<"\n";
     vector_pub.publish(msg);
     ros::spinOnce();
     loop_rate.sleep();
  }
  return 0;
}

