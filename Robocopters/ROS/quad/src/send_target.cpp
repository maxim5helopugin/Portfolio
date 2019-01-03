// Visual of the simulator to mimic the targer viewfinder
#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "quad/Target_coordinates.h"
#include "quad/Sonars.h"
#include <sstream>

#define HALFX 10
#define HALFY 15

float i, j, rate = 0;
int confidence = 0;
int curx, cury;
int sonars[4];

// Control the new target location with "W A S D", any other letter shall stop the movement
// 'c' to close the listener, 't' toggles target, 'r' - confidence
void controlCallback(const std_msgs::Char::ConstPtr& msg){
  if(msg->data == 'a')
    cury = cury -1;
  else if(msg->data == 'd')
    cury = cury + 1;
  else if(msg->data == 'w')
    curx = curx -1;
  else if(msg->data == 's')
    curx = curx + 1;
  else if(msg->data == 't')
    if(confidence == 0)
	   confidence = 80;
    else
	   confidence = 0;
  else if(msg->data == 'r')
    if(confidence == 80.0)
		confidence = 50.0;
    else
		confidence = 80.0;
  else if(msg->data == 'u'){
    sonars[0] = 1;
    sonars[1] = 1;
    sonars[2] = 1;
    sonars[3] = 1;
  }
  else if(msg->data == 'i'){
    sonars[0] = 2;
    sonars[1] = 2;
    sonars[2] = 2;
    sonars[3] = 2;    
  }
  else{
    i=0;
    j=0;
  }
}

// 'GUI'
void print_tar(){
  for(int i = 0; i<20; i++){
    for(int j = 0; j<30; j++){
      if(i==0 || i==19){
        std::cout << "--";
        continue;
      }
      if(j==0 || j == 29){
        std::cout << "|";
        continue;
      }

      if(i==curx && j==cury && confidence > 0)
	if(confidence == 80.0)
        	std::cout << "+ ";
	else
		std::cout << "? ";
      else
        std::cout << "  ";
    }
    std::cout << "\n";
  }
  std::cout << "Sensor status : \t";
  std::cout << ((sonars[0]<2)?'.':'_');
  std::cout << ((sonars[1]<2)?'.':'\\');
  std::cout << ((sonars[2]<2)?'.':'/');
  std::cout << ((sonars[3]<2)?'.':'_');

  std::cout << "\n";
}


int main(int argc, char **argv)
{
  if (argv[1] == NULL)
    rate = 5;
  else
    rate = atoi(argv[1]);

  for(int i =0; i< 4; i++){
    sonars[i] = 2;
  }

  ros::init(argc, argv, "target_finder");
  ros::NodeHandle n, contr, m;
  ros::Subscriber sub = contr.subscribe("target_control", 5, controlCallback);
  ros::Publisher target_pub = n.advertise<quad::Target_coordinates>("target", 5);
  ros::Publisher sonars_pub = m.advertise<quad::Sonars>("sonars", 5);
  ros::Rate loop_rate(rate);

  curx = HALFX;
  cury = HALFY;

  while (ros::ok())
  {
	 quad::Target_coordinates msg;
   quad::Sonars avoid;

  avoid.sonar1 = sonars[0];
  avoid.sonar2 = sonars[1];
  avoid.sonar3 = sonars[2];
  avoid.sonar4 = sonars[3];

	   msg.x = curx;
	   msg.y = cury;
	   msg.confidence = confidence;

    if(curx >= 20 || curx<=0)
      msg.x = HALFX;
    if(cury >=30 || cury <= 0)
      msg.y = HALFY;

     curx = msg.x;
     cury = msg.y;

     msg.x = curx*50/HALFX;
     msg.y = cury*50/HALFY;
    std::cout << "Target" << ": ( " << msg.x << ", " << msg.y <<
    " ), confidence = " <<  msg.confidence << '\n';
     print_tar();

    target_pub.publish(msg);
    sonars_pub.publish(avoid);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
