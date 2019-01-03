//Reading the characters from std::in, used for the simulator only
#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <sstream>
#include <termios.h>
#include <stdio.h>

static struct termios old, newt;
int rate = 0;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  newt = old; /* make new settings same as old settings */
  newt.c_lflag &= ~ICANON; /* disable buffered i/o */
  newt.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &newt); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}

/* Let's test it out */
int main(int argc, char **argv) {
  if (argv[1] == NULL)
  	rate = 5;
  else
  	rate = atoi(argv[1]);

  std::cout << rate; 
  ros::init(argc, argv, "target_control");
  ros::NodeHandle n;
  ros::Publisher target_pub = n.advertise<std_msgs::Char>("target_control", 5);
  ros::Rate loop_rate(rate);

  while (ros::ok()){
	 std_msgs:: Char msg;
// Pass the coordinates here
	  msg.data = getche();
	  if(msg.data == 'c')
	  	break;
    ROS_INFO("( %c )", msg.data);
    target_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
} 
