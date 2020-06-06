#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>
#include <sys/ioctl.h>

#include <map>

#define H 0.7

// Map for acceleration keys
std::map<char, std::vector<float>> speedBindings
{
  {'A', {1, +0.2}}, //multiplicator, additive magnitude [m/s]
  {'B', {1, -0.2}},
  {'0', {0, 0}},
};

// Map for turning keys
std::map<char, std::vector<float>> angleBindings
{
  {'C', {1, -10}},  //multiplicator, additive magnitude [deg]
  {'D', {1, +10}},   
  {'0', {0, 0}},
};

// Map for limiting values
std::map<char, std::vector<float>> limitBindings
{
  {'e', {0, 1.1}}, //case [0,1], magnitude [deg]
  {'d', {0, 0.9}}, // 0 - max angle
  {'w', {1, 1.1}}, // 1 - max speed
  {'s', {1, 0.9}},
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Turning/accelerating: arrow keys

w/s : increase/decrease max speeds by 10%
e/d : increase/decrease max turning angle by 10%


q to quit

)";

// Init variables
float speed(0.0); // Linear velocity (m/s)
float angle(0.0); // Turning angle (deg)
float speedlim(1.0); //speed limiting value
float anglelim(20.0); //speed turning angle value
char key(' ');

// For non-blocking keyboard inputs
bool kbhit(void)
{
  struct termios original;
  tcgetattr(STDIN_FILENO, &original);

  struct termios term;
  memcpy(&term, &original, sizeof(term));

  term.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &term);

  int characters_buffered = 0;
  ioctl(STDIN_FILENO, FIONREAD, &characters_buffered);

  tcsetattr(STDIN_FILENO, TCSANOW, &original);

  bool pressed = (characters_buffered != 0);

  return pressed;
}

void echoOff(void)
{
  struct termios term;
  tcgetattr(STDIN_FILENO, &term);

  term.c_lflag &= ~ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

void echoOn(void)
{
  struct termios term;
  tcgetattr(STDIN_FILENO, &term);

  term.c_lflag |= ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

char getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    printf("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    printf("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
    printf("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    printf("tcsetattr ~ICANON");
  return (buf);
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Create Twist message
  geometry_msgs::Twist twist;

  printf("%s", msg);
  printf("Current: speed %.2f(lim %.2f)\tangle %.2f(lim %.2f) | Awaiting command...\n", speed, speedlim, angle, anglelim);
  
  while(true){
    key = ' ';
    
    echoOff();
    if(kbhit()) {
      // Get the pressed key
      key = getch();
    }
    // If the key corresponds to a key in speedBindings
    if (speedBindings.count(key) == 1)
    {
      // Grab the direction data
      speed = speedBindings[key][0]*speed + speedBindings[key][1];
    
      if (speed > speedlim)
          speed = speedlim;
      if (speed < -speedlim)
          speed = -speedlim;     
    
      printf("Current: speed %.2f(lim %.2f) angle %.2f(lim %.2f) | Last command: %c \n", speed, speedlim, angle, anglelim, key);
    }

    // If it corresponds to a key in angleBindings
    if (angleBindings.count(key) == 1)
    {
      // Grab the speed data
      angle = angleBindings[key][0]*angle + angleBindings[key][1];
      
      if (angle > anglelim)
          angle = anglelim;
      if (angle < -anglelim)
          angle = -anglelim;
 
      printf("Current: speed %.2f(lim %.2f) angle %.2f(lim %.2f) | Last command: %c \n", speed, speedlim, angle, anglelim, key);
    }
    
    // If it corresponds to a key in limitBindings
    if (limitBindings.count(key) == 1)
    {
      // Limit the data
      if (limitBindings[key][0]){
          speedlim = speedlim*limitBindings[key][1];
      }else{
          anglelim = anglelim*limitBindings[key][1];
      }
      
      printf("Current: speed %.2f(lim %.2f) angle %.2f(lim %.2f) | Last command: %c \n", speed, speedlim, angle, anglelim, key);
    }
    
    if (key == 'q')
    {
      printf("Exit \n");
      echoOn();
      break;
    }
  
    // Update the Twist message
    twist.linear.x = speed;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = speed * tan(angle*M_PI/180.0)/H;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}
