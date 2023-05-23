#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class SIGVerseTb3LaserDistanceSensorTeleopKey
{
private:

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEY_A = 0x61;
  static const char KEY_D = 0x64;
  static const char KEY_H = 0x68;
  static const char KEY_S = 0x73;
  static const char KEY_W = 0x77;

  static const char KEYCODE_SPACE  = 0x20;

  const double LINEAR_VEL  = 0.2;
  const double ANGULAR_VEL = 0.4;

public:
  SIGVerseTb3LaserDistanceSensorTeleopKey();

  void keyLoop(int argc, char** argv);

private:

  static void rosSigintHandler(int sig);
  static int  canReceiveKey( const int fd );

  void moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z);

  void showHelp();
};


SIGVerseTb3LaserDistanceSensorTeleopKey::SIGVerseTb3LaserDistanceSensorTeleopKey()
{
}


void SIGVerseTb3LaserDistanceSensorTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseTb3LaserDistanceSensorTeleopKey::canReceiveKey( const int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}


void SIGVerseTb3LaserDistanceSensorTeleopKey::moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = 0.0;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher.publish(twist);
}


void SIGVerseTb3LaserDistanceSensorTeleopKey::showHelp()
{
  puts("\n");
  puts("---------------------------");
  puts("Operate from keyboard");
  puts("---------------------------");
  puts("arrow keys : Move");
  puts("---------------------------");
  puts("Space: Stop");
  puts("---------------------------");
  puts("w: Go Forward");
  puts("s: Go Back");
  puts("d: Turn Right");
  puts("a: Turn Left");
  puts("---------------------------");
  puts("h: Show help");
}


void SIGVerseTb3LaserDistanceSensorTeleopKey::keyLoop(int argc, char** argv)
{
  char c;
  int  ret;
  char buf[1024];

  /////////////////////////////////////////////
  // get the console in raw mode
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  ros::init(argc, argv, "tb3_lds_teleop_key", ros::init_options::NoSigintHandler);

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(10);

  ros::Publisher pub_base_twist = node_handle.advertise<geometry_msgs::Twist>("/tb3/cmd_vel", 10);

  sleep(2);

  showHelp();

  while (ros::ok())
  {
    if(canReceiveKey(kfd))
    {
      // get the next event from the keyboard
      if((ret = read(kfd, &buf, sizeof(buf))) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      c = buf[ret-1];

      switch(c)
      {
        case KEYCODE_SPACE:
        {
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0);
          break;
        }
        case KEY_W:
        case KEYCODE_UP:
        {
          ROS_DEBUG("Go Forward");
          moveBase(pub_base_twist, +LINEAR_VEL, 0.0);
          break;
        }
        case KEY_S:
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Back");
          moveBase(pub_base_twist, -LINEAR_VEL, 0.0);
          break;
        }
        case KEY_D:
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Turn Right");
          moveBase(pub_base_twist, 0.0, -ANGULAR_VEL);
          break;
        }
        case KEY_A:
        case KEYCODE_LEFT:
        {
          ROS_DEBUG("Turn Left");
          moveBase(pub_base_twist, 0.0, +ANGULAR_VEL);
          break;
        }
        case KEY_H:
        {
          ROS_DEBUG("Show Help");
          showHelp();
          break;
        }
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3LaserDistanceSensorTeleopKey teleop_key;

  teleop_key.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

