#include <ros/ros.h>
#include<std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>
#include <joystick_mani/mydmxel.h>

#define ANGULAR1_MAX 10.0  //각속도1의 최댓값 10.0 ~ -10.0
#define ANGULAR2_MAX 10.0  //각속도2의 최댓값 10.0 ~ -10.0
#define SENSITIVITY 0.1    //조이스틱 감도 조절


typedef struct moter_angle{
  int mo1=0;
  int mo2=0;
  int mo3=0;
  int mo4=0;
  int mo5=0;
  double theta;
}moter;

int val2dy(double theta){
  int a = theta*(180/3.14)*11.377;
  int dmx=2048+a;

  return dmx;
}

moter calculate_angle(float x, float y, float z,int l1, int l2){
  moter dmxel;

  double theta1,theta2,theta3;
  double cos3, sin3;
  double ex,ey;
  
  ex = sqrt((x*x)+(y*y));
  ey=z;

  //세타 3 구하기
  cos3= ((x*x)+(y*y))/((l1*l1)+(l2*l2)+(2*l1*l2));
  sin3= sqrt(1-(cos3*cos3));

  theta3= atan2(sin3,cos3);

  //세타2 구하기

  theta2=atan2(ex,ey)-atan2(l1+(l2*cos3),(l2*sin3));

  theta1 = atan2(y,x);

  dmxel.mo1=val2dy(theta1);
  dmxel.mo2=val2dy(theta2);
  dmxel.mo3=val2dy(theta3);
  dmxel.theta= theta1+theta2-1.5708;

  return dmxel;
}

class Joy_cmd_vel_mani
{
public:
  Joy_cmd_vel_mani();
  void operate();
  float MAXtoMIN(float angular, float max);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle n;
 
  
//조이값 저장변수
  float arm_z;   //enf z좌표 업다운
  float arm_x;   //enf x좌표 왼오
  float arm_y;
  float hand;   //조이스틱에서 손목 피치
  float grip_open;
  float grip_close;
  int init_grip;
  int init_ride;
  int right_angle;
//현재값 저장변수
  int crt_init_grip;
  int crt_init_ride;
  int crt_right_angle;
  float crt_arm_x; 
  float crt_arm_z;
  float crt_arm_y;  
  float crt_hand;    
  double crt_grip;


//계산변수
  int l1=23,l2=18;
 
//모터변수



  ros::Publisher vel_pub_;   // 
  ros::Subscriber joy_sub_;  // joy 토픽에서 조이스틱 메시지를 받아 joyCallback 콜백 함수를 호출.
};

Joy_cmd_vel_mani::Joy_cmd_vel_mani()
{
  vel_pub_ = n.advertise<std_msgs::Int64>("/cmd_vel",1000);
  joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_cmd_vel_mani::joyCallback, this);
}

void Joy_cmd_vel_mani::operate()
{
    joystick_mani::mydmxel msg;


  crt_hand= MAXtoMIN(crt_hand, 180);  // 현재 값이 180.0 ~ -180.0 사이에 있도록 함.

  //현재값 저장
  float checkbuf= sqrt((arm_x*arm_x)+(arm_y*arm_y)+(arm_z*arm_z));
  bool check;
  if(checkbuf<45){
    check=true;
  }
  else check=false;



  //최대값 제한필요 

  if(check==true){
  crt_arm_z+= arm_z;
  crt_arm_x+=arm_x;
  crt_arm_y+= arm_y;
  crt_hand+= hand;
  crt_grip+=grip_open ;
 crt_grip +=grip_close ;

  if(crt_init_grip!=0){crt_init_grip+= init_grip;}
  if(crt_init_ride!=0){crt_init_ride+= init_ride;}
  if(crt_right_angle!=0){crt_right_angle+=right_angle;} 

  moter dmx;
  dmx=calculate_angle(arm_x,arm_y,arm_z,l1,l2);
  dmx.mo5=val2dy(crt_grip);
  dmx.mo4=val2dy(crt_hand);

  if(crt_right_angle==1){
    dmx.mo4=val2dy(dmx.theta);
    crt_right_angle=0;
  }
  if(crt_init_grip==1){
      dmx.mo1=2047;
      dmx.mo2=2047;
      dmx.mo3=2047;     
      crt_init_grip=0; 
  }
 if(crt_init_ride==1){
      dmx.mo1=2047;
      dmx.mo2=2047;
      dmx.mo3=2047;     
      crt_init_ride=0; 
  }
msg.moter1=dmx.mo1;
msg.moter2=dmx.mo2;
msg.moter3=dmx.mo3;
msg.moter4=dmx.mo4;
msg.moter5=dmx.mo5;
  vel_pub_.publish(msg);

  crt_hand += SENSITIVITY * hand;  //조이스틱으로 변경한 정도만큼 더해서 현재 각속도1에 저장함
}
}

// 조이스틱 메시지에서 각속도 목표값을 읽어와 변수에 저장함.
void Joy_cmd_vel_mani::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  /*float형식*/
  //_______= joy->axes[0];  //조이스틱에서 왼쪽 방향키 왼쪽 오른쪽
  hand = joy->axes[1];  //조이스틱에서 왼쪽 방향키 위아래
  grip_close= joy->axes[2];  //조이스틱에서 L2
  //= joy->axes[3];  //조이스틱에서 오른쪽 방향키 왼쪽 오른쪽
  arm_z = joy->axes[4];  //조이스틱에서 오른쪽 방향키 위아래
  grip_open = joy->axes[5];  //조이스틱에서 R2
  arm_x= joy->axes[6];  //조이스틱에서 왼쪽위 방향키 왼쪽 오른쪽
  arm_y = joy->axes[7];  //조이스틱에서 왼쪽위 방향키 위아래

  /*int 형식*/
  //_______ = joy->buttons[0];  //조이스틱에서 오른쪽 X표시 버튼
  right_angle= joy->buttons[1];  //조이스틱에서 오른쪽 O표시 버튼
  //_______ = joy->buttons[2];  //조이스틱에서 오른쪽 삼각형표시 버튼
  //_______ = joy->buttons[3];  //조이스틱에서 오른쪽 사각형표시 버튼
  init_grip = joy->buttons[4];  //조이스틱에서 L1
  init_ride = joy->buttons[5];  //조이스틱에서 R1
  //_______ = joy->buttons[6];  //조이스틱에서 L2
  //_______ = joy->buttons[7];  //조이스틱에서 R2
  //_______ = joy->buttons[8];  //조이스틱에서 share
  //_______ = joy->buttons[9];  //조이스틱에서 options
  //_______ = joy->buttons[10];  //조이스틱에서 가운데 버튼
  //_______ = joy->buttons[11];  //조이스틱에서 왼쪽 스틱 누르기
  //_______ = joy->buttons[12];  //조이스틱에서 오른쪽 스틱 누르기
}

// 해당 각속도의 최대 최소 범위 설정함.
float Joy_cmd_vel_mani::MAXtoMIN(float angular, float max)
{
  if (angular > max)
  {
    angular = max;
  }
  else if (angular < -max)
  {
    angular = -max;
  }
  return angular;
}

  

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "vel");  // ROS 노드를 초기화하고 vel 이라는 노드를 생성함.
  Joy_cmd_vel_mani vel;          // Joy_cmd_vel_mani 클래스의 객체를 생성함.

  ros::Rate loop_rate(33);  // 노드가 주기적으로 실행되도록 루프를 설정함.
  while (ros::ok())         // ROS가 정상적으로 실행 중인 동안 계속해서 루프를 실행함.
  {
    vel.operate();      // operate 함수를 호출함.
    ros::spinOnce();    // 콜백 함수를 호출하고 메시지 큐를 처리함.
    loop_rate.sleep();  // 설정된 주기에 따라 루프를 대기
  }
  return 0;
}