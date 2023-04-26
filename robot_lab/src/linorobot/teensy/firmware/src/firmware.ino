#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include "ros.h"
#include "ros/time.h"
#include <std_msgs/Int16.h>
// tệp tiêu đề để xuất bản vận tốc cho odom
#include "lino_msgs/Velocities.h"
// tệp tiêu đề cho cmd_subscribe đến "cmd_vel"
#include "geometry_msgs/Twist.h"
// tập tin tiêu đề cho imu
#include "lino_msgs/Imu.h"

#include "lino_base_config.h"
#include "Kinematics.h"
#include "Imu.h"


#define ENCODER_OPTIMIZE_INTERRUPTS 
extern volatile unsigned long timer0_millis;
#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5


Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;


void commandCallback(const geometry_msgs::Twist& cmd_msg);
void commandCallback1(const geometry_msgs::Twist& cmd_msg);
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_sub1("cmd_vel1", commandCallback1);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

// hc-sr04
std_msgs::Int16 range;
ros::Publisher range_pub("range", &range);
const int trigpin = 40;
const int echopin = 41;
long duration = 0;

//-----------------------------//

long int pulse_left1 = 0, pulse_right2 = 0,pulse_left3 = 0, pulse_right4 = 0 ;
//------------------interrupt functions (Read encoder)----------------//
 void _interrupt_left1()
 {
   if(digitalRead(MOTOR1_ENCODER_B))
     pulse_left1--;
   else
     pulse_left1++;
 }

 void _interrupt_right2()
 {
   if(digitalRead(MOTOR2_ENCODER_B))
     pulse_right2--;
   else
     pulse_right2++;
 }
 void _interrupt_left3()
 {
   if(digitalRead(MOTOR3_ENCODER_B))
     pulse_left3--;
   else
     pulse_left3++;
 }

 void _interrupt_right4()
 {
   if(digitalRead(MOTOR4_ENCODER_B))
     pulse_right4--;
   else
     pulse_right4++;
 }
//-------------------------------------------------------------------//

void setup_pins_encoder()
{
    pinMode(MOTOR1_ENCODER_A, INPUT);
    pinMode(MOTOR1_ENCODER_B, INPUT);
    pinMode(MOTOR2_ENCODER_A, INPUT);
    pinMode(MOTOR2_ENCODER_B, INPUT);
    pinMode(MOTOR3_ENCODER_A, INPUT);
    pinMode(MOTOR3_ENCODER_B, INPUT);
    pinMode(MOTOR4_ENCODER_A, INPUT);
    pinMode(MOTOR4_ENCODER_B, INPUT);
    attachInterrupt(0, _interrupt_left1, FALLING);
    attachInterrupt(1, _interrupt_right2, FALLING);
    attachInterrupt(4, _interrupt_left3, FALLING);
    attachInterrupt(5, _interrupt_right4, FALLING);
  
}
void setup_pins_motor()
{
    pinMode(MOTOR1_PWM, OUTPUT); // plu1+
    pinMode(MOTOR1_IN_A, OUTPUT); // dir1+
    pinMode(MOTOR1_IN_B, OUTPUT); // ena1 +
    digitalWrite(MOTOR1_IN_B, LOW); 
    pinMode(MOTOR2_PWM, OUTPUT); // plu2+
    pinMode(MOTOR2_IN_A, OUTPUT); // dir2+
    pinMode(MOTOR2_IN_B, OUTPUT); // ena2 +
    digitalWrite(MOTOR2_IN_B, LOW); 
    pinMode(MOTOR3_PWM, OUTPUT); // plu3+
    pinMode(MOTOR3_IN_A, OUTPUT); // dir3+
    pinMode(MOTOR3_IN_B, OUTPUT); // ena3+
    digitalWrite(MOTOR3_IN_B, LOW); 
    pinMode(MOTOR4_PWM, OUTPUT); // plu4+
    pinMode(MOTOR4_IN_A, OUTPUT); // dir4+
    pinMode(MOTOR4_IN_B, OUTPUT); // ena4+
    digitalWrite(MOTOR4_IN_B, LOW); 
}
void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(cmd_sub);
    nh.subscribe(cmd_sub1);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    // setup_pin of encoder
    setup_pins_encoder();
    setup_pins_motor();
    // setup of hc-sr04
    nh.advertise(range_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{
   
   static unsigned long range_time = 0; 
  
   if ((millis() - range_time) >=(1000/100))
   {
      nh.loginfo("range_time");
      range.data = ping();
      range_pub.publish(&range);
      range_time = millis();
   }
    // hr-sr04
//     range.data = ping();
//     range_pub.publish(&range);
     
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static unsigned long timer2 = 0;
    static bool imu_is_initialized;
    
    if ((millis() - prev_control_time) >=(1000/COMMAND_RATE))
    {
       //nh.loginfo("moveBasemoveBase");
       moveBase();
       prev_control_time = millis();
      
    }

    if((millis() - g_prev_command_time) >= 400)
    //if (g_req_linear_vel_x==0)
    {
        nh.logfatal("stopBasestopBase");
        stopBase();
    }


     if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
     {
         // kiểm tra độ chính xác nếu IMU được kết nối
         if (!imu_is_initialized)
         {
             imu_is_initialized = initIMU();

             if(imu_is_initialized)
                 nh.loginfo("IMU Initialized");
             else
                 nh.logfatal("IMU failed to initialize. Check your IMU connection.");
         }
         else
         {
             publishIMU();
         }
         prev_imu_time = millis();
     }

  
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    nh.spinOnce();
    Serial.print("\n");
    Serial.print(" millis1() : ");
    Serial.print(millis());
    Serial.print("\n"); 
}


unsigned long i = 0;
unsigned long n = 0;
  
void commandCallback(const geometry_msgs::Twist& cmd_msg)
{   
    i++;
    n=0;
    if(i==1)
    {
        g_req_linear_vel_x = 0;
        g_req_linear_vel_y = 0;
        g_req_angular_vel_z = 0;
    }
    if((millis() - timer2) >= 100)
    {   
        nh.loginfo("commandCallback");
        g_req_linear_vel_x = cmd_msg.linear.x;
        g_req_linear_vel_y = cmd_msg.linear.y;
        g_req_angular_vel_z = cmd_msg.angular.z;
               
    }   
    g_prev_command_time = millis();
}

//float g_req_linear_vel_x1 = 0;
//float g_req_linear_vel_y1 = 0;
//float g_req_angular_vel_z1 = 0;
void commandCallback1(const geometry_msgs::Twist& cmd_msg)
{        
    n++;
    i=0;
    if(n==1)
    {
        g_req_linear_vel_x = 0;
        g_req_linear_vel_y = 0;
        g_req_angular_vel_z = 0;
    }  
    nh.loginfo("commandCallback1");
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;
    g_prev_command_time = millis();
    timer2 = millis();
}


	
float f=0;
float timer=0;
float f3=0;
float timer3=0;
float f4=0;
float timer4=0;
float f5=0;
float timer5=0;
float Ton=50;
float Toff=0;
float backup1 = 0;
float backup2 = 0;
float backup3 = 0;
float backup4 = 0;

void moveBase()
{
    Serial.print("\n");
    Serial.print("pulse_left1 " );
    Serial.print(pulse_left1  );
    Serial.print("\n");
    Serial.print("\n");
    Serial.print(" pulse_right2 : ");
    Serial.print(pulse_right2  );
    Serial.print("\n"); 
   
    Serial.print("\n");
    Serial.print(" pulse_left:");
    Serial.print(pulse_left3 );
    Serial.print("\n"); 
     Serial.print("\n");
    Serial.print("pulse_right4 " );
    Serial.print( pulse_right4 );
    Serial.print("\n");
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    
    Serial.print("\n");
    Serial.print("req_rpm.motor1req_rpm.motorconclusion:1req_rpm.motor1req_rpm.motor1  " );
    Serial.print(req_rpm.motor1);
    Serial.print("\n");
    int intpulse_left1 = 100; 
    int current_rpm1 = RPM_motor(pulse_left1);
    int current_rpm2 = RPM_motor(pulse_right2);
    int current_rpm3 = RPM_motor(pulse_left3);
    int current_rpm4 = RPM_motor(pulse_right4);
    
     init_timer1(req_rpm.motor1);
     init_timer3(req_rpm.motor2);
     init_timer4(req_rpm.motor3);
     init_timer5(req_rpm.motor4);

    
    Kinematics::velocities current_vel;

           // gui van toc tuyen tinh tai thoi diem hien tai cua xe cho odom xac dinh vi tri
    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
//     Serial.print("current_vel.linear_x: " );
//     Serial.print(current_vel.linear_x);
//     Serial.print("\n");
    raw_vel_pub.publish(&raw_vel_msg);
   
}
void init_timer1(float req_rpm1)
{
    if(req_rpm1 != backup1)
    {
      backup1 = req_rpm1;
      nh.loginfo("init_timer1");
      TCCR1A=0; TCCR1B=0;  // reset 2 thanh ghi
      DDRB |= (1 << PB5);  // output ra chan pin 11
    
      TCCR1A |= (1 << WGM11);   // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR1
      TCCR1B |= (1 << WGM12)|(1 << WGM13);
      TCCR1A |= (1 << COM1A1);   // đầu ra kiểu thường (none-inverting)
   
      f = (req_rpm1/(2*3.14))*1600; //6369
      if(f >0)
      {
        digitalWrite(MOTOR1_IN_A, HIGH);
      } 
      if(f<0)
      {
        digitalWrite(MOTOR1_IN_A, LOW);
       
      }  
      TCCR1B |= (1 << CS10);
      ICR1 = 16000000/abs(f);//5233
      timer = 1000000/abs(f);//327
      OCR1A = (Ton/timer)*ICR1;
    }
     
  
    //analogWrite(MOTOR1_PWM,OCR1A);
}
void init_timer3(float req_rpm2)
{
    
    if(req_rpm2 != backup2){
      backup2 = req_rpm2;
      nh.loginfo("init_timer3");
      TCCR3A=0; TCCR3B=0;  // reset 2 thanh ghi
      DDRE |= (1 << PE3);  // output ra chan pin 5
    
      TCCR3A |= (1 << WGM31);   // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR3
      TCCR3B |= (1 << WGM32)|(1 << WGM33);
      TCCR3A |= (1 << COM3A1);   // đầu ra kiểu thường (none-inverting)
    
    
      f3 = (req_rpm2/(2*3.14))*1600; //13757
      if(f3>0)
      {
        digitalWrite(MOTOR2_IN_A, HIGH);
      } 
      if(f3<0)
      {
        digitalWrite(MOTOR2_IN_A, LOW);
      }   
      ICR3 = 16000000/abs(f3);//1162
      timer3 = 1000000/abs(f3);//72
      OCR3A = (Ton/timer3)*ICR3;
      TCCR3B |= (1 << CS30);  
    }
   
}
void init_timer4(float req_rpm3)
{
    if(req_rpm3 != backup3)
    {
      backup3 = req_rpm3;
      nh.loginfo("init_timer4");
      TCCR4A=0; TCCR4B=0;  // reset 2 thanh ghi
      DDRH |= (1 << PH3);  // output ra chan pin 6
    
      TCCR4A |= (1 << WGM41);   // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR4
      TCCR4B |= (1 << WGM42)|(1 << WGM43);
      TCCR4A |= (1 << COM4A1);   // đầu ra kiểu thường (none-inverting)

      f4 = (req_rpm3/(2*3.14))*1600; //13757 
      if(f4>0)
      {
        digitalWrite(MOTOR3_IN_A, HIGH);
      } 
      if(f4<0)
      {
        digitalWrite(MOTOR3_IN_A, LOW);
      }   
      ICR4 = 16000000/abs(f4);//1162
      timer4 = 1000000/abs(f4);//72
      OCR4A = (Ton/timer4)*ICR4;
      TCCR4B |= (1 << CS40);
     
    }
}
void init_timer5(float req_rpm4)
{
    if(req_rpm4 != backup4)
    {
      backup4 = req_rpm4;
      nh.loginfo("init_timer5");
      TCCR5A=0; TCCR5B=0;  // reset 2 thanh ghi
      DDRL |= (1 << PL3);  // output ra chan pin 46
      
      TCCR5A |= (1 << WGM51)|(1 << COM5A1);   // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR5
      TCCR5B |= (1 << WGM52)|(1 << WGM53)|(1 << CS50);
      //TCCR5A |= (1 << COM5A1);   // đầu ra kiểu thường (none-inverting)
      f5 = (req_rpm4/(2*3.14))*1600; //1528
    
      if(f5>0)
      {
          digitalWrite(MOTOR4_IN_A, HIGH);
      } 
      if(f5<0)
      {
          digitalWrite(MOTOR4_IN_A, LOW);
      }   
 
      ICR5 = 16000000/abs(f5);//1162
      timer5 = 1000000/abs(f5);//72
      OCR5A = (Ton/timer5)*ICR5;
//      ICR5 = 65535;
//      OCR5A = 10000;
      //TCCR5B |= (1 << CS50);
    }

   
}

// hr-sr04
long ping()
{
    // Send out PING))) signal pulse
    pinMode(trigpin, OUTPUT);
    pinMode(echopin,INPUT);
    digitalWrite(trigpin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigpin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin, LOW);
    //Get duration it takes to receive echo
    duration = pulseIn(echopin, HIGH);
    //Convert duration into distance
    return duration /29/2;
}
unsigned long prev_update_time_;
long prev_encoder_ticks_;
int RPM_motor(long pluse){
   
    int counts_per_rev_ = COUNTS_PER_REV;
   
    long encoder_ticks = pluse;
    unsigned long current_time = millis();
	unsigned long dt = current_time - prev_update_time_;

	//chuyển đổi thời gian từ mili giây sang giay
	double dtm = (double)dt / 1000;
	double delta_ticks = encoder_ticks - prev_encoder_ticks_;

	

	prev_update_time_ = current_time;
	prev_encoder_ticks_ = encoder_ticks;
	// Serial.print("delta_ticks: " );
    // Serial.print((delta_ticks / counts_per_rev_) / dtm);
    // Serial.print("\n");	
	return ((2*3.14*delta_ticks) / counts_per_rev_) / dtm;  // la W = 2*3.14*f(f = (delta_ticks / counts_per_rev_) / dtm)
}
void stopBase()
{   
    unsigned long current_time1 = millis();
    Serial.print("current_time1: " );
    Serial.print(current_time1);
    Serial.print("\n");	
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishIMU()
{
    
    raw_imu_msg.linear_acceleration = readAccelerometer();

    raw_imu_msg.angular_velocity = readGyroscope();

    raw_imu_msg.magnetic_field = readMagnetometer();

    raw_imu_pub.publish(&raw_imu_msg);
}

void printDebug()
{
    
    // sprintf("Encoder Motor_left1 : %ld",pulse_left1);
    // sprintf("Encoder Motor_Right2 : %ld",pulse_right2);
    // sprintf("Encoder Motor_left3 : %ld",pulse_left3);
    // sprintf("Encoder Motor_Right4 : %ld",pulse_right4);
    //Serial.println(pulse_left1);
    // Serial.println(pulse_right2);
    // Serial.println(pulse_left3);
    // Serial.println(pulse_right4);
}
