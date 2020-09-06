#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/String.h>
#include <ros/time.h>

// Define stepper motor connections and steps per revolution:
#define dirPin1 2
#define stepPin1 3
#define dirPin2 4
#define stepPin2 5
#define dirPin3 6
#define stepPin3 7
#define dirPin4 8
#define stepPin4 9
#define steps1 1500 // total stepper motor steps for robot forward and backward movement
#define steps2 500 // total stepper motor steps for robot rotation
float x;
float z;

class ROBOT
{

  public:

    void goForward()
    {
      // Set the spinning direction clockwise:
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, HIGH);
      digitalWrite(dirPin3, HIGH);
      digitalWrite(dirPin4, HIGH);
    
    
      // Spin the stepper motor 1 revolution slowly:
      for (int i=0; i < steps1; i++)
      {      
        digitalWrite(stepPin1, HIGH);
        digitalWrite(stepPin2, HIGH);
        digitalWrite(stepPin3, HIGH);
        digitalWrite(stepPin4, HIGH);
        delayMicroseconds(100);
        digitalWrite(stepPin1, LOW);
        digitalWrite(stepPin2, LOW);
        digitalWrite(stepPin3, LOW);
        digitalWrite(stepPin4, LOW);
        delayMicroseconds(100);
      }      
    }
  
    void goBackward()
    {
      // Set the spinning direction counterclockwise:
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, LOW);
      digitalWrite(dirPin3, LOW);
      digitalWrite(dirPin4, LOW);
    
      // Spin the stepper motor 1 revolution quickly:
      for (int i=0; i < steps1; i++)
      { 
        digitalWrite(stepPin1, HIGH);
        digitalWrite(stepPin2, HIGH);
        digitalWrite(stepPin3, HIGH);
        digitalWrite(stepPin4, HIGH);
        delayMicroseconds(100);
        digitalWrite(stepPin1, LOW);
        digitalWrite(stepPin2, LOW);
        digitalWrite(stepPin3, LOW);
        digitalWrite(stepPin4, LOW);
        delayMicroseconds(100);
      }      
    }
  
    void rotateClockwise()
    {
      // Set the spinning direction clockwise:
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, HIGH);
      digitalWrite(dirPin3, LOW);
      digitalWrite(dirPin4, HIGH);
    
      // Spin the stepper motor 1 revolution quickly:
      for (int i=0; i < steps2; i++)
      { 
        digitalWrite(stepPin1, HIGH);
        digitalWrite(stepPin2, HIGH);
        digitalWrite(stepPin3, HIGH);
        digitalWrite(stepPin4, HIGH);
        delayMicroseconds(100);
        digitalWrite(stepPin1, LOW);
        digitalWrite(stepPin2, LOW);
        digitalWrite(stepPin3, LOW);
        digitalWrite(stepPin4, LOW);
        delayMicroseconds(100);
      }      
    }
  
    void rotateCounterClockwise()
    {
      // Set the spinning direction counterclockwise:
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      digitalWrite(dirPin3, HIGH);
      digitalWrite(dirPin4, LOW);
    
    
      // Spin the stepper motor 1 revolution slowly:
      for (int i=0; i < steps2; i++)
      { 
        digitalWrite(stepPin1, HIGH);
        digitalWrite(stepPin2, HIGH);
        digitalWrite(stepPin3, HIGH);
        digitalWrite(stepPin4, HIGH);
        delayMicroseconds(100);
        digitalWrite(stepPin1, LOW);
        digitalWrite(stepPin2, LOW);
        digitalWrite(stepPin3, LOW);
        digitalWrite(stepPin4, LOW);
        delayMicroseconds(100);
      }      
    }
    // Turn off motors
    void die()
    {
        digitalWrite(stepPin1, LOW);
        digitalWrite(stepPin2, LOW);
        digitalWrite(stepPin3, LOW);
        digitalWrite(stepPin4, LOW);
    }
  
};

ros::NodeHandle nh;

void velCallBack(const geometry_msgs::Twist& vel)
{
  x = vel.linear.x;
  z = vel.angular.z;

  ROBOT robot;
  
  if (x > 0 && z == 0)
  {
      robot.goForward();
  }
  else if (x > 0 && z > 0 )
  {
    robot.rotateCounterClockwise();
  }
  else if (x > 0 && z < 0 )
  {
    robot.rotateClockwise();
  }
  else if (x < 0)
  {
    robot.goBackward();
  }
  else
  {
    robot.die();
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallBack);

void setup() {
//  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  // Declare pins as output:
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
}

void loop() {  
  nh.spinOnce();
  delay(10);
}
