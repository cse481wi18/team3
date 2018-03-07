#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher pressure("pressure", &str_msg); // Topic name


const int sensor = A0;

//print "test";
void setup() {
//  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pressure);
  pinMode (sensor, INPUT);
  delay(1000); //delay before next reading:
//  Serial.begin(9600); // initialize the serial communications
}

void loop() {
  int fsr = analogRead(sensor); //reads the sensor data
  //Serial.print ("test2");
//  Serial.println(fsr);

  if (fsr == 0)
    str_msg.data = "cancel"; //or 0
  else if (fsr > 1000)
    str_msg.data = "cancel"; //or 0
  else
    str_msg.data = "start"; //or 1
//  Serial.print (str_msg.data);
  int str = 0;
  if (fsr == 0)
    str = 0; //or 0
  else if (fsr > 1000)
    str = 0; //or 0
  else
    str = 1; //or 1
//  Serial.println (str);
  pressure.publish( &str_msg);
  nh.spinOnce();
  delay(1000); //delay before next reading:
}

