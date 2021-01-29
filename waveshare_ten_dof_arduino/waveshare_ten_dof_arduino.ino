#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>

// #include <ros.h>
// #include <sensor_msgs/Imu.h>

// ros::NodeHandle nh;

// sensor_msgs::Imu imu_msg;
// ros::Publisher imu_pub("imu", &imu_msg);

#define RAW_LINEAR_ACCELERATION 1

// gravitational acceleration constant
static const double G_TO_MPSS = 9.80665;

RTIMU *imu;             // the IMU object
RTFusionRTQF fusion;    // the fusion object
RTIMUSettings settings; // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL 100 // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define SERIAL_PORT_SPEED 57600

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
RTQuaternion gravity;

// unsigned long frame_id;

void setup()
{
  int errcode;
  // Ros specifics first
  // nh.initNode();
  // nh.advertise(imu_pub);
  // imu_msg.header.frame_id = "/imu";
  // Other stuff later

  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings); // create the imu object

  Serial.print("ArduinoIMU starting using device ");
  Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0)
  {
    Serial.print("Failed to init IMU: ");
    Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  gravity.setScalar(0);
  gravity.setX(0);
  gravity.setY(0);
  gravity.setZ(1);
}

void loop()
{
  unsigned long now = millis();
  unsigned long delta;
  RTVector3 realAccel;
  RTVector3 rawAccel;
  RTVector3 rollPitchYaw;
  RTVector3 gyroData;
  RTQuaternion rotatedGravity;
  RTQuaternion fusedConjugate;
  RTQuaternion qTemp;
  int loopCount = 0;

  while (imu->IMURead())
  { // get the latest data if ready yet
    // this flushes remaining data in case we are falling behind
    if (++loopCount >= 10)
      continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());

    //  do gravity rotation and subtraction

    // create the conjugate of the pose

    fusedConjugate = fusion.getFusionQPose().conjugate();
    gyroData = imu->getGyro();
    rawAccel = imu->getAccel();

    // now do the rotation - takes two steps with qTemp as the intermediate variable

    qTemp = gravity * fusion.getFusionQPose();
    rotatedGravity = fusedConjugate * qTemp;

    // now adjust the measured accel and change the signs to make sense

    realAccel.setX(-(imu->getAccel().x() - rotatedGravity.x()));
    realAccel.setY(-(imu->getAccel().y() - rotatedGravity.y()));
    realAccel.setZ(-(imu->getAccel().z() - rotatedGravity.z()));

    sampleCount++;
    if ((delta = now - lastRate) >= 1000)
    {
      //      Serial.print("Sample rate: "); Serial.print(sampleCount);
      if (!imu->IMUGyroBiasValid())
        //        Serial.println(", calculating gyro bias");
        delay(0);
      else
//        Serial.println();
        delay(0);

      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL)
    {
      float value;
      lastDisplay = now;

      // Quaternion -- imu_msg.orientation
      value = fusion.getFusionQPose().x();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");

      value = fusion.getFusionQPose().y();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");

      value = fusion.getFusionQPose().z();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");

      
      value = fusion.getFusionQPose().scalar();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");

      // Gyro data -- imu_msgs.angular_velocity
      value = gyroData.x();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");
      value = gyroData.y();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");
      value = gyroData.z();
      if(value == -0.00) value = 0.00;
      Serial.print(value);
      Serial.print("\t");

      // Linear Acceleration data -- imu_msgs.linear_acceleration

      if (RAW_LINEAR_ACCELERATION)
      {
        value = rawAccel.x();
        if(value == -0.00) value = 0.00;
        Serial.print(value * G_TO_MPSS);
        Serial.print("\t");
        value = rawAccel.y();
        if(value == -0.00) value = 0.00;
        Serial.print(value * G_TO_MPSS);
        Serial.print("\t");
        value = rawAccel.z();
        if(value == -0.00) value = 0.00;
        Serial.print(value * G_TO_MPSS);
        Serial.print("\t");
      }
      else
      {
        value = realAccel.x();
        if(value == -0.00) value = 0.00;
        Serial.print(value);
        Serial.print("\t");
        value = realAccel.y();
        if(value == -0.00) value = 0.00;
        Serial.print(value);
        Serial.print("\t");
        value = realAccel.z();
        if(value == -0.00) value = 0.00;
        Serial.print(value);
        Serial.print("\t");
      }
      Serial.println();


      // imu_msg.header.stamp = nh.now();

      // imu_msg.orientation.x = fusion.getFusionQPose().x();
      // imu_msg.orientation.y = fusion.getFusionQPose().y();
      // imu_msg.orientation.z = fusion.getFusionQPose().z();
      // imu_msg.orientation.w = fusion.getFusionQPose().scalar();

      // imu_msg.angular_velocity.x = gyroData.x();
      // imu_msg.angular_velocity.y = gyroData.y();
      // imu_msg.angular_velocity.z = gyroData.z();

      // if (RAW_LINEAR_ACCELERATION)
      // {
      //   imu_msg.linear_acceleration.x = rawAccel.x() * G_TO_MPSS;
      //   imu_msg.linear_acceleration.y = rawAccel.y() * G_TO_MPSS;
      //   imu_msg.linear_acceleration.z = rawAccel.z() * G_TO_MPSS;
      // }
      // else
      // {
      //   imu_msg.linear_acceleration.x = realAccel.x() * G_TO_MPSS;
      //   imu_msg.linear_acceleration.y = realAccel.y() * G_TO_MPSS;
      //   imu_msg.linear_acceleration.z = realAccel.z() * G_TO_MPSS;
      // }
      // imu_pub.publish(&imu_msg);
      // nh.spinOnce();
      // delay(400);
    }
  }
}
