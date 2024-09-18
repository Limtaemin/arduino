#include <MsTimer2.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define m_2_pulse     353             // 1m 당 pulse 수 확인 해야 함
#define pulse_2_m     1./353.         // pulse 당 m 확인 해야 함
#define vel_2_pulse   m_2_pulse/20
#define No_Calibration_Point 14

geometry_msgs::Twist cmd_vel;   // 모터 명령 수신을 위한 변수(수신)
std_msgs::Int32 encoder_data1;  // 모터 엔코더1 값 전달을 위한 변수(송신)
std_msgs::Float32 steering_angle_data;  // 조향각 값 전달을 위한 변수(송신)

void cmd_vel_callback(const geometry_msgs::Twist& msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel_angle", cmd_vel_callback);
ros::Publisher encoder_pub1("encoder1", &encoder_data1);
ros::Publisher steering_angle_pub("steering_angle", &steering_angle_data);  // 조향각 퍼블리셔 추가

struct CalibrationData {
  double X[No_Calibration_Point];
  double Y[No_Calibration_Point];
} cal_data = {
  {0, 60, 100, 200, 300, 400, 500, 600, 700, 800, 900, 960, 1000, 1023},
  {-22.08, -19.42, -17.2, -12.66, -8.08, -4.53, -0.17, 4.37, 8.53, 13.12, 16.9, 20.75, 22.83, 23.92}
};

double linear_mapping(double x) {
  if (x <= cal_data.X[0]) {
    return cal_data.Y[0];
  } else if (x >= cal_data.X[No_Calibration_Point - 1]) {
    return cal_data.Y[No_Calibration_Point - 1];
  }

  for (int i = 0; i < No_Calibration_Point - 1; i++) {
    if (x >= cal_data.X[i] && x < cal_data.X[i + 1]) {
      double x1 = cal_data.X[i];
      double x2 = cal_data.X[i + 1];
      double y1 = cal_data.Y[i];
      double y2 = cal_data.Y[i + 1];
      return y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
    }
  }
  return 0;
}

// Front Motor Drive
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7
int f_speed = 0, r_speed = 0;
int front_motor_pwm = 0;
int rear_motor_pwm = 0;
int velocity = 0;
float steer_angle = 0;

void front_motor_control(int motor1_pwm) {
  if (motor1_pwm > 0) {  // forward
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  } else if (motor1_pwm < 0) {  // backward
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  } else {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}

void rear_motor_control(int motor2_pwm) {
  if (motor2_pwm > 0) {  // forward
    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor2_pwm);
  } else if (motor2_pwm < 0) {  // backward
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor2_pwm);
  } else {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}

void motor_control(int front_speed, int rear_speed) {
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);  
}

#include <SPI.h>
#define ENC1_ADD 22
signed long encoder1count = 0;
signed long encoder1_error = 0;
signed long encoder1_error_d = 0;
signed long encoder1_target = 0;
signed long encoder1_error_old = 0; 
signed long encoder1_error_sum = 0; 

float target_velocity1 = 0.0;
float target_velocity2 = 0.0;

void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH);
  SPI.begin();
  digitalWrite(ENC1_ADD, LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                 // Write to MDR0
  SPI.transfer(0x03);                 // Configure to 4 byte mode
  digitalWrite(ENC1_ADD, HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) {  
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);  // Begin SPI conversation
  SPI.transfer(0x60);                            // Request count
  count_1 = SPI.transfer(0x00);                  // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);                  // Read lowest order byte
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH); // Terminate SPI conversation 
  count_value = ((long)count_1 << 24) + ((long)count_2 << 16) + ((long)count_3 << 8) + (long)count_4;
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);  // Begin SPI conversation  
  SPI.transfer(0x98);
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH); // Terminate SPI conversation 
  delayMicroseconds(100);  
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);  // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH); // Terminate SPI conversation 
}

///////////////////////////////////////  Motor PID 제어 /////////////////////////////////////////////
float Kp_motor = 0.7;
float Kd_motor = 4.1;
float Ki_motor = 0.0001;

void motor_PID_control(void) {
  encoder1count = (-1) * readEncoder(1);
  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_sum += encoder1_error;
  encoder1_error_d = encoder1_error - encoder1_error_old;
  encoder1_error_sum = (encoder1_error_sum >= 100) ? 100 : encoder1_error_sum;
  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;
  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  front_motor_pwm = (front_motor_pwm >= 255) ? 255 : front_motor_pwm;
  front_motor_pwm = (front_motor_pwm <= -255) ? -255 : front_motor_pwm;
  rear_motor_pwm = front_motor_pwm;
  if (fabs(encoder1_error) <= 2) {
    encoder1_error_sum = 0;
    clearEncoderCount(1);     
    encoder1_target = 0;
  } else {
    front_motor_control(front_motor_pwm);
    rear_motor_control(rear_motor_pwm);
  }  
  encoder1_error_old = encoder1_error; 
}

///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////
#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define LEFT_STEER_ANGLE  23.92
#define RIGHT_STEER_ANGLE -22.08
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

float Kp = 9.0;
float Ki = 1.5;
float Kd = 2.0;  // PID 상수 설정
double Setpoint, Input, Output;  // PID 제어 변수
double error, error_old;
double error_s, error_d;
int pwm_output;
int sensorValue = 0;  // value read from the pot
float Steer_Angle_Measure = 0;  // value output to the PWM (analog out)
float NEURAL_ANGLE = 1.38;
float Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int motor_pwm) {
  if (motor_pwm > 0) {  // 전진
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  } else if (motor_pwm < 0) {  // 후진
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  } else {  // 정지
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void PID_Control() {
  error = Steering_Angle - Steer_Angle_Measure;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >= 100) ? 100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  pwm_output = int(Kp * error + Kd * error_d + Ki * error_s);
  pwm_output = (pwm_output >= 255) ? 255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(error) <= 0.2) {  // 특정 값 이하면 제어를 멈추어서 조정이 안되도록
    steer_motor_control(0);
    error_s = 0;
  } else {
    steer_motor_control(pwm_output);
  }
  error_old = error;  
}

void steering_control() {
  if (Steering_Angle >= LEFT_STEER_ANGLE + NEURAL_ANGLE) {
    Steering_Angle = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  }
  if (Steering_Angle <= RIGHT_STEER_ANGLE + NEURAL_ANGLE) {
    Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  }
  PID_Control(); 
}

void control_callback() {
  static boolean output = HIGH;
  
  digitalWrite(13, output);
  output = !output;

  encoder1_target += target_velocity1 * vel_2_pulse;
  motor_PID_control();

  sensorValue = analogRead(Steering_Sensor);
  Steer_Angle_Measure = linear_mapping((double)sensorValue);

  if (Steer_Angle_Measure < RIGHT_STEER_ANGLE) {
    Steer_Angle_Measure = RIGHT_STEER_ANGLE;
  } else if (Steer_Angle_Measure > LEFT_STEER_ANGLE) {
    Steer_Angle_Measure = LEFT_STEER_ANGLE;
  }
  
  Steering_Angle = NEURAL_ANGLE + steer_angle;
  steering_control();

  // 엔코더 값을 주기적으로 퍼블리시
  encoder_data1.data = encoder1count;
  encoder_pub1.publish(&encoder_data1);
  
  // 조향각 값을 퍼블리시
  steering_angle_data.data = Steer_Angle_Measure+NEURAL_ANGLE;
  steering_angle_pub.publish(&steering_angle_data);
}


void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  target_velocity1 = (float)msg.linear.x;
  steer_angle = (float)msg.angular.z;
}

void setup() {
  error = error_s = error_d = error_old = 0.0;
  pwm_output = 0;
  
  initEncoders();  
  clearEncoderCount(1);

  pinMode(13, OUTPUT);

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR1_ENB, OUTPUT);

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);

  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);
  pinMode(MOTOR3_ENB, OUTPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(encoder_pub1);
  nh.advertise(steering_angle_pub);  // 조향각 퍼블리셔 등록
  
  MsTimer2::set(50, control_callback);
  MsTimer2::start();
}

void loop() {
  // ROS 메시지를 처리하고 노드 상태 유지
  nh.spinOnce();

  // 필요에 따라 짧은 딜레이를 넣어 CPU 점유율을 줄일 수 있음
  delay(10);
}
