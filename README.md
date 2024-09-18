# 헤네스 브룬 T870 아두이노 제어 코드 설명

**파일 설명**  
헤네스 브룬 T870 모델을 개조하여 Arduino Mega 2560을 사용해 차량을 제어하고, 이를 Amap 모터 드라이브와 함께 사용할 수 있도록 만든 제어 코드와 관련된 것입니다. 기본적으로, Arduino Mega 2560은 차량의 모터 및 센서를 제어하는 데 사용되며, Amap 모터 드라이브는 차량의 구동 모터에 전력을 공급하여 동작을 제어하는 역할을 합니다.

이 코드는 **아두이노 Mega2560**과 **AMAP 모터 드라이브**를 사용하여 **헤네스 브룬 T870** 차량을 제어하기 위한 코드입니다. 이 코드는 모터 제어, 엔코더 값을 읽고 PID 제어를 수행하여 차량의 속도와 조향 각도를 제어하는 기능을 포함하고 있습니다. **ROS** 노드와 통신하여 `cmd_vel` 명령을 수신하고, 엔코더 및 조향각 데이터를 퍼블리시하는 기능도 포함되어 있습니다.

## 주요 라이브러리
- `MsTimer2.h`: 타이머 인터럽트 사용을 위한 라이브러리
- `ros.h`: ROS와의 통신을 위한 라이브러리
- `geometry_msgs/Twist.h`: ROS에서 속도 명령을 처리하기 위한 메시지 타입
- `std_msgs/Int32.h`, `std_msgs/Float32.h`: ROS에서 엔코더 값과 조향각 값을 퍼블리시하기 위한 메시지 타입

## 상수 정의
- `m_2_pulse`: 1미터 당 펄스 수
- `pulse_2_m`: 펄스 당 미터 수
- `vel_2_pulse`: 속도에서 펄스로 변환하는 상수
- `No_Calibration_Point`: 보정 데이터 포인트 개수

## ROS 메시지 처리
- `cmd_vel`: 속도 명령을 저장하기 위한 변수
- `encoder_data1`: 엔코더 값을 저장하기 위한 변수
- `steering_angle_data`: 조향각 값을 저장하기 위한 변수

### ROS 콜백 함수
- **cmd_vel_callback**: ROS에서 속도와 조향 명령을 수신하여 변수를 업데이트합니다.

## 보정 데이터
- 차량의 조향각과 센서 값을 보정하기 위한 데이터 구조체 `CalibrationData`를 정의하고, `linear_mapping` 함수를 통해 보정된 조향각을 계산합니다.

## 모터 제어
- **front_motor_control**: 앞쪽 모터 제어 (전진, 후진, 정지)
- **rear_motor_control**: 뒤쪽 모터 제어 (전진, 후진, 정지)
- **motor_control**: 앞뒤 모터를 동시에 제어하는 함수

## 엔코더 설정 및 읽기
- `initEncoders`: SPI 통신을 통해 엔코더를 초기화
- `readEncoder`: 엔코더 값을 읽는 함수
- `clearEncoderCount`: 엔코더 값을 초기화하는 함수

## PID 제어
- **motor_PID_control**: 모터 속도를 제어하는 PID 알고리즘을 적용하여 엔코더 값을 바탕으로 모터 속도를 조정합니다.
- **steer_motor_control**: 조향 모터를 제어하는 함수
- **PID_Control**: 조향각에 대해 PID 제어를 수행하는 함수

## 제어 함수
- **control_callback**: 50ms마다 실행되는 제어 함수로, 모터와 조향각 제어를 수행하고 ROS 메시지를 퍼블리시합니다.
- **steering_control**: 주어진 목표 조향각에 따라 PID 제어를 사용하여 차량의 조향각을 제어합니다.

## ROS 통신
- `cmd_vel_callback`: ROS에서 수신한 `cmd_vel` 명령을 처리하여 속도와 조향 각도를 제어합니다.
- ROS 노드에서 엔코더 값과 조향각을 퍼블리시하는 퍼블리셔를 설정하고, `MsTimer2`를 사용하여 주기적으로 제어 로직을 실행합니다.

## setup() 함수
- 엔코더 초기화, 모터 제어 핀 설정, ROS 노드 초기화, 타이머 설정 등을 수행합니다.

## loop() 함수
- ROS 메시지를 처리하고 노드 상태를 유지합니다. 또한 CPU 점유율을 줄이기 위해 작은 딜레이를 추가합니다.
