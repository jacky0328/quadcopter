# quadcopter
Reference on https://github.com/vjaunet/QUADCOPTER_V2

hardware circuit:

Wiring

10 DOU (old MPU6050) -> RPI : -VDD -> 3.3V -GND -> GND -SDA -> SDA -SCL -> SCL -VIO -> 3.3V

Arduino -> RPI : -VCC -> 5V from external Regulator -GND -> GND -MISO -> Logic Level converter ->MISO -MOSI -> Logic Level converter ->MOSI -SCK -> Logic Level converter ->SCK

ESCs and RC Receiver on Arduino: -Using PinChange interrupts -Check in the arduino code

RC receive and arduino connection
THROTTLE_IN_PIN  D2
YAW_IN_PIN D3
PITCH_IN_PIN D4
ROLL_IN_PIN D5
SW1_IN_PIN  A1  //   15 -> A1
SW2_IN_PIN  A2  //   16 -> A2 
SW3_IN_PIN  A3  //   16 -> A3 

ESC and arduino connection

FL_MOTOR_OUT_PIN D6
FR_MOTOR_OUT_PIN D7
BL_MOTOR_OUT_PIN D8
BR_MOTOR_OUT_PIN D9



The quadcopter on RPI would generate log file..... I write a java tool to parse it 


