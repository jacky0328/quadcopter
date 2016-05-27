/*
  ------------------
   Quadcopter Pilot
  ------------------
  author : vincent jaunet
  mail : vincent.jaunet(AT)hotmail(DOT)fr

  - Initialization of PID
  - Initialization of ESC
  - Initialization of MPU6050

  Copyright (c) <2014> <Vincent Jaunet>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <signal.h>
#include "main.h"
#include "HMC5883L.h"
using namespace std;

char filename [100];

void stop_motors(int s){
  printf("Caught signal %d\n",s);

  uint8_t checksum=0;
  ArduSPI.writeByte('S');
  for (int iesc=0;iesc < 8; iesc++) {
    ArduSPI.writeByte((uint8_t) 0);
  }
  ArduSPI.writeByte(checksum);
  //sending end of transaction
  ArduSPI.writeByte('P');

  exit(0);
}

// yprSTAB[PITCH].set_Kpid(0.6,0.000,0.04);
// yprSTAB[ROLL].set_Kpid(0.6,0.000,0.04);

//   kp  ->  3.0 ....  can up  

//   roll
//yprSTAB[2].set_Kpid(2.8, 0.8, 0.6);  //golden
void Set_default_PID_config(){
  //manual initialization of PID constants
  yprRATE[YAW].set_Kpid(6.0, 0.0, 0.0);
  yprSTAB[2].set_Kpid(2.8, 0.8, 0.6);  //golden
  yprSTAB[1].set_Kpid(2.8, 0.8, 0.6);  //golden
  /*
  for (int i=1;i<3;i++){
    //yprSTAB[i].set_Kpid(2.0, 0.001, 0.1);
    //yprRATE[i].set_Kpid(2.0, 0.0, 0.0);
    //yprSTAB[i].set_Kpid(2.0, 0.000, 0.21);
    //yprSTAB[i].set_Kpid(2.2, 0.0004, 0.21);
    //yprSTAB[i].set_Kpid(2.8, 0.5, 0.6);  //golden
    //yprSTAB[i].set_Kpid(4.0, 0.0004, 0.1);
    yprSTAB[i].set_Kpid(2.8, 0.8, 0.6);  //golden
    //yprSTAB[i].set_Kpid(3.0, 0.5, 0.6);  //golden
    //yprRATE[i].set_Kpid(1.07, 0.0, 0.0);
  }
  */
}

void Blink_led(int N){
  // use gpio to blink an led on pin 
  //pinMode (0, OUTPUT) ;  
  for(int i=0;i<N;i++){
  system("/usr/local/bin/gpio write 0 0");
  //digitalWrite (7, LOW) ;
  usleep(50000);
  system("/usr/local/bin/gpio write 0 1");
  //digitalWrite (7, HIGH) ;
  usleep(50000);
  }
}

void Beep(int N){
  // use gpio to blink an led on pin 0
  for(int i=0;i<N;i++){
  //system("sudo /home/pi/QUADCOPTER_V2/RPI/BEEP/beep");
  system("sudo /home/pi/c_code/QUADCOPTER_V2-master/RPI/BEEP/beep");
  usleep(160000);
  }
}

//-------------------------------------
//--------- Main-----------------------

int main(int argc, char *argv[])
{
  printf("QuadCopter Pilot v2.0\n");
  printf("----------------------\n");
  printf("\n");

  //system("rm quadpilot.log");

   time_t now = time(0);
   tm *ltm = localtime(&now);

//   char filename [100];
    sprintf(filename,"/home/pi/c_code/QUADCOPTER_V2-master/RPI/PILOT/log/quad_pilot_%02d_%02d_%02d.log",
               ltm->tm_mday,ltm->tm_hour,ltm->tm_min);

//    printf("file_name : %s\n",filename); 
//   logfile.open(filename, ios::out|ios::app);


  wiringPiSetup () ;
  pinMode (0, OUTPUT) ;  
  system("/usr/local/bin/gpio write 0 0");

  //handling of CTRL+C input
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = stop_motors;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  //setting up IMU
  imu.set_com();
  imu.initialize();
  HMC5883L.initialise();



  //setting up SPI
  ArduSPI.initialize();

  //Set PID config
  if (argc == 8) {
    printf("Setting PID constants\n");
    printf("Rate PID :\n");
    printf("    Kp = %f, Ki = %f, Kd = %f\n",
	   atof(argv[1]),atof(argv[2]),atof(argv[3]));
    printf("Rate PID :\n");
    printf("    Kp = %f, Ki = %f, Kd = %f\n",
	   atof(argv[4]),atof(argv[5]),atof(argv[6]));
      for (int i=1;i<3;i++){
	yprRATE[i].set_Kpid(atof(argv[1]),atof(argv[2]),atof(argv[3]));
	yprSTAB[i].set_Kpid(atof(argv[4]),atof(argv[5]),atof(argv[6]));
      }
    printf("Yaw Rate PID :\n");
    printf("    Kp = %f\n",atof(argv[7]));
    yprRATE[YAW].set_Kpid(atof(argv[7]), 0.0, 0.0);

  }else {
    printf("Setting default PID constants\n");
    Set_default_PID_config();
  }

  //Say I am ready
  Beep(3);
  printf("Blink led\n");
  Blink_led(10);

  //Starting Timer
  Timer.start();

  /* Sleeping everything is done via
     software interrupts  */
  while (true){

    usleep(20000);

  }//end

  return 0;
}


//
