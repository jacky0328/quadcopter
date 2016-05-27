/*
 * File:   Timer.cpp
 * Author: matt
 *
 * Created on 05 November 2012, 10:19
 * Modified 01-2014, vincent jaunet
 *
 * The timer class realize an action on a specific
 * clock signal. The clock oscillates at period PERIOD.
 * it uses a timer
 *
 * The action to be realized is specified in the function
 * sig_handler.
 *
 */

#include "timer.h"
#include <fstream>
#include <string.h>
#include <iomanip>
using namespace std;
extern char filename[100];

//             104839
#define PERIOD 500000
//#define PERIOD 5000

//#define N_RC_CHAN 4
#define N_RC_CHAN 7
#define N_SERVO 4

#define K_YAW 260
#define K_PITCH 120
#define K_ROLL 120

#define RC_MIN 1000
//#define RC_MAX 2000
#define RC_MAX 1960
#define THR_MIN 890
#define THR_MAX 1900

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3

/*Defines which PID control to use*/
#define PID_STAB
//#define PID_RATE
#define XMODE

#define FILE_DUMP
#define PRINT_DUMP
TimerClass Timer;
pthread_mutex_t TimerMutex_;


 s_rawData rawData_;
s_calibratedData calibratedData;


//Values calculated from matlab script MgnCalibration
const double magZeroX = 0.0576;
const double magZeroY = -0.0929;
const double magZeroZ = -0.0092;
const double magEllipsoid00_ = 1.8925;
const double magEllipsoid01_ = 0.0399;
const double magEllipsoid02_ = 0.0132;
const double magEllipsoid11_ = 1.8375;
const double magEllipsoid12_ = 0.0474;
const double magEllipsoid22_ = 2.1528;






TimerClass::TimerClass()
{
  /* Intialize sigaction struct */
   signalAction.sa_handler = &sig_handler_;

   /* Connect a signal handler routine to the SIGALRM signal */
   sigaction(SIGALRM, &signalAction, NULL);

   /* Allocate a timer */
   timer_create(CLOCK_REALTIME, NULL, &timerId);

   started = false;

 }

 TimerClass::TimerClass(const TimerClass& orig)
 {
 }

 TimerClass::~TimerClass()
 {
 }

void TimerClass::start()
{
  timeValue_.tv_sec = 0;
  timeValue_.tv_nsec = PERIOD;
  timeToSet_.it_value = timeValue_;
  timer_settime(timerId, 0, &timeToSet_, NULL);
  started = true;
}

void TimerClass::stop()
{
  timeValue_.tv_sec = 0;
  timeValue_.tv_nsec = 0;
  timeToSet_.it_value = timeValue_;
  timer_settime(timerId, 0, &timeToSet_, NULL);
  started = false;
}

inline void TimerClass::calcdt_()
{
  oldtime_ = time_;
  clock_gettime(CLOCK_MONOTONIC, &time_);
  Timer.dt = ((static_cast <int64_t>(time_.tv_sec) * 1000000000 +
	       static_cast <int64_t>(time_.tv_nsec)) -
	      (static_cast <int64_t>(oldtime_.tv_sec) * 1000000000 +
	       static_cast <int64_t>(oldtime_.tv_nsec))) / 1000000000.0;
}

inline void TimerClass::compensate_()
{
  //Timer aims to get as close to 400Hz as possible, mostly limited by the I2C
  //bandwidth
  clock_gettime(CLOCK_MONOTONIC, &iterationtime_);
  long inttime = PERIOD - ((iterationtime_.tv_sec * 1000000000 +
			    iterationtime_.tv_nsec) - (time_.tv_sec * 1000000000
						       + time_.tv_nsec));

  //printf("compsenate time : %ld\n",inttime);
  
  if (inttime < 0)
    Timer.timeValue_.tv_nsec = 1;
  else
    Timer.timeValue_.tv_nsec = inttime;
  Timer.timeToSet_.it_value = Timer.timeValue_;
  timer_settime(timerId, 0, &timeToSet_, NULL);
}

template <class T, void(T::*member_function)()>
void* thunk(void* p)
{
  (static_cast<T*> (p)->*member_function)();
  return 0;
}


static  int  sw1_prev = 0;   
static  int  sw2_prev = 0;   
static  int  flag_prev =0;




void TimerClass::dynamic_turning_pid(int RC_SW1, int RC_SW2, int RC_SW3, int roll)
{
   int sw1,sw2,sw1_rising,sw2_rising, flag, flag_rising, flag_falling;

  if(RC_SW1 > 1500)
     sw1 = 1;
  else
     sw1 = 0;

  if(RC_SW2 > 1500)
     sw2 = 1;
  else
     sw2 = 0;

  if(RC_SW3 > 1600)
     flag = 1;
  else
     flag = 0;    

  flag_rising  = (flag == 1) & (flag_prev == 0); 


   sw1_rising  = (sw1 ==1) & (sw1_prev==0);
   sw2_rising  = (sw2 ==1) & (sw2_prev==0);

   float ki_val =   yprSTAB[1].get_ki_coeff();
   float kp_val =   yprSTAB[1].get_kp_coeff();
   float kd_val =   yprSTAB[1].get_kd_coeff();

   if(sw1_rising)
      kd_val +=  0.05;

   if(sw2_rising)
      kd_val -=  0.05;

   //if(flag_rising)
   //   kp_val +=  0.05;

   //if(flag_falling)
   //   kp_val -=  0.05;

    yprSTAB[1].set_ki_coeff(ki_val);
    yprSTAB[1].set_kp_coeff(kp_val);
    yprSTAB[1].set_kd_coeff(kd_val);

    float kp_sum = yprSTAB[2].get_kp(); 
    float ki_sum = yprSTAB[2].get_ki(); 
    float kd_sum = yprSTAB[2].get_kd(); 
   //printf("ki: %f, kp: %f, sw3 : %d\n",ki_val,kp_val,RC_SW3);
   printf("Roll: %d, kp: %f, ki: %f, kd : %f, kp_sum: %d, ki_sum: %d, kd_sum: %d\n"
          ,roll,kp_val,ki_val,kd_val,(int)kp_sum,(int)ki_sum,(int)kd_sum);
   //printf("ki: %f\n",ki_val);

   sw1_prev = sw1;
   sw2_prev = sw2;
   flag_prev = flag;
}
   

void TimerClass::sig_handler_(int signum)
{

  pthread_mutex_lock(&TimerMutex_);

  //output to a log file
  //open log file
  #ifdef FILE_DUMP
  fstream logfile;
  logfile.open(filename, ios::out|ios::app);
  //logfile.open("/home/pi/c_code/QUADCOPTER_V2-master/RPI/PILOT/quadpilot.log", ios::out|ios::app);
  if (logfile.fail())          // Check for file creation and return error.
    {
      cout << "Error opening output.\n";
    }
  #endif

  float RCinput[N_RC_CHAN],PIDout[3];
  uint16_t ESC[N_SERVO];

  //------------------------------------------------------
  //1-Get Remote values using SPI
  union bytes{
    uint8_t u8[2];
    uint16_t u16;
  } rc_union;
  uint8_t checksum=0,recv_checksum=1;

  while (checksum != recv_checksum) {

    checksum=0;

    //for (int i=0;i<4;i++){
    for (int i=0;i<7;i++){
      ArduSPI.writeByte((uint8_t) (i+1)*10);
      rc_union.u8[0] = ArduSPI.rwByte((uint8_t) (i+1)*10+1);
      rc_union.u8[1] = ArduSPI.rwByte('S');
       
      //transaction ended
      RCinput[i] = (float) rc_union.u16;

      //printf("RCinput[%d] : %f\n",i,RCinput[i]);

      checksum+=rc_union.u8[0];
      checksum+=rc_union.u8[1];
    }

    //printf("check_sum : %d\n",checksum); 

    //Control checksum
    ArduSPI.writeByte('C');
    recv_checksum = ArduSPI.readByte();
    //printf("receive checksum : %d\n", recv_checksum);
    ArduSPI.writeByte('S');
    //recv_checksum = ArduSPI.rwByte('S');
    //printf("receive checksum : %d\n", recv_checksum);
  }

   // RC_MAX 2000
   // RC_MIN 1000  
  //Bounding RC values to avoid division by zeros fex.
    for (int i=1;i<4;i++){
      if ( RCinput[i] > RC_MAX)	RCinput[i] = RC_MAX;
      if ( RCinput[i] < RC_MIN)	RCinput[i] = RC_MIN;
    }

  //  printf("RC[4]: %f,  RC[5]:%f\n",RCinput[4],RCinput[5]);

 
  
  //printf("Befor RC[0]: %f, RC[1]: %f, RC[2]: %f, RC[3]: %f\n",RCinput[0],RCinput[1],RCinput[2],RCinput[3]);
  //outputing values to logfile

  #ifdef FILE_DUMP
  logfile << RCinput[0] << " " << RCinput[1] << " "
  	  << RCinput[2] << " " << RCinput[3] << " ";
  #endif

  // //convert into PID usable values
  //RCinput[0] = (RCinput[0] - THR_MIN)/(THR_MAX-THR_MIN) * 100.0;
  RCinput[1] = -(RCinput[1] -(RC_MAX+RC_MIN)/2.) /
    (RC_MAX-RC_MIN) * K_YAW; // K_YAW -> 260 
  RCinput[2] = (RCinput[2] -(RC_MAX+RC_MIN)/2.)/
    (RC_MAX-RC_MIN) * K_PITCH; // K_PITCH -> 120
  RCinput[3] = (RCinput[3] -(RC_MAX+RC_MIN)/2.)/
    (RC_MAX-RC_MIN) * K_ROLL; // K_ROLL -> 120

  //outputing values to logfile
  //logfile << RCinput[0] << " " << RCinput[1] << " "
 // 	  << RCinput[2] << " " << RCinput[3] << " ";

  //printf("After RC[0]: %f, RC[1]: %f, RC[2]: %f, RC[3]: %f\n",RCinput[0],RCinput[1], RCinput[2] , RCinput[3]);

  #ifdef XMODE
      //Switch to Xmode instead of +mode
      //orders are given in a ref frame rotated by 90deg.
      float cs45 = sqrt(2.)/2.;
      float Px = RCinput[2]*cs45 +  RCinput[3]*cs45;
      float Rx = - RCinput[2]*cs45 +  RCinput[3]*cs45;
      RCinput[2] = Px;
      RCinput[3] = Rx;
  #endif

  // printf("Received : %6.3f %6.3f %6.3f %6.3f\n", RCinput[0],
  // 	 RCinput[1], RCinput[2], RCinput[3]);

  //------------------------------------------------------
  //2- Get attitude of the drone
  int cnt=0;
  while (imu.getAttitude() < 0){
    cnt++;
  };

   Timer.dynamic_turning_pid((int) RCinput[4], (int) RCinput[5], (int) RCinput[6], (int)imu.ypr[PITCH]); 
   //printf("imu getAttitude() cnt : %d\n",cnt);


  //Compensate lost of Thrust due to angle of drone
  //RCinput[0] = RCinput[0]/cos(imu.ypr[ROLL]/180*M_PI)  /cos(imu.ypr[PITCH]/180*M_PI);

  //output to logfile
  // logfile << imu.ypr[YAW] << " " << imu.ypr[PITCH] << " "
  // 	  << imu.ypr[ROLL] << " "
  // 	  << imu.gyro[YAW] << " " << imu.gyro[PITCH] << " "
  // 	  << imu.gyro[ROLL] << " ";

  HMC5883L.getField(&rawData_);
  MS5611.getPressure(&rawData_.pressure);

  Timer.calibrateData();

 //printf("update RC[0] : %f\n", RCinput[0]);
#ifdef PRINT_DUMP
//  printf("SENSOR: Y:%3.2f, P:%3.2f R:%3.2f, M:%d, ATT:%3.2f\n"
//     ,imu.ypr[YAW],imu.ypr[PITCH],imu.ypr[ROLL],rawData_.mag_angle,calibratedData.altitude);
#endif
   //printf("GYRO: Y:%7.2f, %7.2f %7.2f\n",imu.gyro[YAW], imu.gyro[PITCH],imu.gyro[ROLL]);


  //printf("mag_x : %lf, mag_y : %lf, mag_z : %lf\n",calibratedData.magx,calibratedData.magy,calibratedData.magz);
  //printf("Temp : %lf, Altitude : %lf\n",calibratedData.temp,calibratedData.altitude);

  //------------------------------------------------------
  //3- Timer dt
  Timer.calcdt_();
  //printf("dt : %f \n",Timer.dt);

  //------------------------------------------------------
  //4-1 Calculate PID onuattitude
  #ifdef PID_STAB

  // first stage pid
  for (int i=1;i<DIM;i++){

    //yprSTAB[i].updateKpKi(RCinput[i+1],imu.ypr[i]);

    PIDout[i] =
      yprSTAB[i].update_pid_std(RCinput[i+1],imu.ypr[i],Timer.dt, (int)RCinput[0]);
  			 //   imu.ypr[i],
  			 //   Timer.dt);
  }

  //yaw is rate PID only
  //printf("First stage PID\n: "); 
  PIDout[YAW] = RCinput[YAW+1];

#ifdef PRINT_DUMP
   //printf("YAW:   RC_IN:%7.2f, SENSOR_IN:%7.2f, PID_OUT:%7.2f\n",RCinput[YAW+1],RCinput[YAW+1],PIDout[YAW]);
   //printf("PITCH: RC_IN:%7.2f, SENSOR_IN:%7.2f, PID_OUT:%7.2f\n",RCinput[PITCH+1],imu.ypr[PITCH], PIDout[PITCH]);
   //printf("ROLL:  RC_IN:%7.2f, SENSOR_IN:%7.2f, PID_OUT:%7.2f\n",RCinput[ROLL+1],imu.ypr[ROLL], PIDout[ROLL]);
#endif
  // second stage pid
   
/*
  for (int i=0;i<DIM;i++){
    PIDout[i] =
      yprRATE[i].update_pid_std(PIDout[i],
				imu.gyro[i],
				Timer.dt, (int) RCinput[0]);
  }
*/
   //printf("Second stage PID\n: "); 
   //printf("YAW:     IN:%7.2f SENSOR_IN:%7.2f PID_OUT:%7.2f\n",RCinput[YAW+1],imu.gyro[YAW],PIDout[YAW]);
   //printf("PITCH:   IN:%7.2f SENSOR_IN:%7.2f PID_OUT:%7.2f\n",RCinput[PITCH+1],imu.gyro[PITCH],PIDout[PITCH]);
   //printf("ROLL:    IN:%7.2f SENSOR_IN:%7.2f PID_OUT:%7.2f\n",RCinput[ROLL+1],imu.gyro[ROLL],PIDout[ROLL]);

  // printf("PITCH: %7.2f %7.2f %7.2f\n",RCinput[PITCH+1],
  // 	 imu.gyro[PITCH],
  // 	 PIDout[PITCH]);

  // printf("ROLL:  %7.2f %7.2f %7.2f\n",RCinput[ROLL+1],
  // 	 imu.gyro[ROLL],
  // 	 PIDout[ROLL]);


  #endif

  //4-2 Calculate PID on rotational rate
  #ifdef PID_RATE
  for (int i=0;i<DIM;i++){
    PIDout[i] =
      yprRATE[i].update_pid_std(RCinput[i+1],
      			    imu.gyro[i],
      			    Timer.dt);
  }
  //printf("%7.2f  %7.2f\n",imu.gyro[PITCH],Timer.PIDout[PITCH]);
  #endif

   //logfile << PIDout[YAW] << " " << PIDout[PITCH] << " " << PIDout[ROLL] << " ";
   
  #ifdef FILE_DUMP
    logfile << setprecision(4) << imu.ypr[YAW] << " " << imu.ypr[PITCH] << " " << imu.ypr[ROLL] << " ";
#endif

  //------------------------------------------------------
  //5- Send ESC update via SPI
  //compute each new ESC value

  //if THR is low disable PID and be sure that ESC receive Zero
  //printf("%f \n",RCinput[0]);

  if (RCinput[0] < 7.0) {
    for (int i=0;i<4;i++){
      ESC[i] = (uint16_t)0;
    }
  } else {

    // FL:0 FR:1 BL:2 BR:3
    //uint16_t throttle = (uint16_t)(RCinput[0]*10+1000);
    uint16_t throttle = (uint16_t)(RCinput[0]);
    PIDout[YAW] = 0;
    ESC[1] = (uint16_t)(throttle - PIDout[ROLL] + PIDout[PITCH] + PIDout[YAW]); // FR 
    ESC[3] = (uint16_t)(throttle - PIDout[ROLL] - PIDout[PITCH] - PIDout[YAW]); // BR
    ESC[0] = (uint16_t)(throttle + PIDout[ROLL] + PIDout[PITCH] + PIDout[YAW]); // FL
    ESC[2] = (uint16_t)(throttle + PIDout[ROLL] - PIDout[PITCH] - PIDout[YAW]); // BL

  }

  checksum = 0;
  for (int iesc=0;iesc < N_SERVO; iesc++) {
    ArduSPI.writeByte(ESC[iesc] & 0xff);
    checksum+=ESC[iesc] & 0xff;
    ArduSPI.writeByte((ESC[iesc] >> 8) & 0xff);
    checksum+=(ESC[iesc] >> 8) & 0xff;
    }
  ArduSPI.writeByte(checksum);
  //sending Proccess it
  ArduSPI.writeByte('P');

#ifdef PRINT_DUMP
   //printf("Sent: FL:%4d FR:%4d BL:%4d BR:%4d\n", ESC[0], ESC[1], ESC[2], ESC[3]);
#endif
   //printf("Sent: FR:%4d BL:%4d \n", ESC[1], ESC[2]);

  //------------------------------------------------------
  //6-compensate dt
  Timer.compensate_();

  //ouputting ESC values to logfile
  #ifdef FILE_DUMP
  logfile << ESC[0] << " " << ESC[1] << " "
  	  << ESC[2] << " " << ESC[3] << " ";


  logfile << setprecision(4) << yprSTAB[PITCH].get_kp() << " " << yprSTAB[PITCH].get_ki() << " "
          << yprSTAB[PITCH].get_kd() << " " << PIDout[PITCH] << " ";

  logfile << setprecision(4) << yprSTAB[ROLL].get_kp() << " " << yprSTAB[ROLL].get_ki() << " "
          << yprSTAB[ROLL].get_kd() << " " << PIDout[ROLL] << " ";

   logfile << setprecision(4) << imu.gyro[PITCH] << " " << imu.gyro[ROLL] << " "; 

  logfile << endl;
   
  //closing logfile
  logfile.close();
  #endif
  pthread_mutex_unlock(&TimerMutex_);
  //end of interrupt

}

void TimerClass::calibrateData() {
    /*
    calibratedData.x = (rawData_.x * (9.81 / 4096.0));
    calibratedData.y = (rawData_.y * (9.81 / 4096.0));
    calibratedData.z = (rawData_.z * (9.81 / 4096.0));
    calibratedData.p = (rawData_.p / 65.5);
    calibratedData.q = (rawData_.q / 65.5);
    calibratedData.r = (rawData_.r / 65.5);
    */
    calibratedData.temp = (rawData_.temp + 12412) / 340.0;
    calibratedData.magx = rawData_.mag_x / 1090.0;
    calibratedData.magy = rawData_.mag_y / 1090.0;
    calibratedData.magz = rawData_.mag_z / 1090.0;
    calibratedData.pressure = rawData_.pressure; //Pascals
    calibratedData.altitude = ((-8.31447 * 288.15) / (9.80665 * 0.0289644)) * log(calibratedData.pressure / 101325);
    calibratedData.q = -calibratedData.q;

      //Accelerometer scale and bias correction
    /*
    static double acceltemp[3];
    acceltemp[0] = calibratedData.x - accelZeroX;
    acceltemp[1] = calibratedData.y - accelZeroY;
    acceltemp[2] = calibratedData.z - accelZeroZ;
    calibratedData.x = accelEllipsoid00_ * acceltemp[0] + accelEllipsoid01_ * acceltemp[1] + accelEllipsoid02_ * acceltemp[2];
    calibratedData.y = accelEllipsoid11_ * acceltemp[1] + accelEllipsoid12_ * acceltemp[2];
    calibratedData.z = accelEllipsoid22_ * acceltemp[2];
    */

    //Magnetometer scale and bias correction
    static double magtemp[3];
    magtemp[0] = calibratedData.magx - magZeroX;
    magtemp[1] = calibratedData.magy - magZeroY;
    magtemp[2] = calibratedData.magz - magZeroZ;
    calibratedData.magx = magEllipsoid00_ * magtemp[0] + magEllipsoid01_ * magtemp[1] + magEllipsoid02_ * magtemp[2];
    calibratedData.magy = magEllipsoid11_ * magtemp[1] + magEllipsoid12_ * magtemp[2];
    calibratedData.magz = magEllipsoid22_ * magtemp[2];

      //Altitude LPF
#define LENGTH 20
    static int i = 0;
    static double mvAvg[LENGTH] = {0};
    mvAvg[i] = calibratedData.altitude;
    calibratedData.altitude = 0;
    for(int k = 0; k < LENGTH; k++) {
        calibratedData.altitude += mvAvg[k];
    }
    calibratedData.altitude /= LENGTH;
    i++;
    if(i == LENGTH) {
        i = 0;
    }
    //End Altitude LPF

}
