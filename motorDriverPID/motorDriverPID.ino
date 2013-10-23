#include <TimerOne.h>

struct PID{
  long int command; 	/* commanded value */
  long int feedback; 	/* feedback value */
  float error; 		/* command - feedback */
  float deadband; 		/* param: deadband */
  float maxerror; 		/* param: limit for error */
  float maxerror_i; 	/* param: limit for integrated error */
  float maxerror_d; 	/* param: limit for differentiated error */
  float maxcmd_d; 		/* param: limit for differentiated cmd */
  float error_i; 		/* opt. param: integrated error */
  float prev_error; 	/* previous error for differentiator */
  float error_d; 		/* opt. param: differentiated error */
  long int prev_cmd; 	/* previous command for differentiator */
  float cmd_d; 		/* opt. param: differentiated command */
  float bias; 			/* param: steady state offset */
  float pgain; 		/* param: proportional gain */
  float igain; 		/* param: integral gain */
  float dgain; 		/* param: derivative gain */
  float ff0gain; 		/* param: feedforward proportional */
  float ff1gain; 		/* param: feedforward derivative */
  float maxoutput; 	/* param: limit for PID output */
  float output; 		/* the output value */
  short enable; 		/* enable input */
  short limit_state; 	/* 1 if in limit, else 0 */
  short multiplier; 	/* pc command multiplier */
  short ticksperservo; /* number of 100us ticks/servo cycle */
  int cksum; 		/* data block cksum used to verify eeprom */
} 
pid;

void init_pid(void)
{
  /* init all structure members */
  pid.enable = 1;		// mirror state of PIN
  pid.command = 0.0;
  pid.feedback = 0.0;
  pid.error = 0.0;
  pid.output = 0.0;
  pid.deadband = 0.0;
  pid.maxerror = 1000.0;
  pid.maxerror_i = 0.0;
  pid.maxerror_d = 0.0;
  pid.maxcmd_d = 0.0;
  pid.error_i = 0.0;
  pid.prev_error = 0.0;
  pid.error_d = 0.0;
  pid.prev_cmd = 0.0;
  pid.limit_state = 0;
  pid.cmd_d = 0.0;
  pid.bias = 0.0;
  pid.pgain = 0.0;
  pid.igain = 0.0;
  pid.dgain = 0.0;
  pid.ff0gain = 0.0;
  pid.ff1gain = 0.0;
  pid.maxoutput = 255;		// max PWM DC
  pid.multiplier = 1;
  pid.ticksperservo = 2;		// 1000us/servo calc
}

void calc_pid( void )
{
  float tmp1;
  int enable;
  float periodfp, periodrecip;
  long period = pid.ticksperservo * 100000; 	/* thread period in ns */

  /* precalculate some timing constants */
  periodfp = period * 0.000000001;		// usually .001 sec
  periodrecip = 1.0 / periodfp;			// usually 1000.0
  /* get the enable bit */
  enable = pid.enable;
  /* calculate the error */
  tmp1 = (float)(pid.command - pid.feedback);
  pid.error = tmp1;
  /* apply error limits */
  if (pid.maxerror != 0.0) 
  {
    if (tmp1 > pid.maxerror) 
    {
      tmp1 = pid.maxerror;
    } 
    else if (tmp1 < -pid.maxerror) 
    {
      tmp1 = -pid.maxerror;
    }
  }
  /* apply the deadband */
  if (tmp1 > pid.deadband) 
  {
    tmp1 -= pid.deadband;
  } 
  else if (tmp1 < -pid.deadband) 
  {
    tmp1 += pid.deadband;
  }
  else 
  {
    tmp1 = 0;
  }

  /* do integrator calcs only if enabled */
  if (enable != 0) 
  {
    /* if output is in limit, don't let integrator wind up */
    if ( pid.limit_state == 0 ) 
    {
      /* compute integral term */
      pid.error_i += tmp1 * periodfp;
    }
    /* apply integrator limits */
    if (pid.maxerror_i != 0.0) 
    {
      if (pid.error_i > pid.maxerror_i) 
      {
        pid.error_i = pid.maxerror_i;
      } 
      else if (pid.error_i < -pid.maxerror_i) 
      {
        pid.error_i = -pid.maxerror_i;
      }
    }
  } 
  else 
  {
    /* not enabled, reset integrator */
    pid.error_i = 0;
  }

  /* calculate derivative term */
  pid.error_d = (tmp1 - pid.prev_error) * periodrecip;
  pid.prev_error = tmp1;
  /* apply derivative limits */
  if (pid.maxerror_d != 0.0) 
  {
    if (pid.error_d > pid.maxerror_d) 
    {
      pid.error_d = pid.maxerror_d;
    } 
    else if (pid.error_d < -pid.maxerror_d) 
    {
      pid.error_d = -pid.maxerror_d;
    }
  }

  /* calculate derivative of command */
  pid.cmd_d = (float)(pid.command - pid.prev_cmd) * periodrecip;
  pid.prev_cmd = pid.command;

  /* apply derivative limits */
  if (pid.maxcmd_d != 0.0) 
  {
    if (pid.cmd_d > pid.maxcmd_d) 
    {
      pid.cmd_d = pid.maxcmd_d;
    } 
    else if (pid.cmd_d < -pid.maxcmd_d) 
    {
      pid.cmd_d = -pid.maxcmd_d;
    }
  }

  /* do output calcs only if enabled */
  if (enable != 0) 
  {
    /* calculate the output value */
    tmp1 =
      pid.bias + pid.pgain * tmp1 + 
      pid.igain * pid.error_i +
      pid.dgain * pid.error_d;
    tmp1 += pid.command * pid.ff0gain + pid.cmd_d * pid.ff1gain;
    /* apply output limits */
    if (pid.maxoutput != 0.0) 
    {
      if (tmp1 > pid.maxoutput) 
      {
        tmp1 = pid.maxoutput;
        pid.limit_state = 1;
      } 
      else if (tmp1 < -pid.maxoutput) 
      {
        tmp1 = -pid.maxoutput;
        pid.limit_state = 1;
      } 
      else 
      {
        pid.limit_state = 0;
      }
    }
  } 
  else 
  {
    /* not enabled, force output to zero */
    tmp1 = 0.0;
    pid.limit_state = 0;
  }

  pid.output = tmp1;
  /* done */
}


volatile int RPM = 0;
volatile unsigned int lastMillis = 0;
void updateRPM()
{
  if(!lastMillis)
  {
    lastMillis = millis();
    RPM = 0;
    return;
  }
  int diff = millis() - lastMillis;
  lastMillis = millis();
  if(diff > 0)
  {
    RPM = 60000 / diff;
  }
}

volatile int do_servo;

void setup() {
  Serial.begin(9600);
  attachInterrupt(0, updateRPM, RISING);
  init_pid();
  do_servo = 0;
}

void loop() {
  unsigned int curMillis = millis();
  if(lastMillis && (curMillis - lastMillis) > 1000)
  {
    lastMillis = 0;
    RPM = 0;
  }
  pid.feedback = RPM;
  if(do_servo)
  {
    do_servo = 0;
    calc_pid();
  }
}
