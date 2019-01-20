/*:square.cc
****************************************************************
 * SquareNCO class implementation
 *
 * This class is used to generate the NCO for square wave signal
 * It can output in-phase and reversed output ( 0 & 1)
 * Author: 
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2005
 ***************************************************************/


#include <iostream>
#include <fstream>

#include "./includes/square_nco.h"

using namespace std;
const double SquareNCO::PI = 3.1415926;
const unsigned int SquareNCO::MAX_PHASE = 0xFFFFFFFF;

// constructor of class SquareNCO
SquareNCO::SquareNCO()
{
  phase_step = 1;       // defaule phase step
  current_phase = 0;
  duty_cycle = (unsigned int )(0.5*MAX_PHASE); // defaule duty clcye is 50% 
  current_value = 1;  //defaule value is high-level
  clock_edge = 0;
}

// destructor
SquareNCO::~SquareNCO()
{
}

// for future use
void SquareNCO::init()
{
  ;
}

// for debug purpose, mode indicate different file format
void SquareNCO::Save_debuginfo(string filename, int mode)
{
  ;
}

// set the phase step of NCO
// According to Nyquist theorem, the step can't be greater that half of max phase
bool SquareNCO::set_phasestep( unsigned int s)
{
  unsigned int max_step = MAX_PHASE /2 +1;

  //  cout << "max_step : " << max_step << endl;
  if( s > max_step ) // phase step can't be bigger than half of 2^31;
    {
      
      return false;
    }
  else
    {
      phase_step = s;
      return true;
    }
}

#ifndef __INLINE_FUNC__
unsigned int SquareNCO::get_phasestep() const
{
  return phase_step;
}


//set initial phase of NCO
void SquareNCO::set_initialphase( double ph )
{
  initial_phase = fmod( ph, 2*PI);
  current_phase += (unsigned int)(initial_phase/(2*PI)*MAX_PHASE);
}

double SquareNCO::get_initialphase() const
{
  return initial_phase;
}

// set duty cycle of generated square wave
void SquareNCO::set_dutycycle( double dcycle )
{
  assert( dcycle > 0.0 && dcycle < 1.0);
  duty_cycle = (unsigned int )( dcycle * MAX_PHASE );
}

double SquareNCO::get_dutycycle() const
{
  return (double) duty_cycle/MAX_PHASE;
}

unsigned int SquareNCO::get_currentphase() const
{
  return current_phase;
}
#endif

// update the current phase of NCO when one clock comes
// also update current value according to current phase
void SquareNCO::OneClockUpdate()
{
  current_phase += phase_step;
  
  int new_value;

  if( current_phase > duty_cycle )
    new_value = 0;
  else
    new_value = 1;

  // now check if clock rising happened or not
  
  if( current_value == 1 && new_value == 0) // falling jump happened
    {
      clock_edge = -1;
      current_value = new_value;
      return;
    }

  if( current_value == 0 && new_value == 1) // rising jump happened
    {
      clock_edge = 1;
      current_value = new_value;
      return;
    }
  
  // otherwise , no jump happened, reset clock_edge
  clock_edge = 0;
  current_value = new_value;
}


#ifndef __INLINE_FUNC__
// return 0 if no jump happened
// return -1 if falling jump happened
// return 1 if rising jump happened
int SquareNCO::edge_happened() const
{
  return clock_edge;
}

// in-phase output
int SquareNCO::Output() const
{
  return current_value;
}
// reversely output
int SquareNCO::R_Output() const
{
  return current_value == 1? 0:1;
}

#endif
