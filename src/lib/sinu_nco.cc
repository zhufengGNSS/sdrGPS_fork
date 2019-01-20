/*:sinu_nco.cc
*****************************************************************
 * SinusoidalNCO class implementation 
 *
 * This class is to generate the NCO for sinusoidal signal
 * It can output I and Q signal ( cos() and sin() )
 * Author: 
 *         Yu Lu, softwareGNSS@gmail.com
 *         Jan, 2005
 *
 ****************************************************************/

#include <math.h>
#include <iostream>
#include <fstream>
#include "./includes/sinu_nco.h"
using namespace std;

const double SinusoidalNCO::PI = 3.1415926;
const int SinusoidalNCO::lookup_table_length =4096 ; // the length of look-up table
const unsigned int SinusoidalNCO::MAX_PHASE = 0xFFFFFFFF;


// constructor of class SinusoidalNCO
SinusoidalNCO::SinusoidalNCO()
{
  lookup_table = new Sincos [lookup_table_length];
  if( lookup_table )
    init_lookuptable();
  
  phase_step = 1;
  current_phase = 0;
  initial_phase = 0.0;
}

// destructor 
SinusoidalNCO::~SinusoidalNCO()
{
  if( lookup_table )
    delete [] lookup_table;
}


// for future use
void SinusoidalNCO::init()
{
  ;
}

// for debug purpose, mode indicate different file format
void SinusoidalNCO::Save_debuginfo(string filename, int mode)
{
  
    int i;
  ofstream outfile(filename.c_str());

  if( mode == 1){
    outfile << "sintheta=["<< endl;
    for( i=0; i<lookup_table_length; i++)
      outfile << lookup_table[i].sin_theta <<" "<< endl;
    outfile << "];";
    
    outfile << "costheta=["<< endl;
    for( i=0; i<lookup_table_length; i++)
      outfile << lookup_table[i].cos_theta <<" "<< endl;
    outfile<< "];";
    return;
  }
  
  if( mode == 2)
    {
      outfile << "sincostheta=["<< endl;
      for( i=0; i<lookup_table_length; i++)
	outfile << lookup_table[i].sin_theta <<", " << lookup_table[i].cos_theta<< endl;
      outfile << "];";
      
    }
}

// init the lookup table 
void SinusoidalNCO::init_lookuptable()
{
	double theta; 

	for( int i=0; i<lookup_table_length; i++)
	{
		theta = 2*PI*i/lookup_table_length;
		lookup_table[i].sin_theta = (int)(sin( theta )*SINU_NCO_NORM_VAL);
		lookup_table[i].cos_theta = (int)(cos( theta )*SINU_NCO_NORM_VAL);
	}
}


// set the phase step of NCO
// According to Nyquist theorem, the step can't be greater that half of max phase
bool SinusoidalNCO::set_phasestep( unsigned int s)
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
unsigned int SinusoidalNCO::get_phasestep() const
{
  return phase_step;
}
#endif


// set inital phase for NCO
void SinusoidalNCO::set_initialphase( double ph )
{
  initial_phase = fmod(ph, 2*PI);
  current_phase += (unsigned int )((initial_phase)/(2*PI)*MAX_PHASE);
}

#ifndef __INLINE_FUNC__
double SinusoidalNCO::get_initialphase() const
{
  return initial_phase;
}

unsigned int SinusoidalNCO::get_currentphase() const
{
  return current_phase;
}
#endif

// Update the current phase of NCO when one clock comes
// also update the current value of output according to current phase
void SinusoidalNCO::OneClockUpdate()
{
  current_phase += phase_step;
  int phase_idx = current_phase>>20;  // max phase is 2^32, while the lookup talbe depth is only 2^12;
  current_value = lookup_table[phase_idx];
  
}

#ifndef __INLINE_FUNC__
int SinusoidalNCO::I_output() const 
{
  return current_value.cos_theta;
}

int SinusoidalNCO::Q_output() const
{
  return current_value.sin_theta;
}
#endif
