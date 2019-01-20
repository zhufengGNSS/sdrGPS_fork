#ifndef SQUARE_NCO_H
#define SQUARE_NCO_H

#include <string>
#include <assert.h>
#include <math.h>
using namespace std;

class SquareNCO{
 public:

  static const unsigned int MAX_PHASE; // = 0xFFFFFFFF;
  static const double PI;
  
  SquareNCO();
  ~SquareNCO();

  bool set_phasestep( unsigned int );
  unsigned int get_phasestep() const;
  void set_initialphase( double );
  double get_initialphase() const;
  void set_dutycycle( double );
  double get_dutycycle() const;
  unsigned int get_currentphase() const;

  void OneClockUpdate(); // one clock comes, update everything
  int Output() const; // output of Square wave
  int R_Output() const; // reverse output of Output()
  int edge_happened() const; // give the clock edge info
  void Save_debuginfo( string, int); // for debug use only
  
 protected:
 private:
  int clock_edge;   // indicate if the rising edge of square wave happened 
  unsigned int current_phase;
  unsigned int phase_step;
  double initial_phase;
  int current_value;
  unsigned int  duty_cycle;  // this int determines the duty cycle of generated square wave
  
  void init();

};

#ifdef __INLINE_FUNC__
inline unsigned int SquareNCO::get_phasestep() const
{
  return phase_step;
}


//set initial phase of NCO
inline void SquareNCO::set_initialphase( double ph )
{
  initial_phase = fmod( ph, 2*PI);
  current_phase += (unsigned int)(initial_phase/(2*PI)*MAX_PHASE);
}

inline double SquareNCO::get_initialphase() const
{
  return initial_phase;
}

// set duty cycle of generated square wave
inline void SquareNCO::set_dutycycle( double dcycle )
{
  assert( dcycle > 0.0 && dcycle < 1.0);
  duty_cycle = (unsigned int )( dcycle * MAX_PHASE );
}

inline double SquareNCO::get_dutycycle() const
{
  return (double) duty_cycle/MAX_PHASE;
}

inline unsigned int SquareNCO::get_currentphase() const
{
  return current_phase;
}
#endif

#ifdef __INLINE_FUNC__
// return 0 if no jump happened
// return -1 if falling jump happened
// return 1 if rising jump happened
inline int SquareNCO::edge_happened() const
{
  return clock_edge;
}

// in-phase output
inline int SquareNCO::Output() const
{
  return current_value;
}
// reversely output
inline int SquareNCO::R_Output() const
{
  return current_value == 1? 0:1;
}

#endif

#endif
