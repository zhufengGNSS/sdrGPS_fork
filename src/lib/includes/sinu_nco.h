#ifndef SINU_NCO_H
#define SINU_NCO_H
#include <fstream>
#include "softgps.h"

typedef struct Struct_sincos{
  int sin_theta;
  int cos_theta;
} Sincos, *Pt_Sincos;

class SinusoidalNCO{

 public:
  static const int lookup_table_length; // =4096 ; // the length of look-up table
  static const unsigned int MAX_PHASE; // = 0xFFFFFFFF;
  const static double PI;
	
  SinusoidalNCO(); // constructor
  ~SinusoidalNCO();// destructor


  bool set_phasestep( unsigned int ); //set step phase, namely, frequency is controlled by this function
  unsigned int get_phasestep() const; // get step phase
  void set_initialphase( double );    // set initial phase
  double get_initialphase( ) const;
  unsigned int get_currentphase() const;// get oscillator phase
	
  void OneClockUpdate();     // one clock comes, update everything
  int I_output() const;   // output I,Q
  int Q_output() const;
  void  Save_debuginfo(std::string , int ); // for debug info

 protected:
  ;
 private:
  Pt_Sincos lookup_table;    // lookup table, 
  unsigned int current_phase;
  unsigned int phase_step;
  double initial_phase;
  Sincos current_value;

  void init();
  void init_lookuptable();
};



//const int SinusoidalNCO::lookup_table_length = 4096
//const double SinusoidalNCO::PI = 3.1415926;


#ifdef __INLINE_FUNC__
inline unsigned int SinusoidalNCO::get_phasestep() const
{
  return phase_step;
}
#endif

#ifdef __INLINE_FUNC__
inline double SinusoidalNCO::get_initialphase() const
{
  return initial_phase;
}

inline unsigned int SinusoidalNCO::get_currentphase() const
{
  return current_phase;
}
#endif

#ifdef __INLINE_FUNC__
inline int SinusoidalNCO::I_output() const 
{
  return current_value.cos_theta;
}

inline int SinusoidalNCO::Q_output() const
{
  return current_value.sin_theta;
}
#endif


#endif
