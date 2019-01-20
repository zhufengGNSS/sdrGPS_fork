#ifndef ONECH_CORRELATOR_H
#define ONECH_CORRELATOR_H

#include <assert.h>

#include "softgps.h"

class SinusoidalNCO;
class CAcodeModule;


class OnechCorrelator{
 public:
  
  OnechCorrelator();
  ~OnechCorrelator();

  void init_correlator( const Correlator_config &);

  void set_prn_num( unsigned int );
  unsigned int get_prn_num() const;
  void reset_ch();

  // set and read the sampling frequency 
  void set_sampling_freq( double );
  double get_sampling_freq( ) const;
  // set and read the carrier frequency without Doppler shift
  void set_carrier_freq( double );
  double get_carrier_freq( ) const;

  // control and read the phase step of carrier NCO
  void set_carrier_NCO_step( unsigned int );
  unsigned int get_carrier_NCO_step() const;

  // control and read the phase step of code NCO
  void set_code_NCO_step( unsigned int );
  unsigned int get_code_NCO_step() const;

  int get_code_NCO_phase() const;
  int get_code_NCO_subPhase() const;
  //  void getMeas(correlator_meas* ) const;

  // clear I_, Q_ to zero
  void clear_integration(); 

  // get I_, Q_ 
  void getIQ(double*, double*) const; // get the transient values too
  void getIQ(double*) const;          // only get the stable values

  // get measurement 
  void get_IQ_measure( P_correlator_IQ );
  
  // check if it is time to output integration & dump
  bool is_time_to_output() const;
  
  void slew_half_chip( unsigned int );
  // one clock comes, update everything
  void OneClockUpdate( int ); // input is the data sample of current instant
  void GetLocalReplica( int* data);
  
  
 protected:
 private:

  double samplingfreq;   // sampling frequency 
  double carrierfreq;    // center frequency without Doppler shift
  int I_early, Q_early;  // integration result of early code, I and Q
  int I_prompt, Q_prompt; // integration for prompt code, I and Q
  int I_late, Q_late;  // integration for late code, I and Q
  
  double I_e_pre, Q_e_pre, I_p_pre, Q_p_pre, I_l_pre, Q_l_pre; // old value of integration
  
  bool outputflag;
  
  // phase step for carrier and code NCOs
  unsigned int carrier_step, code_step; 

  SinusoidalNCO *ca_osc;
  CAcodeModule  *cd_osc_module;

  
};



#ifdef __INLINE_FUNC__
// set and read the sampling frequency
inline double OnechCorrelator::get_sampling_freq( ) const
{
  return samplingfreq;
}
#endif

#ifdef __INLINE_FUNC__
// set and read center carrier frequency without doppler shift

inline void OnechCorrelator::set_carrier_freq( double fs )
{
  carrierfreq = fs;
}

inline double OnechCorrelator::get_carrier_freq() const
{
  return carrierfreq;
}
#endif

#ifdef __INLINE_FUNC__
// set and read carrier NCO phase step

inline unsigned int OnechCorrelator::get_carrier_NCO_step() const
{
  return carrier_step;
}
#endif

#ifdef __INLINE_FUNC__
// set and read code NCO phase step
inline unsigned int OnechCorrelator::get_code_NCO_step() const
{
  return code_step;
}
#endif

#ifdef __INLINE_FUNC__
//reset the value of Integration&Dump unit
inline void OnechCorrelator::clear_integration()
{
  I_early=Q_early=I_prompt=Q_prompt=I_late=Q_late = 0;
}
#endif

#ifdef __INLINE_FUNC__
//get I_ Q_
inline void OnechCorrelator::getIQ(double *stable_IQ, double* transient_IQ) const
{
  stable_IQ[0] = I_e_pre;
  stable_IQ[1] = Q_e_pre;
  stable_IQ[2] = I_p_pre;
  stable_IQ[3] = Q_p_pre;
  stable_IQ[4] = I_l_pre;
  stable_IQ[5] = Q_l_pre;

  transient_IQ[0] = (double)(I_early>>SINU_NCO_NORM_BIT);
  transient_IQ[1] = (double)(Q_early>>SINU_NCO_NORM_BIT);
  transient_IQ[2] = (double)(I_prompt>>SINU_NCO_NORM_BIT);
  transient_IQ[3] = (double)(Q_prompt>>SINU_NCO_NORM_BIT);
  transient_IQ[4] = (double)(I_late>>SINU_NCO_NORM_BIT);
  transient_IQ[5] = (double)(Q_late>>SINU_NCO_NORM_BIT);
}  
inline void OnechCorrelator::getIQ(double *stable_IQ) const
{
  stable_IQ[0] = I_e_pre;
  stable_IQ[1] = Q_e_pre;
  stable_IQ[2] = I_p_pre;
  stable_IQ[3] = Q_p_pre;
  stable_IQ[4] = I_l_pre;
  stable_IQ[5] = Q_l_pre;
}

inline void OnechCorrelator::get_IQ_measure( P_correlator_IQ p_corr_IQ )
{
  assert( p_corr_IQ); // check if p_corr_IQ valid

  p_corr_IQ->early_I = I_e_pre;
  p_corr_IQ->early_Q = Q_e_pre;
  p_corr_IQ->prompt_I = I_p_pre;
  p_corr_IQ->prompt_Q = Q_p_pre;
  p_corr_IQ->late_I = I_l_pre;
  p_corr_IQ->late_Q = Q_l_pre;
}
  
inline bool OnechCorrelator::is_time_to_output() const
{
  return outputflag;
}
#endif


#endif
