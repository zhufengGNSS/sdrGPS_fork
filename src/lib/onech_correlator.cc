/*:onech_correlator.cc
********************************************************
 * One channel correlator module implementation
 * 
 * This class is used to generate the CA prn code for certain PRN number
 * It can output prn code of three phases: early, prompt, delay, 
 * seperated by 1/2 chip, relatively
 * It also generates the carrier signal for de-modulation.
 * It also receives the input data sample and calculate the integration
 * with local CA code, output I and Q results for further processing
 *
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2005
 *******************************************************/
#include "./includes/onech_correlator.h"
#include "./includes/cacode_module.h"
#include "./includes/sinu_nco.h"

#include <iostream>
#include <fstream>


using namespace std;

//inline double SinusoidalNCO::I_output() const ;
//inline double SinusoidalNCO::Q_output() const ;

// constructor and destructor

OnechCorrelator::OnechCorrelator()
{
  samplingfreq = 5e6; // default sampling freq is 5MHz
  
  ca_osc = new SinusoidalNCO;
  cd_osc_module = new CAcodeModule;

  cd_osc_module->set_sampling_freq( samplingfreq );
}

OnechCorrelator::~OnechCorrelator()
{
  delete ca_osc;
  delete cd_osc_module;
}

// initialize setting for correlator,
// including sampling freq, code_step , carrier_step, prn number.

void OnechCorrelator::init_correlator( const Correlator_config& corr_cfg)
{
  set_prn_num( corr_cfg.prnnum );
  set_sampling_freq( corr_cfg.sampling_freq );
  set_carrier_freq( corr_cfg.carrier_freq );
  unsigned int codestep, carrierstep;
  codestep = (unsigned int)((2.046e6/samplingfreq)*0xFFFFFFFF); // code freq setting
  set_code_NCO_step(codestep);   // 0x67DF4E1B); // 2.046Mhz
                              // 0xffffffff = 4294967295
  carrierstep = (unsigned int)((corr_cfg.carrier_freq/samplingfreq)*0xFFFFFFFF); // carrier freq setting

  set_carrier_NCO_step(carrierstep); 
}

// set and get CA prn number

void OnechCorrelator::set_prn_num( unsigned int prn)
{
  cd_osc_module->set_prn_num( prn );
}

unsigned int OnechCorrelator::get_prn_num() const
{
  return cd_osc_module->get_prn_num();
}

void OnechCorrelator::reset_ch()
{
  cd_osc_module->reset_module();
  I_early=Q_early=I_prompt=Q_prompt=I_late=Q_late = 0;
  outputflag = false;
}

// set and read the sampling frequency
void OnechCorrelator::set_sampling_freq( double fs )
{
  samplingfreq = fs;
  cd_osc_module->set_sampling_freq( fs );
}

#ifndef __INLINE_FUNC__
double OnechCorrelator::get_sampling_freq( ) const
{
  return samplingfreq;
}
#endif

#ifndef __INLINE_FUNC__
// set and read center carrier frequency without doppler shift

void OnechCorrelator::set_carrier_freq( double fs )
{
  carrierfreq = fs;
}

double OnechCorrelator::get_carrier_freq() const
{
  return carrierfreq;
}
#endif


// set and read carrier NCO phase step

void OnechCorrelator::set_carrier_NCO_step( unsigned int ca_ph)
{
  carrier_step = ca_ph;
  ca_osc->set_phasestep( ca_ph );
}

#ifndef __INLINE_FUNC__
unsigned int OnechCorrelator::get_carrier_NCO_step() const
{
  return carrier_step;
}
#endif


// set and read code NCO phase step
void OnechCorrelator::set_code_NCO_step( unsigned int cd_ph )
{
  code_step = cd_ph;
  cd_osc_module->set2046M( cd_ph );
}

#ifndef __INLINE_FUNC__
unsigned int OnechCorrelator::get_code_NCO_step() const
{
  return code_step;
}
#endif


// Read the code phase,  in the range of [1-1023]
// This function will be called in the correlator module to read 
// measurement when TIC happened
int OnechCorrelator::get_code_NCO_phase() const
{
  return cd_osc_module->out_prompt_phase();
}

// Read the code subphase , in the range of [0-1023]
int OnechCorrelator::get_code_NCO_subPhase() const
{
  return cd_osc_module->out_subphase();
}

#ifndef __INLINE_FUNC__
//reset the value of Integration&Dump unit
void OnechCorrelator::clear_integration()
{
  I_early=Q_early=I_prompt=Q_prompt=I_late=Q_late = 0;
}

#endif

#ifndef __INLINE_FUNC__

//get I_ Q_
void OnechCorrelator::getIQ(double *stable_IQ, double* transient_IQ) const
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
void OnechCorrelator::getIQ(double *stable_IQ) const
{
  stable_IQ[0] = I_e_pre;
  stable_IQ[1] = Q_e_pre;
  stable_IQ[2] = I_p_pre;
  stable_IQ[3] = Q_p_pre;
  stable_IQ[4] = I_l_pre;
  stable_IQ[5] = Q_l_pre;
}

void OnechCorrelator::get_IQ_measure( P_correlator_IQ p_corr_IQ )
{
  assert( p_corr_IQ); // check if p_corr_IQ valid

  p_corr_IQ->early_I = I_e_pre;
  p_corr_IQ->early_Q = Q_e_pre;
  p_corr_IQ->prompt_I = I_p_pre;
  p_corr_IQ->prompt_Q = Q_p_pre;
  p_corr_IQ->late_I = I_l_pre;
  p_corr_IQ->late_Q = Q_l_pre;
}

bool OnechCorrelator::is_time_to_output() const
{
  return outputflag;
}

#endif

void OnechCorrelator::slew_half_chip( unsigned int chip_c)
{
  assert( chip_c>=0 && chip_c<2046 );
  
  cd_osc_module->slew_half_chip( chip_c );
}
  
// one clock comes, duty here is:
//   1. update carrier and code NCO status
//   2. calculate the integration for E,P,L version of CA code
//   3. At last phase digit, clear I&D unit and set the outputflag 

void OnechCorrelator::OneClockUpdate( int data_smp)
{
  
  ca_osc->OneClockUpdate();
  cd_osc_module->OneClockUpdate();

  outputflag =  cd_osc_module->time_to_dump();
  
  if( outputflag )  // time to dump current integration results
    {
      I_e_pre = (double)(I_early>>SINU_NCO_NORM_BIT);
      Q_e_pre = (double)(Q_early>>SINU_NCO_NORM_BIT);
      I_p_pre = (double)(I_prompt>>SINU_NCO_NORM_BIT);
      Q_p_pre = (double)(Q_prompt>>SINU_NCO_NORM_BIT);
      I_l_pre = (double)(I_late>>SINU_NCO_NORM_BIT);
      Q_l_pre = (double)(Q_late>>SINU_NCO_NORM_BIT);
      
      clear_integration(); // clear I&D units
    }
  
  int codeEPL[3], I_out, Q_out;
  cd_osc_module->out_EPL_code( codeEPL );
  I_out = ca_osc->I_output()*data_smp;
  Q_out = ca_osc->Q_output()*data_smp;

  I_early  += (codeEPL[0]>0? I_out:-I_out); // early code, carrier I
  Q_early  += (codeEPL[0]>0? Q_out:-Q_out); // early code, carrier Q
  I_prompt += (codeEPL[1]>0? I_out:-I_out); // prompt code, carrier I
  Q_prompt += (codeEPL[1]>0? Q_out:-Q_out); // prompt code, carrier Q
  I_late   += (codeEPL[2]>0? I_out:-I_out); // late code, carrier I
  Q_late   += (codeEPL[2]>0? Q_out:-Q_out); // late code, carrier Q
 
  /*
  I_early  += data_smp*ca_osc->I_output()*codeEPL[0]; // early code, carrier I
  Q_early  += data_smp*ca_osc->Q_output()*codeEPL[0]; // early code, carrier Q
  I_prompt += data_smp*ca_osc->I_output()*codeEPL[1]; // prompt code, carrier I
  Q_prompt += data_smp*ca_osc->Q_output()*codeEPL[1]; // prompt code, carrier Q
  I_late   += data_smp*ca_osc->I_output()*codeEPL[2]; // late code, carrier I
  Q_late   += data_smp*ca_osc->Q_output()*codeEPL[2]; // late code, carrier Q
  */
}

//get the replica of local IF replica, which is the mixed carrier and code
//This is useful when doing fast acquisition

void OnechCorrelator::GetLocalReplica( int* data)
{
  int codeEPL[3];  
  ca_osc->OneClockUpdate();
  cd_osc_module->OneClockUpdate();
  cd_osc_module->out_EPL_code( codeEPL );
  data[0] = ca_osc->I_output()*codeEPL[1];
  data[1] = ca_osc->Q_output()*codeEPL[1];
  
}
