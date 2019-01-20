/*:multi_correlator.cc
********************************************************
 * 12 channels correlator module implementation
 * 
 * This class is used to generate the 12 channel correlator 
 * Just contains 12 copies of onechannel_correlator
 * each channel will update with one coming signal sample
 *
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2005
 *******************************************************/

#include "./includes/multich_correlator.h"
#include "./includes/square_nco.h"
#include <assert.h>
#include <string.h>

//constructor: use config info to initialize all 12 channel correlator
MultichCorrelator::MultichCorrelator(const Correlator_config& corr_cfg)
{
    int i;
  // defaultly all channels are de-activated
  for(i=0; i<12; i++)
    ch_activated[i] = 0; 

  ticvalue = old_ticvalue = 0; 

  for( i=0; i<12; i++)
    ch12_correlator[i].init_correlator(corr_cfg);
  
  tic_clk_gen = new SquareNCO;
  
  sampling_freq = corr_cfg.sampling_freq;
  carrier_ref = corr_cfg.carrier_freq;
  nco_norm_factor=((double)(0xFFFFFFFF))/sampling_freq;
  
  // default TIC period is 100ms, namely, 10Hz
  unsigned int tmp_i = (unsigned int)(10.0*nco_norm_factor);
  tic_period = nco_norm_factor/tmp_i;
  tic_clk_gen->set_phasestep(tmp_i);

  memset((void*)ch12_meas, 0, 12*sizeof(correlator_meas));
  meas_rdy_flag = false;
  IQ_rdy_flag = 0;

}

// destructor, release the allocated memory
MultichCorrelator::~MultichCorrelator()
{
  delete tic_clk_gen;
}

// set new TIC period
void MultichCorrelator::set_TIC(unsigned int tic_l)
{
  tic_clk_gen->set_phasestep( tic_l );
}


// set and get carrier freq and sampling freq
void MultichCorrelator::set_sampling_freq(double fs)
{
  sampling_freq = fs;
}

double MultichCorrelator::get_sampling_freq() const
{
  return sampling_freq;
}

void MultichCorrelator::set_carrier_freq(double ca_fs)
{
  carrier_ref = ca_fs;
}

double MultichCorrelator::get_carrier_freq() const
{
  return carrier_ref;
}
double MultichCorrelator::get_ticperiod(void)
{
    return tic_period;
}
// get TIC flag: 
//    1: TIC happened
//    0: TIC not happened
// When TIC happened, baseband unit should read the measurement
bool MultichCorrelator::is_time_to_get_meas() const
{
  return meas_rdy_flag;
}

////////////////////////////////////
// following are the functions for distinct channels

void MultichCorrelator::activate_ch( int ch )
{
  assert( ch>=0 && ch<12);
  ch_activated[ch] = 1;
}
void MultichCorrelator::set_prn_num(int ch, unsigned int prn)
{
  assert(ch >=0 && ch<12);
  ch12_correlator[ch].set_prn_num(prn);
}
unsigned int MultichCorrelator::get_prn_num( int ch) const
{
  assert(ch >=0 && ch<12);
  return ch12_correlator[ch].get_prn_num();
}
void MultichCorrelator::reset_ch(int ch)
{
  assert(ch >=0 && ch<12);
  ch12_correlator[ch].reset_ch();
}
void   MultichCorrelator::set_carrier_NCO_step(int ch, unsigned int ca_step)
{
  assert(ch >=0 && ch<12);
  ch12_correlator[ch].set_carrier_NCO_step(ca_step);
}
unsigned int MultichCorrelator::get_carrier_NCO_step(int ch) const
{
  assert(ch >=0 && ch<12);
  return ch12_correlator[ch].get_carrier_NCO_step();
}
void MultichCorrelator::set_code_NCO_step(int ch, unsigned int cd_step)
{
  assert(ch >=0 && ch<12);
  ch12_correlator[ch].set_code_NCO_step(cd_step);
}
unsigned int MultichCorrelator::get_code_NCO_step(int ch) const
{
  assert(ch >=0 && ch<12);
  return ch12_correlator[ch].get_code_NCO_step();
}
int MultichCorrelator::get_code_NCO_phase(int ch) const
{
  assert(ch >=0 && ch<12);
  return ch12_correlator[ch].get_code_NCO_phase();
}
int MultichCorrelator::get_code_NCO_subPhase(int ch) const
{
  assert( ch>=0 && ch<12 );
  return ch12_correlator[ch].get_code_NCO_subPhase();
}
void MultichCorrelator::get_IQ_measure(int ch, P_correlator_IQ p_corr_IQ)
{
  assert(ch >=0 && ch<12);
  ch12_correlator[ch].get_IQ_measure( p_corr_IQ);
}
bool MultichCorrelator::is_time_to_get_IQ( int ch) const
{
   assert(ch >=0 && ch<12);
  return ch12_correlator[ch].is_time_to_output();
}

void MultichCorrelator::slew_half_chip(int ch, unsigned int chip_c)
{
  assert(ch >=0 && ch<12);
  ch12_correlator[ch].slew_half_chip( chip_c );
}

unsigned short MultichCorrelator:: get_IQ_rdy_flag() const
{
  return IQ_rdy_flag; 
}

// one clock update function 
// each correlator need to update
// also the TIC need to be updated, if rising edge happens, set the meas_rdy_flag
// and store all the measurement

void MultichCorrelator::OneClockUpdate(int data_smp)
{
  unsigned short bit_i=0; 
  IQ_rdy_flag = 0; 

  // if the channel is activated, call its OneClockUpdate function
  for( int i=0; i<12; i++)
    if( ch_activated[i] )
      {
	ch12_correlator[i].OneClockUpdate(data_smp);
	
	bit_i = (ch12_correlator[i].is_time_to_output())?1:0;
	bit_i= bit_i << i;
	IQ_rdy_flag |= bit_i;  // set the bit for this channel
	
      }
  
  
  // also update TIC square NCO
  tic_clk_gen->OneClockUpdate();
  meas_rdy_flag = (tic_clk_gen->edge_happened() == 1)? true:false;

  if( meas_rdy_flag ) // rising edge happened
    {// update measurement for activated channel
      for( int i=0; i<12; i++)
	if( ch_activated[i] )
	  {
	    
	    ch12_meas[i].code_phase = ch12_correlator[i].get_code_NCO_phase();
	    ch12_meas[i].code_sub_phase= ch12_correlator[i].get_code_NCO_subPhase();
	  }
    }
  
}
