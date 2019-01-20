#ifndef MULTICH_CORRELATOR_H
#define MULTICH_CORRELATOR_H

#include "softgps.h"
#include "onech_correlator.h"

//forwart class declaration
class SquareNCO;  // TIC signal generator

class MultichCorrelator{

 public:

  MultichCorrelator(const Correlator_config&);
  ~MultichCorrelator();

  void OneClockUpdate(int); //input is the IF data sample of current instant

  //activate the certain channel
  void activate_ch(int );

  // set and get prn number for the certain channel
  void set_prn_num(int, unsigned int);
  unsigned int get_prn_num(int) const;
  // reset certain channel
  void reset_ch(int);

  double get_ticperiod(void);

  //set and get the phase step of carrier NCO for certain channel
  void set_carrier_NCO_step(int, unsigned int);
  unsigned int get_carrier_NCO_step(int) const;
  
  //set and get the phase step of code NCO for certain channel
  void set_code_NCO_step( int, unsigned int);
  unsigned int get_code_NCO_step(int) const;

  // get the EPL phase of the code generator for certain channel
  int get_code_NCO_phase(int) const;
  int get_code_NCO_subPhase(int) const;

  // get measurement for certain channel
  void get_IQ_measure(int, P_correlator_IQ);

  // check if its time to output I&D for certain channel
  bool is_time_to_get_IQ(int) const;

  // check if its time to generate measurement 
  bool is_time_to_get_meas( ) const;

  // slew some chip for the code phse for certain channel
  void slew_half_chip(int, unsigned int);
  
  // set and get sampling frequency
  void set_sampling_freq(double);
  double get_sampling_freq() const;
  // set and get the carrier frequency withoud Doppler shift
  void set_carrier_freq(double);
  double get_carrier_freq() const;
  
  
  //set the TIC period length
  void set_TIC(unsigned int);
  
  unsigned short get_IQ_rdy_flag() const;
  

 private:
  OnechCorrelator  ch12_correlator[12];  // 12 channels
  correlator_meas ch12_meas[12];
  char ch_activated[12];  // activated flag of all 12 channels
  SquareNCO* tic_clk_gen;

  int old_ticvalue, ticvalue;
  double sampling_freq, carrier_ref, code_ref;
  double nco_norm_factor, tic_period;
  bool meas_rdy_flag;
  unsigned short IQ_rdy_flag; // flag to indicate the IQ meas ready or not for each channel
                              // each bit indicates one channel
};

#endif
