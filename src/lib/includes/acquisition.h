// This class is dedicated to finish signal acqusition, 
// either cold start( no information about current constellation)
// or warm start( Candidate satellite prnnums are known )
// FFT based softradio method will be added in the future

#ifndef ACQUISITION_MDL_H
#define ACQUISITION_MDL_H

#include <list>
#include <fstream>
#include "softgps.h"
#include "gpslibconfig.h"


//#define GPS_ACQMDL_RESULT
//#define GPS_ACQMDL_VERBOSE

class OnechCorrelator;

class GPS_Acquisition_mdl{
 public:
  GPS_Acquisition_mdl(const Controller_config&, const Correlator_config&);
  ~GPS_Acquisition_mdl();
  
  bool check_acquisition(double *); // acquisition 
  bool check_confirmation(corrresult_freqphidx *); //confirmation
  void check_exist_cold(double);    // check the existance of all GPS sat, like cold start
  void check_exist_warm(double);   // check the existance of GPS sat, read prn list from config file  
  bool check_exist(int, double);   // check the cenrain prn number

  void set_incoming_data(int*);
  void print_checkexist(void);
  
 protected:
 private:
  
  OnechCorrelator* correlator;

  int *incoming_data;
  int data_count_20ms;

  double threshhold;
  unsigned int carrier_ref, code_ref;
  correlator_IQ corr_meas;  // correlator IQ measurement for E,P,L phase
  double nco_norm_factor, point_per_chip;
  
  list <corrresult_phidx> possible_corr_ph;
  list <corrresult_freqphidx> possible_corr_freqph;
  int exist_satnum;   // the number of all the existing sats 
  satnum_freq_phidx ph_freq_info[32];// the exist_satnum-th elements contains their phase and carrier freq infos
  
  int possible_exist_satnum[32]; // the possible sat # read from config file
  
  bool Tryconfirmation(double,  int, int, int);
  
};

#endif
