#ifndef GPS_CONTROLLER_H
#define GPS_CONTROLLER_H

#include <list>
#include <fstream>
#include <iostream>
#include <sstream>
#include "softgps.h"
#include "gpslibconfig.h"
#include "multich_correlator.h"


//#define  GPS_CNTL_DEBUG
//#define  GPS_ACQ_RESULT

#define TRACKING_THRD  700


#define ST_RESET                  0x00
#define ST_CODE_LOCK_CANDIDATE    0x01
#define ST_CODE_LOCK              0x02
#define ST_CARRIER_LOCK           0x04
#define ST_BIT_SYNC               0x08
#define ST_FRAME_SYNC             0x10

class MultichCorrelator;
class GPS_msg_nav;
class GPS_nav_fix;
class GPS_pos_math;
// 6 states for controller's stage
// ST_RESET: 
//     controller has no idea about CA code phase, and Doppler shift
// ST_CODE_LOCK_CANDIDATE:
//     controller get one value greater than preset threshhold,
//     then it need confirm this one is the real signal's result
// ST_CODE_LOCK:
//     The signal really exists, and CA code phase is in [0, +/- 1/2chip],
//     Doppler shift is in [0, 500Hz], controller try to pull the carrier
//     frequency close to true doppler as much as possible.
// ST_CARRIER_LOCK:
//     The signal's carrier phase is locked with local carrier 
// ST_BIT_SYNC:
//     mesg data bit can be demodulated,...
// ST_FRAME_SYNC:
//     Preamble is searched and parity check passed


class GPSController{
public:
    GPSController( MultichCorrelator&,const Controller_config & );
    ~GPSController();
  
    void process_sig(int); // only be run after CheckExist() return true;

    char get_local_t_flag(void);
    void set_local_t_flag(double t);
    double get_local_t(void);

protected:
  
private:
  
    char local_t_flag;
    double local_gps_time;
    ostringstream  dbg_str;

    MultichCorrelator& correlator;
    GPS_msg_nav *gps_msgnav;
    GPS_nav_fix *gps_navfix;
    GPS_pos_math *gps_posmath;
    double threshhold;
    int correct_phase;
    unsigned int process_count;  // how many samples has been processed
    unsigned int ms_count;     // how many ms passed
    unsigned int carrier_ref, code_ref;
    double nco_norm_factor;
    allch_transtime  allchtrtime;          
#ifdef GPS_CNTL_DEBUG
    ofstream debughdle[12], gbl_debughdle;
#endif

    static const int confirm_m;         // = 10;
    static const int n_of_m_threshold;  // = 7;
    static const double PI,SpeedOfLight;
    static const int pullin_t_threshold;// = 1500; // 1.5s 
    static const int phase_test;        // = 500;  // 0.5s
    static const double cc_scale;       // = 1540;
    static const unsigned short bitMask[12]; // = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800};
    static const int NAG_PREAM;// = 0x22c00000;
    static const int pb1, pb2, pb3;//=0xbb1f3480,pb2=0x5d8f9a40,pb3=0xaec7cd00;
    static const int pb4, pb5, pb6;//=0x5763e680,pb5=0x6bb1f340,pb6=0x8b7a89c0;



    channel multich[12];
    int num_valid_ch;  // number of valid channels 

    void update_loop_state(int);
    void try_code_lock(int);
    void confirm_code_lock(int);
    void pullin_carrier_lock(int);
    void tracking_carrier_lock(int);
    int  check_more_parity(const unsigned int* src, int idx, int word_leng);
    void search_pream( int );
    void analysis_config(const Controller_config&, int*, double*, int*, unsigned int*);
};


#endif
