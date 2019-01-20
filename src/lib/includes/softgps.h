#ifndef SOFTGPS_H
#define SOFTGPS_H

#define SINU_NCO_NORM_BIT 8
#define SINU_NCO_NORM_VAL (1<<SINU_NCO_NORM_BIT)

#define LS_FIX            0
#define KF_FIX            1

#define SatMax            32

#define LOG_BUF_SIZE      4096

#include <string>
using namespace std;

/////////////////////////////////////////////////////////////////////////////
// This class is dedicated to provide some basic function to 
// do coordinate position-related functions
////////////////////////////////////////////////////////////////////////////

// structures definition 
/*
 * Cartesian vector.
 */

typedef struct 
{
    double x;                                            /* x co-ordinate. */
    double y;                                            /* y co-ordinate. */
    double z;                                            /* z co-ordinate. */
} cartstruc;

/*
 * Spherical vector.
 */

typedef struct 
{
    double el;                                   /* Elevation co-ordinate. */
    double az;                                     /* Azimuth co-ordinate. */
    double hgt;                                     /* Height co-ordinate. */
} sphstruc;

/*
 * Longitude, latitude and height position state vector.
 */

typedef struct 
{
    double lat;                                   /* Latitude co-ordinate. */
    double lon;                                  /* Longitude co-ordinate. */
    double hgt;                                     /* Height co-ordinate. */
} llhstruc;

typedef struct
{
    double n;
    double e;
    double d;
} nedvelstruc;

/*
 * GPS time structure.
 */

typedef struct
{
    int week;                                          /* GPS week number. */
    double sec;                                       /* Second into week. */
} gpstimestruc;


// configuration structure definition for "config.ini"

typedef struct Source_config_struct{
	std::string source_file;  // source file name
  double sampling_freq; // sampling freq, used to set half chip step
  unsigned int init_ph; // init phase(delay) for signal source
}Source_config;

typedef struct Correlator_config_struct{
  unsigned int prnnum;   // prn number
  double  sampling_freq; // sampling frequency
  double  carrier_freq;  // center carrier frequency excluding Doppler shift
}Correlator_config;
  
typedef struct Controller_config_struct{
  double threshold;
  std::string prnlist;
  std::string ca_freqlist;
  std::string cd_phaselist;
}Controller_config;


typedef struct SOFTGPS_config_struct{
  Source_config srce_config;
  Correlator_config corr_config;
  Controller_config cntl_config;
}SOFTGPS_config;


// configuration structure definition for "config.ini"
// These structures contains the identical info with SOFTGPS_config
// The reason here is the DLL interface can be problematic if 
// the interface argument using string. 

typedef struct DLL_Source_config_struct{
  char source_file[256];  // source file name
  double sampling_freq; // sampling freq, used to set half chip step
  unsigned int init_ph; // init phase(delay) for signal source
}DLL_Source_config;

typedef struct DLL_Correlator_config_struct{
  unsigned int prnnum;   // prn number
  double  sampling_freq; // sampling frequency
  double  carrier_freq;  // center carrier frequency excluding Doppler shift
}DLL_Correlator_config;

typedef struct DLL_Controller_config_struct{
  double threshold;
  char  prnlist[256];
  char  ca_freqlist[256];
  char  cd_phaselist[256];
}DLL_Controller_config;

typedef struct DLL_SOFTGPS_config_struct{
  DLL_Source_config srce_config;
  DLL_Correlator_config corr_config;
  DLL_Controller_config cntl_config;
}DLL_SOFTGPS_config;



// result from acquisition and confirmation
// contain the correlation result and related phase index

typedef struct corrresult_phidx_struct{
  double correlationresult;
  int phase_idx;
}corrresult_phidx, *P_corrresult_phidx;


typedef struct corrresult_freqphidx_struct{
  double carrier_freq;
  double correlationresult;
  int phase_idx;
}corrresult_freqphidx, *P_corrresult_freqphidx;


typedef struct satnum_freq_phidx_struct{
  int prnnum;
  corrresult_freqphidx freq_corrres_phidx;
}satnum_freq_phidx;

typedef struct correlator_IQ_struct{
  double early_I, early_Q;
  double prompt_I, prompt_Q;
  double late_I, late_Q;
}correlator_IQ, *P_correlator_IQ;

typedef struct correlator_meas_struct{
  int code_phase;
  int code_sub_phase;
  int carrier_cycle;
  int carrier_sub_cycle;
} correlator_meas, *P_correlator_meas;

enum GPScontrollerState {
  ACQUISITION,
  CONFIRMATION,
  PULLIN,
  TRACKING};


typedef struct channel_struct{
  char prn;
  int dbg_count;
  int process_state;

  int code_freq, carrier_freq;
  short acq_codes,n_freq,del_freq;
  long double tr_time;
  double pseudorange;
  correlator_IQ corr_meas;

  // variables required by CONFIRM
  int  n_threshold, i_confirmation;

  // variables needed by pullin state processing
  int pullin_chms; // the ms count during pulling state
   double dfreq, theta;
  double CdLI, CrLI;
  double EML_tau;
  double corr_sum, corr_sum_avg; 
  double theta_err;
  double phase_change_int;
  double old_prompt_I, old_prompt_Q;
  int ms_count;
  // variables needed by tracking state processing
  /*
  double q_sum_20ms_E, i_sum_20ms_E;
  double q_sum_20ms_L, i_sum_20ms_L;*/
  double q_sum_20ms_P, i_sum_20ms_P; 
  double dcarr;

  unsigned int tlm_tst,how_tst,nav_msg[302],msg_count, TIC_mscount;
  int subframe_i, TOW, TLM, z_count;
  double tr_time_HOW;

  unsigned int ms_sign;
  double azim, elev;
  double sv_pos[3], sv_vel[3];
} channel;

typedef struct EphInfo_struct{        // Precise orbital parameters
  int prn; 
  int valid;

  int iode;     /*Issue of Data(Ephemeris)   */
  int iodc;     /*Issue of Clock(Data)       */
  int ura;      /* SV Accuracy */
  int health;   /* SV Health */
  int week;     /* GPS Week */

  double dn;    /*Mean motion difference from computed value */
  double tgd;   /* L1-L2  correction term */
  double toe;   /*Reference time ephemeris */
  double toc;   /* clock data reference time in seconds */
  double omegadot; /*Rrate of right ascension */
  double idot;  /* Rate of inclination angle */
  double cuc,cus,crc,crs,cic,cis;  /* Harmonic correction terms */
  double ma;    /* mean anomaly at reference time */
  double ety;     /* Eccentricity */
  double sqra;  /* square root of the semi-major Axis */
  double w0;    /* Longitude of ascending node of orbit plane at weekly epoch */
  double inc0;  /* Inclination angle at reference time */
  double w;     /* argument of perigee */
  double wm;    /* mean angular velocity of sat, called Mean Motion */
  double af0,af1,af2; /* Apparent SV clock correction terms */
  double bclk;

} EphInfo;

// struct used to pass prn # and transfer bit time
typedef struct transtime_struct{
  int prn;
  long double tr_time;
  double dopp_f;
}transtime;
typedef struct allch_transtime_struct{
  int ch_count;
  double local_t;
  transtime ch_transtime[12];
}allch_transtime, *PT_allch_transtime;

typedef struct sat_tr_pos_struct{
  int prn;
  long double delta_time;
  long double sv_trans_time;
  double pos[3];
  double vel[3];
  double doppler_freq;
}sat_tr_pv;

typedef struct ls_group_meas_struct{
    unsigned int sv_count;
    double local_t;
    sat_tr_pv  meas[12]; // at most 12 svs
}ls_group_meas;

// structure to store the navigation information for each epoch

typedef struct nav_state_struct{
    // pos infos
    cartstruc pos;
    sphstruc  sph_pos;
    llhstruc  llh_pos;

    // vel infos
    cartstruc   vel;
    nedvelstruc ned_vel; 
    // DOP infos
    double hdop;
    double vdop;
    double pdop;
    double gdop;
    
    // time infos
    double clk_bias;
    double clk_drift;
    gpstimestruc gps_time;
    int    tic_count;
    double tic_period;

    // fix type
    int fix_type;       //0: ls fix; 1: kf fix;
    int sv_count;
}nav_state, *PT_nav_state;

// measurement definition for Kalman Filter fix
typedef struct kf_measurement_struct{
    unsigned int prn; 
    unsigned int type;  // 0: only pseudorange; 1: both pseudorange and doppler
    double doppler;
    double pseudo_range;
    double sv_vel[3];
    double sv_pos[3];
    double h_vec[3];
}kf_measurement;

typedef struct kf_group_meas_struct{
    unsigned int sv_count;
    double local_t;
    kf_measurement meas[12]; // at most 12 measurements
}kf_group_meas;

typedef struct {
    char buf[LOG_BUF_SIZE];
    int   wr_idx, rd_idx;
}log_circular_buf;


extern void log_msg(string st);

#endif
