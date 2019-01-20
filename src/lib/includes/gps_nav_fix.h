#ifndef GPS_NAV_FIX_H
#define GPS_NAV_FIX_H

#include "softgps.h"
#include <fstream>
#include <sstream>
#include "matrix.h"
#include "gpslibconfig.h"

//#define GPS_NAVFIX_DBG
//#define GPS_NAVFIX_KF_DBG
//#define GPS_NAVFIX_DBG1  /* Enable to log Eph data */

class GPS_pos_math;
class GPS_msg_nav;
class Kalman;

class GPS_nav_fix{
 public:
  GPS_nav_fix(GPS_msg_nav&, GPS_pos_math&);
  ~GPS_nav_fix();
  void increase_tic_count(int);
  int pvt_resolve( const allch_transtime &, double *);
  void calc_azim_elev(int , double*, double* );
    
 protected:

 private:
  void satpv_ephinfo(EphInfo*, double , sat_tr_pv*);
  void calc_h_vec(double *h, double sv_pos[3], double usr_pos[3]);
  void calc_h_vec(double *h, double sv_pos[3], cartstruc usr_p);
    
  int nav_ls_fix(const ls_group_meas&, double *);
    
#ifdef GPS_NAVFIX_DBG
  ofstream debughdle;
#endif
#ifdef GPS_NAVFIX_DBG1
  char eph_log_flag[33];
#endif
  ostringstream  dbg_str;
  GPS_msg_nav &gps_navmsg;
  GPS_pos_math &posmath_ref;
  Matrix  nav_matrix;
  Kalman  *nav_kf; 
  static const double SemiMajorAxis,    EccentrSquared, lambda,
    OneMinusEccentrSquared,    OmegaDotEarth,    SpeedOfLight;
  nav_state  rcvr;
  kf_group_meas kfmeas;
  ls_group_meas lsmeas;
  bool init_kf_flag;
};

#endif
