#ifndef GPS_MSG_NAV_H
#define GPS_MSG_NAV_H

#include "softgps.h"
#include "gpslibconfig.h"
#include <fstream>
#include <sstream>

//#define GPS_MSGNAV_DBG
#define rotl(x,y)  (((x)<<(y))|((x)>>(32-(y))))

/////////////////////////////////////////////////////////////////////////////
// This class is dedicated to process the navigation message
// and PVT resolution when ephemeris data  and 
//                         pseudorange & Doppler measurement available
////////////////////////////////////////////////////////////////////////////

typedef struct navmsg_par_struct{
  unsigned int msg[5][10];
  unsigned int par[5];
}navmsg_par, *PT_navmsg_par;


class GPS_msg_nav{
 public:
  GPS_msg_nav();
  ~GPS_msg_nav();
  
  void set_prnlist(int, unsigned int);
  void process_navmsg(const unsigned int*, int, int);
  bool eph_valid(int);
  EphInfo* get_eph_pt(int);
 protected:

 private:
  
  unsigned int nav_msg_bit[302];
  unsigned int nav_msg[12][10]; // 12-channels' one subframe of nav message
  unsigned int p_error[12]; 
  int prev_bit[2];                     
  navmsg_par  navmsgparity[12];
  EphInfo  Ephemeris[SatMax+1];
  unsigned int ch_prn[12];
  bool eph_set_flag ;           // indicate if we set new ephemeris data
  // if this flag is true, we need to store them before exit

#ifdef GPS_MSGNAV_DBG
  ofstream debughdle;
#endif
  ostringstream  dbg_str;
  // binary constants for nav message decoding
  //
  static const double  c_2p12;// = 4096;
  static const double  c_2p4;//  = 16;
  static const double  c_2m5;//  = 0.03125;
  static const double  c_2m11;// = 4.8828125e-4;
  static const double  c_2m19;// = 1.9073486328125e-6;
  static const double  c_2m20;// = 9.5367431640625e-7;
  static const double  c_2m21;// = 4.76837158203125e-7;
  static const double  c_2m23;// = 1.19209289550781e-7;
  static const double  c_2m24;// = 5.96046447753906e-8;
  static const double  c_2m27;// = 7.45058059692383e-9;
  static const double  c_2m29;// = 1.86264514923096e-9;
  static const double  c_2m30;// = 9.31322574615479e-10;
  static const double  c_2m31;// = 4.65661287307739e-10;
  static const double  c_2m33;// = 1.16415321826935E-10;
  static const double  c_2m38;// = 3.63797880709171e-12;
  static const double  c_2m43;// = 1.13686837721616e-13;
  static const double  c_2m50;// = 8.881784197e-16;
  static const double  c_2m55;// = 2.77555756156289e-17;
  static const double PI;
  static const double lambda;
  static const double SemiMajorAxis,    EccentrSquared,
    OneMinusEccentrSquared,    OmegaDotEarth,    SpeedOfLight;
    

  //for parity check calculation use
  static const int pb1, pb2, pb3;//=0x3b1f3480,pb2=0x1d8f9a40,pb3=0x2ec7cd00;
  static const int pb4, pb5, pb6;//=0x1763e680,pb5=0x2bb1f340,pb6=0x0b7a89c0;

  static const int SecPerHour;//=3600;
  static const int SecPerDay;//=86400;
  static const int SecPerWeek;//=604800;
  

  void copy_navmsg(const unsigned int *, int, int);
  void parity_check_channel(int);
  int parity_check_word(unsigned int gpsword);
  int read_frame_num(int , int);
  void read_allsubframe(int );
  void save_eph_bin(void);
  void load_eph_bin(void);
};

#endif
