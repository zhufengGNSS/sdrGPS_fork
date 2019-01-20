#ifndef POS_MATH_H
#define POS_MATH_H

#include "softgps.h"
#include "gpslibconfig.h"
#include <fstream>

//#define POS_MATH_DBG

class GPS_pos_math{
 public:
  GPS_pos_math();
  ~GPS_pos_math();

  // subtraction of two vectors  
  cartstruc sub_vector(const cartstruc &a, const cartstruc &b);
  // length of vector
  double    vec_length(double x, double y, double z);
  // Longitude, latitude, altitude to ECEF pos transformation
  void      llh2ecef(const llhstruc *llh, cartstruc *xyz);
  // ECEF pos to longitude, latitude, altitude pos transformation
  void      ecef2llh(const cartstruc *xyz, llhstruc *llh);
  //Cartesian pos to spherical pos transformation
  void      cart2sph(const cartstruc *xyz, sphstruc *eah);
  //spherical pos to cartesian pos transformation
  void      sph2cart(const sphstruc *eah, cartstruc *xyz);
  // spherical vel to cartesian vel transformation
  void      sphV2cartV(const sphstruc *eah, const sphstruc *veleah, cartstruc *velxyz);
  // cartesian vel to spherical vel transformation
  void      cartV2sphV(const sphstruc *eah, const cartstruc *velxyz, sphstruc *veleah);
  // generate topocentric matrix based on current ecef position
  void      ecef_topocentric_matrix(const cartstruc *xyz, double t[3][3]);
  // ecef vector to topocentric frame transformation
  void      ecef2topo(const cartstruc *ecef, const double t[3][3], cartstruc *topo);
  // calculate elevation and azimuth of GPS SVs, given the pos
  void calc_elev_azim(const cartstruc &user_pos, 
		      const cartstruc &sv_pos, 
		      double *elev, double *azim);
 protected:

 private:

#ifdef POS_MATH_DBG
  ofstream debughdle;
#endif
  
  // binary constants 
  //
  static const double PI;
  static const double PI2DEG;
  static const double lambda;

  static const double SemiMajorAxis, 
    SemiMinorAxis,
    EccentrSquared,
    OneMinusEccentrSquared,
    OmegaDotEarth,    
    SpeedOfLight, 
    EarthEcc;
    
  static const int SecPerHour; //=3600;
  static const int SecPerDay; //=86400;
  static const int SecPerWeek; //=604800;

};

#endif
