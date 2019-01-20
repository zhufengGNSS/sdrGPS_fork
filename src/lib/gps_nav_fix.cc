/*:gps_nav_fix.cc
*******************************************************
* GPS position and navigation fix solution
* 
* This class is used to process the measurements from tracking loop and
* finish the nav fix functions,
* including LS and KF algorithm. 
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Jan, 2005
*******************************************************/
#include "./includes/gps_nav_fix.h"
#include "./includes/gps_nav_msg.h"
#include "./includes/pos_math.h"
#include "./includes/kalman_filter.h"

#include <iostream>
#include <math.h>
#include <string.h>


const double GPS_nav_fix::SemiMajorAxis=6378137.0L;
const double GPS_nav_fix::EccentrSquared=0.00669437999L;
const double GPS_nav_fix::OneMinusEccentrSquared=0.993305620010000L;
const double GPS_nav_fix::OmegaDotEarth=7.292115146E-5L;
const double GPS_nav_fix::SpeedOfLight = 2.99792458e8;
const double GPS_nav_fix::lambda = .1902936728;

GPS_nav_fix::GPS_nav_fix( GPS_msg_nav &gpsmsg,GPS_pos_math& gps_math ):gps_navmsg(gpsmsg),posmath_ref(gps_math)
{
  nav_kf = new Kalman(0, posmath_ref, nav_matrix);
#ifdef GPS_NAVFIX_DBG
  {
    time_t file_time;
    char buff[40];
    ostringstream filenm(" ");
    file_time = time(NULL);
    filenm<<"./data/ls_fix"; 		
    strftime(buff,40, "_%d%m%Y_%H%M.m",
	     localtime(&file_time));
    filenm<<buff;
    
    debughdle.open(filenm.str().c_str());
    if( !debughdle.is_open() )
      cerr <<"Failed to open debug file for GPS msgnav class" << endl;
  }

#endif
#ifdef GPS_NAVFIX_DBG1
  memset((void*)eph_log_flag, 0, 33*sizeof(char));
#endif
  dbg_str.str("");
  rcvr.tic_count = 0;
  rcvr.fix_type  = LS_FIX; 
  init_kf_flag = false;
}
GPS_nav_fix::~GPS_nav_fix()
{
  delete nav_kf;
#ifdef GPS_NAVFIX_DBG
  if( debughdle.is_open() )
    debughdle.close();
#endif
}

void GPS_nav_fix::increase_tic_count(int n )
{
  rcvr.tic_count += n;
}

// calculate sat position using ephemeris data and time 
void GPS_nav_fix::satpv_ephinfo(EphInfo *EphPtr, double Time, sat_tr_pv *spv)
{
  double d_toc,bclk,tc, d_toe, ei, ea, diff, aol, ta;
  double delr, delal, delinc, r, inc, la, xp, yp;
  double Ek_dot, Phik_dot,rk_dot, uk_dot, ik_dot,xk_dot, yk_dot, temp_v;
  double cos_uk, sin_uk, sin_2pk, cos_2pk, sin_ek, cos_ek, sin_ik, cos_ik, sin_ok, cos_ok;

    
  d_toc = Time - EphPtr->toc;
  if(d_toc>302400.0)
    d_toc = d_toc - 604800.0;
  else if(d_toc<-302400.0)
    d_toc = d_toc + 604800.0;

  bclk = EphPtr->af0 + EphPtr->af1*d_toc + EphPtr->af2*d_toc*d_toc
    -EphPtr->tgd;
  tc = Time - bclk;

#ifdef GPS_NAVFIX_DBG1

  /*
    % af0       =  3.355182707310E-05;
    % af1       =  5.570655048359E-12;
    % af2       =  0.000000000000E+00;
    % toc       =  1.656000000000E+05;
    % toe       =  1.656000000000E+05;
    % Delta_n   =  5.340579599434E-09*pi;
    % cuc       =  6.761401891708E-07;
    % cus       =  7.819384336472E-06;
    % cic       =  -2.160668373108E-07;
    % cis       =  -1.117587089539E-07;
    % crc       =  2.096875000000E+02;
    % crs       =  1.309375000000E+01;
    % ecc       =  9.196566301398E-03;
    % sqrta     =  5.153733177185E+03;
    % m0        =  2.108562288780E+00*pi;
    % Omega_0   =  5.583032574002E-02*pi;
    % in0       =  9.263470653421E-01;
    % om_small  =  7.333881984822E-01*pi;
    % Omega_dot =  -8.344276144023E-09*pi;
    % idot      =  3.092985978138E-10*pi;
    % tgd       =  -4.190951585770E-09;
  */
  if( !eph_log_flag[EphPtr->prn-1] )
    {
      debughdle.precision(16);
      debughdle << "PRN       =  "<< EphPtr->prn <<";" <<endl ;
      debughdle << "af0       =  "<< EphPtr->af0 <<";" <<endl ;
      debughdle << "af1       =  "<< EphPtr->af1 <<";" <<endl ;
      debughdle << "af2       =  "<< EphPtr->af2 <<";" <<endl ;
      debughdle << "toc       =  "<< EphPtr->toc <<";" <<endl ;
      debughdle << "toe       =  "<< EphPtr->toe <<";" <<endl ;
      debughdle << "Delta_n   =  "<< EphPtr->dn  <<";" <<endl ;
      debughdle << "cuc       =  "<< EphPtr->cuc <<";" <<endl ;
      debughdle << "cus       =  "<< EphPtr->cus <<";" <<endl ;
      debughdle << "cic       =  "<< EphPtr->cic <<";" <<endl ;
      debughdle << "cis       =  "<< EphPtr->cis <<";" <<endl ;
      debughdle << "crc       =  "<< EphPtr->crc <<";" <<endl ;
      debughdle << "crs       =  "<< EphPtr->crs <<";" <<endl ;
      debughdle << "ecc       =  "<< EphPtr->ety <<";" <<endl ;
      debughdle << "sqrta     =  "<< EphPtr->sqra<<";" <<endl ;
      debughdle << "m0        =  "<< EphPtr->ma  <<";" <<endl ;
      debughdle << "Omega_0   =  "<< EphPtr->w0  <<";" <<endl ;
      debughdle << "in0       =  "<< EphPtr->inc0<<";" <<endl ;
      debughdle << "om_small  =  "<< EphPtr->w   <<";" <<endl ;
      debughdle << "Omega_dot =  "<< EphPtr->omegadot<<";"<<endl ;
      debughdle << "idot      =  "<< EphPtr->idot<<";" <<endl ;
      debughdle << "tgd       =  "<< EphPtr->tgd <<";" <<endl ;

      eph_log_flag[EphPtr->prn-1] = 1;
    }
  /*
    debughdle <<"toe     = " <<EphPtr->toe << " ;" <<endl;
    debughdle <<"Delta_n = " <<EphPtr->dn  << " ;" <<endl;
    debughdle <<"cuc     = " <<EphPtr->cuc << " ;" <<endl;
    debughdle <<"cus     = " <<EphPtr->cus << " ;" <<endl;
    debughdle <<"cic     = " <<EphPtr->cic << " ;" <<endl;
    debughdle <<"cis     = " <<EphPtr->cis << " ;" <<endl;
    debughdle <<"crc     = " <<EphPtr->crc << " ;" <<endl;
    debughdle <<"crs     = " <<EphPtr->crs << " ;" <<endl;
    debughdle <<"ecc     = " <<EphPtr->ety << " ;" <<endl;
    debughdle <<"sqrta   = " <<EphPtr->sqra<< " ;" <<endl;
    debughdle <<"m0      = " <<EphPtr->ma  << " ;" <<endl;
    debughdle <<"Omega_0 = " <<EphPtr->w0 <<  " ;" <<endl;
    debughdle <<"in0     = " <<EphPtr->inc0<< " ;" <<endl;
    debughdle <<"om_small= " <<EphPtr->w   << " ;" <<endl;
    debughdle <<"Omega_dot=" <<EphPtr->omegadot<<" ;" <<endl;
    debughdle <<"idot    = " <<EphPtr->idot<< " ;" <<endl;
    debughdle <<"tt      = " <<tc <<" ;" <<endl;
  */
#endif 

    
  d_toe = tc - EphPtr->toe;
  if(d_toe>302400.0)
    d_toe = d_toe-604800.0;
  else if(d_toe<-302400.0)
    d_toe = d_toe+604800.0;

  ei = EphPtr->ma + d_toe*(EphPtr->wm + EphPtr->dn);
  ea = ei;
  do
    {
      diff = (ei - (ea - EphPtr->ety*sin(ea)))/(1.0E0 - EphPtr->ety*cos(ea));
      ea = ea + diff;
    } while (fabs(diff) > 1.0e-12 );
    
  //prepare for the sat's velocity 
  Ek_dot = (EphPtr->wm + EphPtr->dn)/(1-EphPtr->ety*cos(ea));
    
  bclk = bclk + 4.442807633E-10*EphPtr->ety*EphPtr->sqra*sin(ea);
  EphPtr->bclk = bclk;

  //     ea is the eccentric anomaly
  // ea ==> ek
  sin_ek = sin(ea);
  cos_ek = cos(ea);
    
  ta = atan2(sqrt(1.00-pow(EphPtr->ety,2))*sin(ea),cos(ea)-EphPtr->ety);

  // TA IS THE TRUE ANOMALY (ANGLE FROM PERIGEE)
  // aol ==> Phi_k
  aol = ta + EphPtr->w;
  sin_2pk = sin(2.0*aol);
  cos_2pk = cos(2.0*aol);
  Phik_dot = sqrt(1.0-EphPtr->ety*EphPtr->ety)*Ek_dot/(1-EphPtr->ety*cos_ek);
  //     AOL IS THE ARGUMENT OF LATITUDE OF THE SATELLITE

  //     calculate the second harmonic perturbations of the orbit

  delr  = EphPtr->crc*cos(2.0*aol) + EphPtr->crs*sin(2.0*aol);
  delal = EphPtr->cuc*cos(2.0*aol) + EphPtr->cus*sin(2.0*aol);
  delinc= EphPtr->cic*cos(2.0*aol) + EphPtr->cis*sin(2.0*aol);

  //     R IS THE RADIUS OF SATELLITE ORBIT AT TIME T
  r = pow(EphPtr->sqra,2)*(1.00 - EphPtr->ety*cos(ea)) + delr;
  //prepare for the sat's velocity
  rk_dot = pow(EphPtr->sqra,2)*EphPtr->ety*sin_ek*Ek_dot +
    2.0*(EphPtr->crs*cos_2pk-EphPtr->crc*sin_2pk)*Phik_dot;
    
  aol = aol + delal;  // now aol ==> uk
  sin_uk = sin(aol);
  cos_uk = cos(aol);
  // prepare for the sat's velocity 
  uk_dot = ( 1+2*EphPtr->cus*cos_2pk-2*EphPtr->cuc*sin_2pk)*Phik_dot;

  // inc ==> ik
  inc = EphPtr->inc0 + delinc + EphPtr->idot*d_toe;
  sin_ik = sin(inc);
  cos_ik = cos(inc);
  // prepare for the sat's velocity
  ik_dot = EphPtr->idot + 2.0*(EphPtr->cis*cos_2pk - EphPtr->cic*sin_2pk)*Phik_dot;

  //     LA IS THE CORRECTED LONGITUDE OF THE ASCENDING NODE
  // la ==> omega_k
  la = EphPtr->w0 + (EphPtr->omegadot - OmegaDotEarth)*d_toe
    - OmegaDotEarth*EphPtr->toe;
  sin_ok = sin(la);
  cos_ok = cos(la);

  xp = r*cos(aol);
  yp = r*sin(aol);

  //prepare for the sat's velocity
  xk_dot = rk_dot*cos_uk - yp*uk_dot;
  yk_dot = rk_dot*sin_uk + xp*uk_dot;
    
  spv->pos[0] = xp*cos(la) - yp*cos(inc)*sin(la);
  spv->pos[1] = xp*sin(la) + yp*cos(inc)*cos(la);
  spv->pos[2] = yp*sin(inc);

  temp_v = yk_dot*cos_ik - yp*sin_ik*ik_dot;

  spv->vel[0] = -(EphPtr->omegadot-OmegaDotEarth)*spv->pos[1] +
    xk_dot*cos_ok - temp_v*sin_ok;
  spv->vel[1] = (EphPtr->omegadot-OmegaDotEarth)*spv->pos[0]+
    xk_dot*sin_ok + temp_v*cos_ok;
  spv->vel[2] = yp*cos_ik*ik_dot + yk_dot*sin_ik;
}

 // calculate the geodetic vector, earth rotation effect considered
void GPS_nav_fix::calc_h_vec(double *h, double sv_pos[3], cartstruc usr_pos)
{
  double usr_p[3];
  usr_p[0] = usr_pos.x;
  usr_p[1] = usr_pos.y;
  usr_p[2] = usr_pos.z;
  calc_h_vec(h, sv_pos, usr_p);
}

void GPS_nav_fix::calc_h_vec(double *h, double sv_pos[3], double usr_pos[3])
{
  cartstruc sv_p, usr_p, delta_p;
  double alpha;
  double r;
  sv_p.x = sv_pos[0];
  sv_p.y = sv_pos[1];
  sv_p.z = sv_pos[2];

  usr_p.x = usr_pos[0];
  usr_p.y = usr_pos[1];
  usr_p.z = usr_pos[2];

  delta_p = posmath_ref.sub_vector( sv_p, usr_p);
  r = posmath_ref.vec_length( delta_p.x, delta_p.y, delta_p.z);

  alpha = (r/SpeedOfLight)*OmegaDotEarth;

  r  = posmath_ref.vec_length(sv_p.x*cos(alpha)-sv_p.y*sin(alpha)-usr_p.x,
			      sv_p.y*cos(alpha)+sv_p.x*sin(alpha)-usr_p.y,
			      sv_p.z-usr_p.z);
    
  h[0] = (sv_p.x*cos(alpha) - sv_p.y*sin(alpha) - usr_p.x)/r;
  h[1] = (sv_p.y*cos(alpha) + sv_p.x*sin(alpha) - usr_p.y)/r;
  h[2] = (sv_p.z - usr_p.z)/r;
}
// Pos and Vel fix function
// This function take the argument of pseudorange meas and doppler meas
// Calculate the pos fix by LS method.
// est_time: the estimated local time when TIC happened

//int GPS_nav_fix::navfix(const sat_tr_pv sv_meas[12], int sv_count, double est_time, double *clk)
int GPS_nav_fix::nav_ls_fix(const ls_group_meas &ls_meas, double *clk)
{
  // next we will use recursion to get current location 
  int nits, i, sv_count; 
  double clk_bias,x,y,z;
  double  dd[4][4],r,bm[12],br[4],correct_mag; //, ms[4][12];
  double ms_t[48], inv_ms[16], ms[48], pm[48];;
  double alpha; 
  double AccurateTime;
  const sat_tr_pv *sv_meas;

  sv_meas     = ls_meas.meas;
  sv_count    = ls_meas.sv_count;
  correct_mag = 100000.0;
  
  nits=0;  // the count of iteration number
  clk_bias=0.0;   // initial clock bias

  x=0.;    // initial position , x,y,z
  y=0.;
  z=0.;
  do
    {
      for(i=0;i<sv_count;i++)
	{
	  //   Compute range in ECI at the time of arrival at the receiver
            
	  alpha    = (clk_bias - sv_meas[i].delta_time)*OmegaDotEarth;
	  r        = sqrt(pow(sv_meas[i].pos[0]*cos(alpha)-sv_meas[i].pos[1]*sin(alpha)-x,2)+
			  pow(sv_meas[i].pos[1]*cos(alpha)+sv_meas[i].pos[0]*sin(alpha)-y,2)+
			  pow(sv_meas[i].pos[2]-z,2));
	  bm[i]    = r-(sv_meas[i].delta_time - clk_bias)*SpeedOfLight;
	  ms[0*sv_count+i]=(sv_meas[i].pos[0]*cos(alpha)-sv_meas[i].pos[1]*sin(alpha)-x)/r;
	  ms[1*sv_count+i]=(sv_meas[i].pos[1]*cos(alpha)+sv_meas[i].pos[0]*sin(alpha)-y)/r;
	  ms[2*sv_count+i]=(sv_meas[i].pos[2]-z)/r;
	  ms[3*sv_count+i]=1.0;

        }
      nav_matrix.matrix_transpose((double*)ms_t, (double*)ms, (int)4, (int)sv_count);
      nav_matrix.matrix_mul((double*)inv_ms, (double*)ms, (double*)ms_t, 4,sv_count,4);
      if( !nav_matrix.matrix_inverse((double*)dd, (double*)inv_ms, 4))
        {

#ifdef GPS_LOG_CONSOLE
	  cout <<"Matrix Singular "<<endl;
#else

	  dbg_str<<"MSG Maxtrix Singular!" << endl;
	  log_msg(dbg_str.str());
	  dbg_str.str("");
#endif

	}
      else
	{
	  nav_matrix.matrix_mul(pm, (double*)dd, ms, 4,4,sv_count);
	  nav_matrix.matrix_mul(br, pm, bm, 4, sv_count, 1);

	  nits++;
	  x=x+br[0];
	  y=y+br[1];
	  z=z+br[2];
	  clk_bias=clk_bias-br[3]/SpeedOfLight;
	  correct_mag=sqrt(br[1]*br[1]+br[2]*br[2]+br[0]*br[0]);
	}
    } while ( correct_mag > 0.01 && correct_mag < 1.e8 && nits < 10);

  AccurateTime = ls_meas.local_t - clk_bias;
  *clk = AccurateTime;

  rcvr.pos.x = x;
  rcvr.pos.y = y;
  rcvr.pos.z = z;
    
  rcvr.clk_bias = clk_bias;
  rcvr.gps_time.sec = AccurateTime;
    
  // for velocity
  for (i=0;i<sv_count;i++)
    {
      alpha = ( AccurateTime - sv_meas[i].sv_trans_time)*OmegaDotEarth;
      r     = sqrt(pow(sv_meas[i].pos[0]*cos(alpha)-sv_meas[i].pos[1]*sin(alpha)-x,2)+
		   pow(sv_meas[i].pos[1]*cos(alpha)+sv_meas[i].pos[0]*sin(alpha)-y,2)+
		   pow(sv_meas[i].pos[2]-z,2));

      bm[i] = ((sv_meas[i].pos[0]*cos(alpha)-sv_meas[i].pos[1]*sin(alpha)-x)*(sv_meas[i].vel[0])+
	       (sv_meas[i].pos[1]*cos(alpha)+sv_meas[i].pos[0]*sin(alpha)-y)*(sv_meas[i].vel[1])+
	       (sv_meas[i].pos[2]-z)*sv_meas[i].vel[2])/r - sv_meas[i].doppler_freq*lambda;

    }
  nav_matrix.matrix_mul(br, pm, bm, 4,sv_count, 1);
    
  rcvr.vel.x = br[0]; // + y*OmegaDotEarth;
  rcvr.vel.y = br[1]; // - x*OmegaDotEarth;
  rcvr.vel.z = br[2];
  rcvr.clk_drift  = -br[3]/SpeedOfLight*1000000.0;
    
  //calculate DOPs
  rcvr.hdop = sqrt(dd[0][0] + dd[1][1]);
  rcvr.vdop = sqrt(dd[2][2]);
  rcvr.pdop = sqrt(dd[0][0] + dd[1][1] + dd[2][2]);
  rcvr.gdop = sqrt(dd[0][0] + dd[1][1] + dd[2][2] + dd[3][3]);
  posmath_ref.ecef2llh(&rcvr.pos, &rcvr.llh_pos);

  rcvr.fix_type = LS_FIX;

  dbg_str.precision(16);
  dbg_str   << rcvr.pos.x        << " " 
	    << rcvr.pos.y        << " " 
	    << rcvr.pos.z        << " " 
	    << rcvr.llh_pos.lat  << " " 
	    << rcvr.llh_pos.lon  << " " 
	    << rcvr.llh_pos.hgt  << " " 
	    << sv_count          << " "
	    << rcvr.vel.x      << " "
	    << rcvr.vel.y      << " " 
	    << rcvr.vel.z      << " " 
	    << (rcvr.clk_drift*SpeedOfLight*1e-6)  << " " 
	    << (rcvr.clk_bias*SpeedOfLight)   << " "
	    << rcvr.hdop       << " "
	    << rcvr.vdop       << " "
	    << rcvr.pdop       << " "
	    << rcvr.gdop       << " " 
	    << rcvr.gps_time.sec<< " "
	    << endl;
#ifdef GPS_NAVFIX_DBG
  debughdle <<dbg_str.str();
#endif

  log_msg("DATA LS " + dbg_str.str());
  dbg_str.str("");

  if( nits>10 )
    return 0;
  return 1;
}


// PVT resolution , 
// input argument is the all channels' transmision time
int GPS_nav_fix::pvt_resolve( const allch_transtime & allchtrtime, double *clk)
{
  long double min_trtime, max_trtime, local_t;
  int prn, i;
  EphInfo *ephPtr;
  sat_tr_pv sat_pvtpara;
  int ret_val; 

  // first check if all trans_time make sense? 
  min_trtime = max_trtime = allchtrtime.ch_transtime[0].tr_time; 
 
  for( i=1; i< allchtrtime.ch_count; i++) 
    { 
      if( allchtrtime.ch_transtime[i].tr_time > max_trtime) 
	max_trtime = allchtrtime.ch_transtime[i].tr_time; 
      if( allchtrtime.ch_transtime[i].tr_time < min_trtime) 
	min_trtime = allchtrtime.ch_transtime[i].tr_time; 
    } 
 
  if( fabs( max_trtime - min_trtime ) > 0.1 )    // doesn't make sense 
    return 0; 
 
  // if any channel is not ready , quit 
  for( i=0; i< allchtrtime.ch_count; i++) 
    { 
      prn = allchtrtime.ch_transtime[i].prn; 
      if( !gps_navmsg.eph_valid(prn) ) 
	{
#ifdef GPS_LOG_CONSOLE
	  cout <<"Err, prn " << prn << " eph not ready"<<endl;
#endif

	  return 0; 
	}
    } 

#ifdef GPS_LOG_CONSOLE
  cout <<"."<<flush;
#else
  dbg_str<<"MSG" << ".";
  log_msg(dbg_str.str());
  dbg_str.str("");
#endif
    
    
  if( allchtrtime.local_t<0.0 )
    {
      local_t = (long double)( max_trtime + 0.08);  // guess
    }
  else
    local_t = allchtrtime.local_t;
    
  // m_time = allchtrtime.local_t;

  //setup the KF and LS measurements
  kfmeas.sv_count = allchtrtime.ch_count;
  kfmeas.local_t  = local_t;
  lsmeas.sv_count = allchtrtime.ch_count;
  lsmeas.local_t  = local_t;
    
  for( i=0; i< allchtrtime.ch_count; i++)
    {
      prn = allchtrtime.ch_transtime[i].prn;
      ephPtr = gps_navmsg.get_eph_pt(prn);
      satpv_ephinfo(ephPtr,
		    allchtrtime.ch_transtime[i].tr_time ,
		    &sat_pvtpara);
      sat_pvtpara.prn = prn;

      //setup ls meas
      memcpy(&lsmeas.meas[i], &sat_pvtpara, sizeof(sat_tr_pv));
      lsmeas.meas[i].delta_time    = local_t - allchtrtime.ch_transtime[i].tr_time + ephPtr->bclk;
      lsmeas.meas[i].sv_trans_time = allchtrtime.ch_transtime[i].tr_time - ephPtr->bclk;
      lsmeas.meas[i].doppler_freq  = allchtrtime.ch_transtime[i].dopp_f;
        
      //setup kf_meas
      memcpy(kfmeas.meas[i].sv_pos, sat_pvtpara.pos, sizeof(double)*3);
      memcpy(kfmeas.meas[i].sv_vel, sat_pvtpara.vel, sizeof(double)*3);
      calc_h_vec(kfmeas.meas[i].h_vec, sat_pvtpara.pos, rcvr.pos);
      kfmeas.meas[i].prn = prn;
      kfmeas.meas[i].type   = 1;   // both meas( ps_range & doppler )
      kfmeas.meas[i].pseudo_range = lsmeas.meas[i].delta_time*SpeedOfLight;

      kfmeas.meas[i].doppler = -allchtrtime.ch_transtime[i].dopp_f*lambda +
	(kfmeas.meas[i].h_vec[0]*(sat_pvtpara.vel[0]/*+rcvr.pos.y*OmegaDotEarth*/)+
	 kfmeas.meas[i].h_vec[1]*(sat_pvtpara.vel[1]/*-rcvr.pos.x*OmegaDotEarth*/)+
	 kfmeas.meas[i].h_vec[2]*sat_pvtpara.vel[2]);
        
#ifdef GPS_NAVFIX_SV_DBGx
      debughdle.precision(16);
      debughdle << "Prn:" <<prn << " "
		<<"local_t: " << local_t
		<< " tr: " << allchtrtime.ch_transtime[i].tr_time
		<< " x: " << lsmeas.meas[i].pos[0]
		<< " y: " << lsmeas.meas[i].pos[1]
		<< " z: " << lsmeas.meas[i].pos[2]
		<< " vx:" << lsmeas.meas[i].vel[0]
		<< " vy:" << lsmeas.meas[i].vel[1]
		<< " vz:" << lsmeas.meas[i].vel[2]
		<< " dop:" <<lsmeas.meas[i].doppler_freq
		<< endl;
#endif

    }
    
  ret_val = nav_ls_fix(lsmeas, clk);

  
#ifdef GPS_NAVFIX_KF_DBGx
  debughdle<< "SV"<<kfmeas.sv_count<< " ";

  for( i=0; i< allchtrtime.ch_count; i++)
    {
      prn = allchtrtime.ch_transtime[i].prn; 
      debughdle.precision(16);
      debughdle<<  prn << " "
	       << kfmeas.meas[i].h_vec[0] << " "
	       << kfmeas.meas[i].h_vec[1] << " "
	       << kfmeas.meas[i].h_vec[2] << " "
	       << kfmeas.meas[i].sv_pos[0]<< " "
	       << kfmeas.meas[i].sv_pos[1]<< " "
	       << kfmeas.meas[i].sv_pos[2]<< " " 
	       << kfmeas.meas[i].pseudo_range << " "
	       << kfmeas.meas[i].doppler << " ";
      //<<endl
    }
    
  debughdle<< rcvr.clk_bias;
  debughdle<<endl;
#endif
    
  if( !init_kf_flag )
    {
      nav_kf->init_kf(rcvr);
      init_kf_flag = true;
    }
  else
    nav_kf->nav_kf_fix(kfmeas);

  return ret_val; 
}

void GPS_nav_fix::calc_azim_elev(int prn, double* elev_p, double* azim_p)
{
  int i;
  cartstruc  sv_pos;

  for( i=0; i<12; i++)
    if( lsmeas.meas[i].prn == prn)
      break;

  if( i==12 )
    return;

  sv_pos.x = lsmeas.meas[i].pos[0];
  sv_pos.y = lsmeas.meas[i].pos[1];
  sv_pos.z = lsmeas.meas[i].pos[2];

  posmath_ref.calc_elev_azim(rcvr.pos, sv_pos, elev_p, azim_p);

}
