/*:gps_controller.cc
*******************************************************
* GPS controller module implementation
* 
* This class is used to implement the control logic 
* for GPS acquisition & tracking. 
* This module acts as the uC interfacing with correlator
* Each time the correlator has new data dumped, this module should 
* process the data according to current state( acquisition, confirmation,
* pullin, tracking)
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Jan, 2005
*******************************************************/

#include "./includes/pos_math.h"
#include "./includes/gps_controller.h"
#include "./includes/multich_correlator.h"
#include "./includes/gps_nav_msg.h"
#include "./includes/gps_nav_fix.h"

#include <iomanip>
#include <algorithm>
#include <math.h>

const double GPSController::PI = 3.1415926;
const double GPSController::SpeedOfLight = 2.99792458e8;
const double GPSController::cc_scale = 1540.0;
const unsigned short GPSController::bitMask[12] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800};

const int GPSController::confirm_m = 10;
const int GPSController::n_of_m_threshold = 7;
const int GPSController::pullin_t_threshold = 1500; // 1.5s 
const int GPSController::phase_test = 500;  // 0.5s
const int GPSController::NAG_PREAM = 0x22c00000;
const int GPSController::pb1=0xbb1f3480;
const int GPSController::pb2=0x5d8f9a40;
const int GPSController::pb3=0xaec7cd00;
const int GPSController::pb4=0x5763e680;
const int GPSController::pb5=0x6bb1f340;
const int GPSController::pb6=0x8b7a89c0;

////////////////////////////////////////////////////////////////
// some static functions, not belongs to the GPSController class

static double Magnitude(double a1, double a2)
{
  return sqrt( a1*a1 + a2*a2 );
}

static int bit_sign(double a)
{
  if( a>0 ) return 1;
  else return 0;
}

static int xors(long pattern)
{
  int count,i;
  count=0;
  pattern=pattern>>6;
  for (i=0;i<=25;i++)
    {
      count+=int(pattern & 0x1);
      pattern=pattern>>1;
    }
  count=count%2;
  return(count);
}


static int exor(char bit, unsigned int parity)
{
  char i;
  int result, tmp_par;
  result=0;
  tmp_par = parity>>6;
  for (i=0;i<24;i++)
    {
      if( tmp_par & 0x1) result++;
      tmp_par = tmp_par>>1;
    }
  result=result%2;

  result=(bit ^ result) & 0x1;
  return(result);
}

//////////////////////////////////////////////////////////////

GPSController::GPSController( MultichCorrelator& corr, const Controller_config &cntl_cfg):
  correlator(corr)
{
  int prnlist[12], i;
  unsigned int prnnum; 
  double cafreqlist[12];
  int cdphaselist[12];

  for( i=0;i<12; i++)
    cdphaselist[i] = 0;

  process_count = 0; 
  local_t_flag  = 0;
  
  threshhold = cntl_cfg.threshold;  // should set through config.ini
  nco_norm_factor=((double)(0xFFFFFFFF))/corr.get_sampling_freq();
  carrier_ref = (unsigned int)(corr.get_carrier_freq()*nco_norm_factor);
  code_ref = (unsigned int)(2.046e6*nco_norm_factor);


  analysis_config(cntl_cfg, prnlist, cafreqlist, cdphaselist, &prnnum);
  num_valid_ch = (signed)prnnum;

  for( i=0 ;i<num_valid_ch; i++)
    {
      multich[i].process_state = ST_RESET;
      multich[i].acq_codes = 0;
      multich[i].prn = prnlist[i];
      multich[i].z_count = 0;
      correlator.activate_ch(i);
      correlator.set_prn_num(i, prnlist[i]);
      correlator.set_carrier_NCO_step(i,(unsigned int)(cafreqlist[i]*nco_norm_factor));
      multich[i].code_freq = correlator.get_code_NCO_step(i);
      multich[i].carrier_freq = correlator.get_carrier_NCO_step(i);
      multich[i].tr_time_HOW = 0;
      multich[i].subframe_i = -1;
      correlator.slew_half_chip(i, cdphaselist[i]);
#ifdef GPS_LOG_CONSOLE
      cout <<"Setup CH " << i << " , prn " << (int)multich[i].prn<<endl;
#else
      dbg_str<<"MSG Setup CH" << i << " , prn " << (int)multich[i].prn<<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif

    }
  
  // allocate memory for GPS_msg_nav object
  gps_posmath = new GPS_pos_math;
  gps_msgnav = new GPS_msg_nav;
  gps_navfix = new GPS_nav_fix(*gps_msgnav, *gps_posmath);

  for( i=0; i<num_valid_ch; i++)
    gps_msgnav->set_prnlist(i, prnlist[i]);

  dbg_str.str("");
  
#ifdef GPS_CNTL_DEBUG
  {
    time_t file_time;
    char buff[40];
    ostringstream filenm(" ");
    file_time = time(NULL);
    strftime(buff,40, "_%d%m%Y_%H%M.m",
	     localtime(&file_time));	    
	    
    for( i=0; i<num_valid_ch; i++)
      {
	filenm.seekp(0, ios::beg);
	filenm<<"./data/Channel"<<i<<buff;
	debughdle[i].open(filenm.str().c_str());
	if( !debughdle[i].is_open())
	  cerr <<"Failed to open gpscntl log file for channel " << i<<endl;
      }
	    
    filenm.seekp(0, ios::beg);
    filenm<<"./data/gbldbg"<<buff;
    gbl_debughdle.open(filenm.str().c_str());
    if( !gbl_debughdle.is_open() )
      cerr <<"Failed to open global debug file for GPS controller" << endl;

  }
#endif
  
}

GPSController::~GPSController()
{
  delete gps_msgnav;
  delete gps_navfix;
  delete gps_posmath;

#ifdef GPS_CNTL_DEBUG
  for( int i=0; i<num_valid_ch; i++)
    if(debughdle[i].is_open())
      debughdle[i].close();
  
  if( gbl_debughdle.is_open() )
    gbl_debughdle.close();
#endif

}

char GPSController::get_local_t_flag(void)
{
  return local_t_flag;
}

void GPSController::set_local_t_flag(double t)
{
  local_t_flag = 1;
  local_gps_time = t;
}
double GPSController::get_local_t(void)
{
  return local_gps_time;
}


void GPSController::analysis_config(const Controller_config& cntl_cfg, 
				    int* prnlist,
				    double* cafreq_list,
				    int* cdphase_list,
				    unsigned int* prncount)
{
  string tmpstr, tmpsubstr;
  unsigned int i=0, j=0, ti;
  string::size_type k=0;
  double ca_fs;
  int cd_ph;
  
  // next analyse the prnlist from config file
  tmpstr = cntl_cfg.prnlist+",";
  while( k!=string::npos)
    {
      i=0;
      //ignore the preceeding space or ","
      while( (tmpstr[i]==' ' || tmpstr[i]==',') && i<tmpstr.size())
	{
	  i++;
	}
      
      tmpstr = tmpstr.substr(i, tmpstr.size() -i);

      k=tmpstr.find(",",0);
      if( k!=string::npos)
	{
	  tmpsubstr=tmpstr.substr(0,k);
	  tmpstr = tmpstr.substr(k,tmpstr.size()-k);
	  ti=atoi(tmpsubstr.c_str());
	  if(ti>0 && ti<33)
	    {
	      prnlist[j] = ti;
	      j++;

	    }
	  
	}
    } // end of while(k!
  *prncount = j;
  
  //next analyze the carrier freq list

  j=0;
  k=0;
  tmpstr = cntl_cfg.ca_freqlist+",";

  while( k!=string::npos)
    {
      i=0;
      //ignore the preceeding space or ","
      while( (tmpstr[i]==' ' || tmpstr[i]==',') && i<tmpstr.size())
	{
	  i++;
	}
      
      tmpstr = tmpstr.substr(i, tmpstr.size() -i);

      k=tmpstr.find(",",0);
      if( k!=string::npos)
	{
	  tmpsubstr=tmpstr.substr(0,k);
	  tmpstr = tmpstr.substr(k,tmpstr.size()-k);
	  ca_fs=atof(tmpsubstr.c_str());
	  cafreq_list[j] = ca_fs;
	  j++;
	}
    } // end of while(k!
  if( *prncount != j)
    {
#ifdef GPS_LOG_CONSOLE
      cout<<"Warning: PRN list doesn't match CARRIER list !"<<endl;
      cout<<"PRN list: "<< cntl_cfg.prnlist <<endl;
      cout<<"Carrier List: "<< cntl_cfg.ca_freqlist <<endl;
#else 
      dbg_str<<"MSG Warning: PRN list doesn't match CARRIER list !"<<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
      dbg_str<<"MSG PRN list: "<< cntl_cfg.prnlist <<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
      dbg_str<<"MSG Carrier List: "<< cntl_cfg.ca_freqlist <<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif
    }
  // next analyze the cd phase list
  j=0;
  k=0;
  tmpstr = cntl_cfg.cd_phaselist + ",";
    
  while ( k!=string::npos)
    {
      i=0;
      while( (tmpstr[i]==' ' || tmpstr[i]==',') && i<tmpstr.size())
	{
	  i++;
	}
      tmpstr = tmpstr.substr(i, tmpstr.size()-i);
      k=tmpstr.find(",", 0);
      if( k!=string::npos)
	{
	  tmpsubstr = tmpstr.substr(0, k);
	  tmpstr = tmpstr.substr(k, tmpstr.size()-k);
	  cd_ph  = atoi(tmpsubstr.c_str());
	  cdphase_list[j] = cd_ph;
	  j++;
	}
    } // end of while(k!
  if( *prncount!=j )
    {
#ifdef GPS_LOG_CONSOLE
      cout << "Warning: PRN list doesn't match CDPHASE list !"<<endl;
      cout <<"PRN list: "<< cntl_cfg.prnlist<<endl;
      cout <<"CDPhase list: "<<cntl_cfg.cd_phaselist<<endl;
#else
      dbg_str<<"MSG Warning: PRN list doesn't match CDPHASE list !"<<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
      dbg_str<<"MSG PRN list: "<< cntl_cfg.prnlist<<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
      dbg_str<<"MSG CDPhase list: "<<cntl_cfg.cd_phaselist<<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif
    }
    
}
// process signal data sample after existance of certain GPS sat is guaranteed

void GPSController::process_sig(int sig_sam)
{
  int i=0; 
  unsigned short IQ_rdy_f=0; 
  bool tic_done=false;
  process_count++;

  correlator.OneClockUpdate(sig_sam);
  IQ_rdy_f = correlator.get_IQ_rdy_flag();
  tic_done = correlator.is_time_to_get_meas();

  for(i=0; i< num_valid_ch; i++)  // process different channels
    {
      if( IQ_rdy_f & bitMask[i] ) // check IQ_rdy flag
	{

	  correlator.get_IQ_measure(i, &multich[i].corr_meas);
	  update_loop_state(i);
	}
    }// end of for( 

  // collect all the transimission time for all available channels
  if(tic_done)
    {
      
      int ch_cnt=0;
      
      if(local_t_flag)
        {
	  local_gps_time += correlator.get_ticperiod();
	  //local_gps_time += process_count/correlator.get_sampling_freq();
	  process_count   = 0; //reset process_count 
        }

      gps_navfix->increase_tic_count(1);

      allchtrtime.ch_count = 0;
      for( i=0; i<num_valid_ch; i++)
	{
	  if( (multich[i].process_state&ST_CARRIER_LOCK) 
              &&(multich[i].process_state&ST_FRAME_SYNC)
              &&(multich[i].tr_time_HOW >0.0))
	    {
	      ch_cnt = allchtrtime.ch_count++;
	      allchtrtime.ch_transtime[ch_cnt].prn = multich[i].prn;
	      allchtrtime.ch_transtime[ch_cnt].tr_time =
		multich[i].tr_time_HOW+
		multich[i].TIC_mscount*0.001+
		(correlator.get_code_NCO_phase(i))/2.046e6 +
		(correlator.get_code_NCO_subPhase(i))/2.046e6L/1024.;
	      multich[i].tr_time = allchtrtime.ch_transtime[ch_cnt].tr_time; 
	      allchtrtime.ch_transtime[ch_cnt].dopp_f =
		( multich[i].carrier_freq/nco_norm_factor-((int)carrier_ref)/nco_norm_factor)
		+200;  /* doppler compensation due to the GP2012 RF */
	    }
	}
      //      cout <<"TIC: " <<(int)allchtrtime.ch_count <<endl;
      
      if(local_t_flag)
	allchtrtime.local_t = local_gps_time;
      else
	allchtrtime.local_t = -1.0;
      
      if( allchtrtime.ch_count >=4 )
	{
	  double accu_clk;
	  int fix_done;
	  fix_done = gps_navfix->pvt_resolve( allchtrtime, &accu_clk );
	  for( i=0; i< num_valid_ch; i++)
            {
	      if( local_t_flag )
                {
		  multich[i].pseudorange = ( local_gps_time - multich[i].tr_time)*SpeedOfLight;
                }
	      gps_navfix->calc_azim_elev(multich[i].prn, &multich[i].elev, &multich[i].azim);
            }
	    
	  if(!local_t_flag && fix_done)
            {
	      set_local_t_flag(accu_clk);
	      process_count = 0;
            }

	}
      
    }// endof  if(tic_done)
  
  
#ifdef GPS_CNTL_DEBUG1
  if(tic_done && allchtrtime.ch_count>=4 )
    {
      gbl_debughdle << allchtrtime.ch_count<< " " ;
      for(  i=0; i< allchtrtime.ch_count; i++)
	gbl_debughdle <<setw(2)
		      <<allchtrtime.ch_transtime[i].prn <<" "
		      <<setw(12)<<setprecision(6)<<setiosflags(ios::fixed|ios::showpoint)
		      <<allchtrtime.ch_transtime[i].tr_time<<" ";
      gbl_debughdle << endl;
    }
#endif
  

}

void GPSController::update_loop_state(int ch_idx)
{

  /* if carrier phase is locked */
  if( multich[ch_idx].process_state & ST_CARRIER_LOCK)
    {
      tracking_carrier_lock(ch_idx);
    }/*else if only code phase is locked */
  else if( multich[ch_idx].process_state & ST_CODE_LOCK)
    {
      pullin_carrier_lock(ch_idx);
    }/*else if only code_lock candidate */
  else if( multich[ch_idx].process_state & ST_CODE_LOCK_CANDIDATE)
    {
      confirm_code_lock(ch_idx);
    }/* if state is beginning of signal acquisition */
  else if( multich[ch_idx].process_state == ST_RESET)
    {
      try_code_lock(ch_idx);
    }
  else
    {
      ; /*default action*/
    }

}

void GPSController::try_code_lock(int ch_idx)
{
  
  double mag_prompt, mag_early, mag_late;
  mag_early = Magnitude( multich[ch_idx].corr_meas.early_I, multich[ch_idx].corr_meas.early_Q);
  mag_prompt = Magnitude( multich[ch_idx].corr_meas.prompt_I, multich[ch_idx].corr_meas.prompt_Q);
  mag_late = Magnitude( multich[ch_idx].corr_meas.late_I, multich[ch_idx].corr_meas.late_Q);
  
  multich[ch_idx].theta = atan2(multich[ch_idx].corr_meas.prompt_Q, multich[ch_idx].corr_meas.prompt_I);
  
  if(  mag_prompt> threshhold )  //threshold reached, jump to confirmation state
    {
      multich[ch_idx].process_state |= ST_CODE_LOCK_CANDIDATE ;
      multich[ch_idx].i_confirmation = 0;
      multich[ch_idx].n_threshold = 0;

#ifdef GPS_LOG_CONSOLE
      cout<<"Channel " << ch_idx<< ":CODE_LOCK_CANDIDATE"<< endl;
#else
      dbg_str<<"MSG CH" << ch_idx << ":CODE_LOCK_CANDIDATE" << endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif

    }
  else
    {
      correlator.slew_half_chip(ch_idx, 2 );
      multich[ch_idx].acq_codes += 2;
      multich[ch_idx].acq_codes = multich[ch_idx].acq_codes % 2046;
    }
      
    
  //setup the output message for this state
    
  double tt = (multich[ch_idx].carrier_freq/nco_norm_factor);
  double ts = (multich[ch_idx].code_freq/nco_norm_factor);
  dbg_str  <<(int)multich[ch_idx].prn <<" "
	   <<multich[ch_idx].process_state <<" "
	   <<multich[ch_idx].ms_count <<" "
	   <<setw(12)<<setprecision(4)<<setiosflags(ios::fixed|ios::showpoint)
	   << multich[ch_idx].corr_meas.prompt_I <<" "
	   << multich[ch_idx].corr_meas.prompt_Q <<" "
   	   << multich[ch_idx].theta <<" " 
	   << 0.0<<" " 
	   << tt <<" "
	   << 0.0 <<" " 
	   << ts<< " "
	   << mag_early<< " " 
	   << mag_prompt<< " " 
	   << mag_late <<" "
	   << multich[ch_idx].azim <<" "
	   << multich[ch_idx].elev<< " " 
	   << 0.0 <<" "
	   <<endl;
    
  ostringstream channelStr(" ");
  channelStr <<"DATA CH" << ch_idx << " "
	     <<multich[ch_idx].subframe_i<<" " <<multich[ch_idx].z_count<<" ";
  log_msg(channelStr.str() + dbg_str.str());
    
  dbg_str.str("");  // clear the string
    
}

void GPSController::confirm_code_lock(int ch_idx)
{
  double mag_prompt, mag_early, mag_late;
  mag_early = Magnitude( multich[ch_idx].corr_meas.early_I, multich[ch_idx].corr_meas.early_Q);
  mag_prompt = Magnitude( multich[ch_idx].corr_meas.prompt_I, multich[ch_idx].corr_meas.prompt_Q);
  mag_late = Magnitude( multich[ch_idx].corr_meas.late_I, multich[ch_idx].corr_meas.late_Q);
  
  multich[ch_idx].theta = atan2(multich[ch_idx].corr_meas.prompt_Q, multich[ch_idx].corr_meas.prompt_I);
  
  if( mag_early > threshhold ||
      mag_prompt> threshhold ||
      mag_late  > threshhold )  //threshold reached, jump to confirmation state
    multich[ch_idx].n_threshold++;
  if( multich[ch_idx].i_confirmation == confirm_m )
    {
      if( multich[ch_idx].n_threshold >= n_of_m_threshold )
	{
	  multich[ch_idx].process_state |= ST_CODE_LOCK;
	  multich[ch_idx].pullin_chms = 0;
	  multich[ch_idx].dfreq = 0.;
	  multich[ch_idx].theta = 0.;
	  multich[ch_idx].corr_sum = 0.;
	  multich[ch_idx].theta_err = 0.;
	  multich[ch_idx].ms_count = 0;
	  multich[ch_idx].ms_sign = 0xaaaaaaaa;
	  
#ifdef GPS_LOG_CONSOLE
	  cout <<"Channel " << ch_idx<< ":CODE_LOCK" << endl;
#else
	  dbg_str<<"MSG CH" << ch_idx << ":CODE_LOCK" << endl;
	  log_msg(dbg_str.str());
	  dbg_str.str("");
#endif
	    
	}
      else
	multich[ch_idx].process_state = ST_RESET;
    }
  multich[ch_idx].i_confirmation++;
  
  //setup the output message for this state
  
  double tt = (multich[ch_idx].carrier_freq/nco_norm_factor);
  double ts = (multich[ch_idx].code_freq/nco_norm_factor);
  dbg_str  <<(int)multich[ch_idx].prn <<" "
	   <<multich[ch_idx].process_state <<" "
	   <<multich[ch_idx].ms_count <<" "
	   <<setw(12)<<setprecision(4)<<setiosflags(ios::fixed|ios::showpoint)
	   << multich[ch_idx].corr_meas.prompt_I <<" "
	   << multich[ch_idx].corr_meas.prompt_Q <<" "
	   << multich[ch_idx].theta <<" " 
	   << 0.0<<" " 
	   << tt <<" "
	   << 0.0 <<" " 
	   << ts<< " "
	   << mag_early<< " " 
	   << mag_prompt<< " " 
	   << mag_late <<" "
	   << multich[ch_idx].azim <<" "
	   << multich[ch_idx].elev<< " " 
	   << 0.0 <<" "
	   <<endl;
  
  ostringstream channelStr(" ");
  channelStr <<"DATA CH" << ch_idx << " " 
	     <<multich[ch_idx].subframe_i<<" " 
	     <<multich[ch_idx].z_count<<" ";
  log_msg(channelStr.str() + dbg_str.str());
  
  dbg_str.str("");  // clear the string	
}

void GPSController::pullin_carrier_lock(int ch_idx)
{
  double mag_prompt, mag_early, mag_late, new_theta, new_dfreq;
  double delta_theta, theta_e, theta_e_avg;

  // new algorithm for pullin stage, save more time and fast and more reliable

  mag_early = Magnitude( multich[ch_idx].corr_meas.early_I, multich[ch_idx].corr_meas.early_Q);
  mag_prompt = Magnitude( multich[ch_idx].corr_meas.prompt_I, multich[ch_idx].corr_meas.prompt_Q);
  mag_late = Magnitude( multich[ch_idx].corr_meas.late_I, multich[ch_idx].corr_meas.late_Q);



  new_dfreq = (mag_late-mag_early)/(mag_late+mag_early);
  if( multich[ch_idx].corr_meas.prompt_Q!=0. || multich[ch_idx].corr_meas.prompt_I!=0. )
    new_theta = atan2(multich[ch_idx].corr_meas.prompt_Q, multich[ch_idx].corr_meas.prompt_I);
  else
    new_theta = multich[ch_idx].theta;
  delta_theta = new_theta - multich[ch_idx].theta;
  
  // code loop setting
  if( multich[ch_idx].pullin_chms > 2)
    {
      multich[ch_idx].code_freq = (int)((-1*new_dfreq+(new_dfreq-multich[ch_idx].dfreq)*(-1))*100) + multich[ch_idx].code_freq;
      correlator.set_code_NCO_step(ch_idx,multich[ch_idx].code_freq); 
    }
  
  multich[ch_idx].dfreq = new_dfreq;

  // carrier loop setting

  if( multich[ch_idx].pullin_chms > 3)
    {
      if( fabs(delta_theta) < PI )
	{
	  multich[ch_idx].carrier_freq =(int)(-(delta_theta*3)*nco_norm_factor) +multich[ch_idx].carrier_freq;
	  correlator.set_carrier_NCO_step(ch_idx,multich[ch_idx].carrier_freq);
	}
    }
  
  multich[ch_idx].theta = new_theta;
  multich[ch_idx].ms_count++;

  theta_e = fabs(multich[ch_idx].theta)>PI/2? fabs(multich[ch_idx].theta)-PI: multich[ch_idx].theta;
  multich[ch_idx].theta_err = 0.978*multich[ch_idx].theta_err + theta_e*theta_e;
  theta_e_avg = multich[ch_idx].theta_err/100.0;
  multich[ch_idx].corr_sum =0.988*multich[ch_idx].corr_sum +  mag_prompt; // accumulate correlation result
  multich[ch_idx].corr_sum_avg = multich[ch_idx].corr_sum /100.0;

  if(multich[ch_idx].pullin_chms >  200 )
    {

      if( fabs(multich[ch_idx].old_prompt_I) > TRACKING_THRD &&
	  fabs(multich[ch_idx].corr_meas.prompt_I) >TRACKING_THRD&&
	  multich[ch_idx].old_prompt_I*multich[ch_idx].corr_meas.prompt_I<0. &&
	  (multich[ch_idx].ms_sign==0xfffff || multich[ch_idx].ms_sign==0x00000) &&
	  !(multich[ch_idx].process_state&ST_BIT_SYNC) &&
	  fabs(theta_e)<PI/4)
	{
	  multich[ch_idx].ms_count =0;
	  multich[ch_idx].process_state |= ST_BIT_SYNC;

#ifdef GPS_LOG_CONSOLE
	  cout <<"Channel " << ch_idx<<" Bit Synchronized!"<<endl;
#else
	  dbg_str<<"MSG CH" <<  ch_idx<<" Bit Synchronized!"<<endl;
	  log_msg(dbg_str.str());
	  dbg_str.str("");
#endif

	}

    }

  multich[ch_idx].ms_sign = (multich[ch_idx].ms_sign<<1) & 0xfffff;
  if(multich[ch_idx].corr_meas.prompt_I>0)
    multich[ch_idx].ms_sign = multich[ch_idx].ms_sign|0x01;

  if( (multich[ch_idx].process_state&ST_BIT_SYNC) && multich[ch_idx].pullin_chms>400  )
    {

      
      if(  multich[ch_idx].ms_count==19 ) // it's time to check tracking conditions
	{
	  if(theta_e_avg< PI/3. &&
	     multich[ch_idx].corr_sum_avg > 500.)
	    {
	      multich[ch_idx].process_state |= ST_CARRIER_LOCK;
	      multich[ch_idx].i_sum_20ms_P = 0.;
	      multich[ch_idx].q_sum_20ms_P = 0.;
	      multich[ch_idx].CdLI  = mag_prompt;
	      multich[ch_idx].phase_change_int = 0.0;

	      multich[ch_idx].dcarr = multich[ch_idx].theta;
	      multich[ch_idx].dfreq = 0;

	      multich[ch_idx].tlm_tst = 0x0;
	      multich[ch_idx].how_tst = 0x0;
	      multich[ch_idx].subframe_i = -1; 
	      multich[ch_idx].msg_count = 0; 
	      multich[ch_idx].TIC_mscount = 0;
	      multich[ch_idx].EML_tau = 0.0;


#ifdef GPS_LOG_CONSOLE
	      cout<<"Channel " << ch_idx<< ":CARRIER_LOCK"<<endl;
#else
	      dbg_str<<"MSG CH" <<  ch_idx<< ":CARRIER_LOCK"<<endl;
	      log_msg(dbg_str.str());
	      dbg_str.str("");
#endif

	    }
	  else
	    {
	      if( theta_e_avg > PI/3 || multich[ch_idx].corr_sum_avg <500)
                {
		  multich[ch_idx].process_state = ST_RESET;

#ifdef GPS_LOG_CONSOLE
		  cout <<"Channel " << ch_idx<<"Back to ACQUISITION..."
		       << multich[ch_idx].corr_sum_avg << " "
		       << theta_e_avg<<endl;
#else
		  dbg_str<<"MSG CH" << ch_idx<<"Back to ACQUISITION..."
                         << multich[ch_idx].corr_sum_avg << " "
                         << theta_e_avg<<endl;
		  log_msg(dbg_str.str());
		  dbg_str.str("");
#endif
                }
	      else  // back to pullin and reset state variables
		{
		  multich[ch_idx].pullin_chms = 0;
		  
		  multich[ch_idx].dfreq = 0.;
		  multich[ch_idx].theta = 0.;
		  multich[ch_idx].corr_sum = 0.;
		  multich[ch_idx].theta_err = 0.;
		  multich[ch_idx].ms_count = 0;
		  multich[ch_idx].process_state &= (~ST_BIT_SYNC);
		  multich[ch_idx].ms_sign = 0xaaaaa;
		}
	       
	    }
	}
    }
  
  multich[ch_idx].old_prompt_I = multich[ch_idx].corr_meas.prompt_I;
  multich[ch_idx].old_prompt_Q = multich[ch_idx].corr_meas.prompt_Q;
  multich[ch_idx].pullin_chms++;
  multich[ch_idx].ms_count = multich[ch_idx].ms_count%20;

  // for debug only

  double tt = (multich[ch_idx].carrier_freq/nco_norm_factor);
  double ts = (multich[ch_idx].code_freq/nco_norm_factor);
  
  dbg_str  <<multich[ch_idx].ms_count <<" "
	   <<setw(12)<<setprecision(4)<<setiosflags(ios::fixed|ios::showpoint)
	   << multich[ch_idx].corr_meas.prompt_I <<" "
	   << multich[ch_idx].corr_meas.prompt_Q <<" "
	   << multich[ch_idx].theta <<" " 
	   << delta_theta<<" " 
	   << tt <<" "
	   << multich[ch_idx].dfreq*nco_norm_factor <<" " 
	   << ts<< " "
	   << mag_early<< " " 
	   << mag_prompt<< " " 
	   << mag_late <<" "
	   << multich[ch_idx].azim <<" "
	   << multich[ch_idx].elev<< " " 
	   << 0.0<<" "
	   <<endl;

#ifdef GPS_CNTL_DEBUG  
  debughdle[ch_idx]<<dbg_str.str();
#endif
  ostringstream channelStr(" ");
  channelStr <<"DATA CH" << ch_idx << " "
	     <<multich[ch_idx].subframe_i<<" " 
	     <<multich[ch_idx].z_count<<" "
	     <<(int)multich[ch_idx].prn <<" "
	     <<multich[ch_idx].process_state<<" " ;
  log_msg(channelStr.str() + dbg_str.str());

  dbg_str.str("");  // clear the string
}

void GPSController::tracking_carrier_lock(int ch_idx)
{
  double  mag_prompt,mag_early, mag_late, new_dfreq, new_dcarr;
  double phase_change, carrier_update; 
  int sign_I;
  new_dfreq=0;
  mag_early = Magnitude( multich[ch_idx].corr_meas.early_I, multich[ch_idx].corr_meas.early_Q);
  mag_prompt = Magnitude( multich[ch_idx].corr_meas.prompt_I, multich[ch_idx].corr_meas.prompt_Q);
  mag_late = Magnitude( multich[ch_idx].corr_meas.late_I, multich[ch_idx].corr_meas.late_Q);

  /* The Lock indicator */
  multich[ch_idx].CdLI += (mag_prompt-multich[ch_idx].CdLI)*0.2;

  /* carrier phase change */
  phase_change = -multich[ch_idx].corr_meas.prompt_Q*multich[ch_idx].old_prompt_I
    + multich[ch_idx].corr_meas.prompt_I*multich[ch_idx].old_prompt_Q;

  carrier_update = multich[ch_idx].phase_change_int*0.001+ phase_change*2;
  multich[ch_idx].phase_change_int += phase_change*4.0;

  if( multich[ch_idx].CdLI <300.)
    {
      multich[ch_idx].process_state = ST_RESET;

#ifdef GPS_LOG_CONSOLE
      cout <<"Channel " << ch_idx<<"Back to ACQUISITION..."
	   << mag_prompt << endl;
#else
      dbg_str<<"MSG CH" << ch_idx<<"Back to ACQUISITION..."
             << mag_prompt << endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif
      return;
    }
     
  // update i_sum_20ms, q_sum_20ms
  multich[ch_idx].i_sum_20ms_P += multich[ch_idx].corr_meas.prompt_I;
  multich[ch_idx].q_sum_20ms_P += multich[ch_idx].corr_meas.prompt_Q;

  multich[ch_idx].EML_tau += ((multich[ch_idx].corr_meas.early_I-multich[ch_idx].corr_meas.late_I)
			      *multich[ch_idx].corr_meas.prompt_I 
			      + (multich[ch_idx].corr_meas.early_Q-multich[ch_idx].corr_meas.late_Q)*multich[ch_idx].corr_meas.prompt_Q);
  
  sign_I = multich[ch_idx].corr_meas.prompt_I>0?1:-1;

  new_dcarr = -multich[ch_idx].corr_meas.prompt_Q*sign_I/mag_prompt;
  multich[ch_idx].carrier_freq += (int)(phase_change*0.0015+ new_dcarr*50.*20.);
  //multich[ch_idx].carrier_freq += (int)(carrier_update*0.02);

  correlator.set_carrier_NCO_step(ch_idx, multich[ch_idx].carrier_freq);  
  multich[ch_idx].dcarr = new_dcarr;


  if( multich[ch_idx].ms_count == 19)
    {
      multich[ch_idx].code_freq = int((multich[ch_idx].EML_tau/12319.0) -
				      2.*(multich[ch_idx].carrier_freq/cc_scale - carrier_ref/cc_scale) + 	
				      code_ref);
      //Here 2.*(multich[ch_idx.carrier_freq  , the coefficient of 2.0 is due to the square_nco set2046M,  not set1023M 
      
      correlator.set_code_NCO_step(ch_idx,multich[ch_idx].code_freq); 
	    
      // set navigation message
      multich[ch_idx].msg_count = (++ multich[ch_idx].msg_count) % 302;
      multich[ch_idx].nav_msg[ multich[ch_idx].msg_count ] = 
	multich[ch_idx].tlm_tst & 0x20000000;
      if( multich[ch_idx].how_tst & 0x20000000 ) // check 30th bit of how_tst
	multich[ch_idx].tlm_tst = multich[ch_idx].tlm_tst*2 + 1;
      else
	multich[ch_idx].tlm_tst = multich[ch_idx].tlm_tst*2;
      // pipe the i_sum_20ms_P bit into HOW
      multich[ch_idx].how_tst = multich[ch_idx].how_tst*2 + 
	bit_sign(multich[ch_idx].i_sum_20ms_P);
#ifdef GPS_CNTL_DEBUG
      
      gbl_debughdle <<(int)multich[ch_idx].prn<<" "
		    <<bit_sign(multich[ch_idx].i_sum_20ms_P)
		    << endl;
#endif
      
      
      search_pream( ch_idx );
      multich[ch_idx].i_sum_20ms_P = 0.;
      multich[ch_idx].q_sum_20ms_P = 0.;
      multich[ch_idx].EML_tau      = 0.;
    }
    

  // update ms_count
  multich[ch_idx].ms_count++;
  multich[ch_idx].ms_count = multich[ch_idx].ms_count%20;
  multich[ch_idx].TIC_mscount++;
  multich[ch_idx].old_prompt_I = multich[ch_idx].corr_meas.prompt_I;
  multich[ch_idx].old_prompt_Q = multich[ch_idx].corr_meas.prompt_Q;  
  
  
  // for debug only

  double tt = (multich[ch_idx].carrier_freq/nco_norm_factor);
  double ts = (multich[ch_idx].code_freq/nco_norm_factor);
  dbg_str << multich[ch_idx].ms_count <<" "
	  <<setw(12)<<setprecision(4)<<setiosflags(ios::fixed|ios::showpoint)
	  << multich[ch_idx].corr_meas.prompt_I <<" "
	  << multich[ch_idx].corr_meas.prompt_Q <<" "
	  << new_dcarr <<" "
	  << (multich[ch_idx].EML_tau/12319.0)<<" "
	  << tt<< " "
	  << multich[ch_idx].dcarr<<" "
	  << ts << " " 
	  << mag_early<< " " 
	  << mag_prompt<< " " 
	  << mag_late <<" "
	  << multich[ch_idx].azim <<" "
	  << multich[ch_idx].elev<< " "
	  << multich[ch_idx].pseudorange<<" "
	  <<endl;
#ifdef GPS_CNTL_DEBUG  
  debughdle[ch_idx] << dbg_str.str();
#endif
  ostringstream channelStr(" ");
  channelStr <<"DATA CH" << ch_idx << " "
	     <<multich[ch_idx].subframe_i<<" " 
	     <<multich[ch_idx].z_count<<" "
	     <<(int)multich[ch_idx].prn <<" "
	     <<multich[ch_idx].process_state<<" " ;
  log_msg(channelStr.str() + dbg_str.str());
  
  dbg_str.str("");
}

int GPSController::check_more_parity(const unsigned int* src, int idx, int word_leng)
{
  int cur_idx, m_parity, parity, flag;
  char prev_bit[2],b_1, b_2, b_3, b_4, b_5, b_6;
  unsigned int sbf_word[10];

  flag = 0;
  cur_idx = (idx + 1)%302;
        
  // pre_bit[0-1] are used to check paritys
  prev_bit[0] = (src[ cur_idx ])>>29;
  prev_bit[1] = (src[ (++cur_idx)%302 ])>>29;
    
  // re-organize the message bits to form message words
  for( int wi = 0; wi < word_leng; wi++ )
    {
      sbf_word[wi] = 0;
      for( int j=0; j<30; j++)
	{
	  if( src[(++cur_idx)%302] & 0x20000000 )
	    sbf_word[wi] = sbf_word[wi]*2 + 0x1;
	  else
	    sbf_word[wi] = sbf_word[wi]*2 ;
	}
    }

  for( int word=0; word<word_leng; word++)
    {

      int tpb1=0x3b1f3480,tpb2=0x1d8f9a40,tpb3=0x2ec7cd00;
      int tpb4=0x1763e680,tpb5=0x2bb1f340,tpb6=0x0b7a89c0;
        
      m_parity = sbf_word[word] & 0x3f;
      b_1 = exor(prev_bit[0],sbf_word[word] & tpb1) *32;
      b_2 = exor(prev_bit[1],sbf_word[word] & tpb2) *16;
      b_3 = exor(prev_bit[0],sbf_word[word] & tpb3) *8;
      b_4 = exor(prev_bit[1],sbf_word[word] & tpb4) *4;
      b_5 = exor(0,sbf_word[word] & tpb5) *2;
      b_6 = exor(prev_bit[0]^prev_bit[1],sbf_word[word] & tpb6);
      parity = b_1+b_2+b_3+b_4+b_5+b_6;
      if( parity != m_parity ) // parity check failed
	flag++;
        
      prev_bit[0]=(m_parity & 0x2) >>1;
      prev_bit[1]= m_parity & 0x1;
    }
    
  //    cout<<" More parity check: " << parity <<" "<< m_parity <<" "<< flag<<endl;
  return (flag>0? 0:1);
}

// To search the correct HOW and TLM
// If correct sync header found, copy the navigation message for the further processing
void GPSController::search_pream( int ch_idx)
{
  unsigned int HOW,TLM,parity0,parity1,frm;
  unsigned int word0, word1;

  word1 = multich[ch_idx].how_tst;
  word0 = multich[ch_idx].tlm_tst;
  //check if the data message is inverted phase
  if(word1&0x01)
    {
      word0 = ~word0;
      word1 = ~word1;
    }

  if (word0 & 0x40000000)
    {                    
      TLM = word0^0x3fffffc0; // then data bit reverse
    }
  else
    {
      TLM = word0;            // Otherwise, data bit unchanged
    }

  if (word1 & 0x40000000)  //
    {                                  //  then data bit reverse
      HOW = word1^0x3fffffc0;
    }
  else
    {
      // Otherwise, data bit unchanged
      HOW = word1;
    }
  // then check parity

  if (((NAG_PREAM^TLM)& 0x3fc00000) == 0x0) // preamble pattern found?
    {

      parity0=(xors(TLM & pb1)*32)+(xors(TLM & pb2)*16)+
	(xors(TLM & pb3)*8)+(xors(TLM & pb4)*4)+
	(xors(TLM & pb5)*2)+(xors(TLM & pb6));

      if (parity0 ==(TLM & 0x3f))  // is parity of TLM ok?
	{
	  parity1=(xors(HOW & pb1)*32)+(xors(HOW & pb2)*16)+
	    (xors(HOW & pb3)*8)+(xors(HOW & pb4)*4)+
	    (xors(HOW & pb5)*2)+(xors(HOW & pb6));
             
	  if (parity1==(HOW & 0x3f))  // is parity of HOW ok?
            {
	      // Get the correct sync , TLM & HOW
	      //if(check_more_parity(&multich[ch_idx].nav_msg[0],multich[ch_idx].msg_count,1))
	      if((HOW&0x03)==0x00)
                {
		  frm = int((HOW & 0x700)>>8);
		  if( frm > 0 && frm < 6)  // valid sub frame #?
                    {
		      
#ifdef GPS_CNTL_DEBUG                        
		      gbl_debughdle <<(int)multich[ch_idx].prn
				    <<" find frame num "
				    <<frm
				    <<" TIC_mscount "
				    << multich[ch_idx].TIC_mscount
				    <<endl;
#endif
                        
		      if( multich[ch_idx].TIC_mscount % 6000 == 0) // this is a correct pream ?
                        {
			  multich[ch_idx].subframe_i = frm;
			  multich[ch_idx].TOW =(HOW & 0x3fffffffL)>>13;
			  multich[ch_idx].TLM =(TLM>>8) & 0x3fff;
			  multich[ch_idx].tr_time_HOW = (multich[ch_idx].TOW-1)*6. + 1.2;
			  multich[ch_idx].z_count = (multich[ch_idx].TOW-1)*6; 
			  multich[ch_idx].process_state |= ST_FRAME_SYNC;
                            
#ifdef GPS_CNTL_DEBUG
                            
			  gbl_debughdle << " CH: " << ch_idx << " "<< frm << " "<< hex
					<< multich[ch_idx].TOW << " "
					<< multich[ch_idx].TLM <<dec << endl;
#endif
                            
			  // then copy msg
			  /* 
			     allch_navmsg[idx].newdata = true;
			     allch_navmsg[idx].prn = multich[ch_idx].prn;
			     allch_navmsg[idx].index =  multich[ch_idx].msg_count;
			     memcpy(&allch_navmsg[idx].msg[0], 
			     &multich[ch_idx].nav_msg[0], 
			     302*sizeof( unsigned int ));
			  */
			  gps_msgnav->process_navmsg(&multich[ch_idx].nav_msg[0],
						     multich[ch_idx].msg_count,
						     ch_idx);
 
                        }
		      else
                        {
#ifdef GPS_CNTL_DEBUG 
			  gbl_debughdle <<(int)multich[ch_idx].prn
					<<" TIC_mscount err: "
					<<multich[ch_idx].TIC_mscount<< endl;
#endif
                        }
		      multich[ch_idx].TIC_mscount = 0;  //                  
                    }
		}
	    }
	}
    }
}
