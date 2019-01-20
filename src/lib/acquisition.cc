/*:acquisition.cc
*******************************************************
* GPS acquisition module implementation
* 
* This class is used to implement the signal acquisition logic 
* for GPS IF signals. 
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Jan, 2005
*******************************************************/

#include "./includes/acquisition.h"
#include "./includes/onech_correlator.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <string.h>
using namespace std;

static bool Greatthan( const corrresult_phidx& t1, const corrresult_phidx& t2)
{
  return t1.correlationresult > t2.correlationresult;
}
static bool Greatthanfreq( const corrresult_freqphidx& t1, const corrresult_freqphidx& t2)
{
  return t1.correlationresult > t2.correlationresult;
}

GPS_Acquisition_mdl::GPS_Acquisition_mdl(const Controller_config &cntl_cfg,
					 const Correlator_config& corr_cfg)
{
  string tmpstr, tmpsubstr;
  unsigned int i=0, j=0, ti;
  string::size_type k=0;
  correlator = new OnechCorrelator;
  correlator->init_correlator( corr_cfg );

  // calculate the length of 20ms IF data
  data_count_20ms = int(corr_cfg.sampling_freq * 0.02);

  threshhold = cntl_cfg.threshold;  // should set through config.ini
  memset( (void*)possible_exist_satnum, 0, sizeof(int)*32);

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
	      possible_exist_satnum[j+1] = ti;
	      j++;
	      possible_exist_satnum[0] = j;
	    }
	}
    } // end of while(k!
  
  nco_norm_factor=(0xFFFFFFFF)/correlator->get_sampling_freq();
  point_per_chip = correlator->get_sampling_freq()/1.023E6;
  carrier_ref = (unsigned int)(correlator->get_carrier_freq()*nco_norm_factor);
  code_ref = (unsigned int)(2.046e6*nco_norm_factor);

  exist_satnum =0;
  possible_corr_ph.clear();
  possible_corr_freqph.clear();
}

GPS_Acquisition_mdl::~GPS_Acquisition_mdl()
{
  delete correlator;
}

// Acquisition and confirmation for input signal
// Return true if the signal really exist , and set the phase 
// Return falst if no signal exists, namely, no correlation peak greater than 
// threshhold found.
bool GPS_Acquisition_mdl::check_acquisition(double *corr_result )
{
  int slew_ph_counter =0, count;
  bool done_flag;
  double IQ[6], result;
  corrresult_phidx tmp_cp;
  list <corrresult_phidx>::const_iterator p;
  
#ifdef GPS_ACQMDL_VERBOSE  
  cout << "Begin acquisition for prn#" << correlator->get_prn_num()<<" ..."<< endl;
#endif

  for( slew_ph_counter =0; slew_ph_counter<2046; slew_ph_counter++)
    {
      done_flag = false;
      correlator->reset_ch();

      int relativeshift = (int)(slew_ph_counter*0.5*point_per_chip);

      count=0;
      while( !done_flag )
	{
	  correlator->OneClockUpdate( incoming_data[relativeshift+count] );
	  count++;
	  done_flag = correlator->is_time_to_output();
	  if( done_flag )
	    correlator->getIQ( IQ );
	}

      result= sqrt(IQ[2]*IQ[2] + IQ[3]*IQ[3]);

      if( corr_result != NULL)   // if the pointer of corr_result is not empty, store the result to it
	corr_result[slew_ph_counter] = result;

      if( result > threshhold ) // save this result and its phase
	{
	  tmp_cp.correlationresult= result;
	  tmp_cp.phase_idx = slew_ph_counter;
	  possible_corr_ph.push_back( tmp_cp );
	}
      // set flag for next phase
      done_flag = false;
#ifdef GPS_ACQMDL_VERBOSE      
      if( slew_ph_counter%100 == 0)
	cout<<"."<<flush;
#endif      
    }
#ifdef GPS_ACQMDL_VERBOSE
  cout << endl;
#endif

  if( possible_corr_ph.empty())
    {
      // cout<<" No peak over threshold found!" <<endl;
      return false;
    }
  
  // sort the correlation result
  possible_corr_ph.sort(Greatthan);
  
  tmp_cp = *possible_corr_ph.begin();

  // for debug only  
  /*
    for( p=possible_corr_ph.begin(); p!=possible_corr_ph.end(); p++)
    cout << "result: " << p->correlationresult
    <<", phase: " << p->phase_idx<< endl;
  */  
  // reset the source 
   
  //  cout << "Correlation peak: " << tmp_cp.correlationresult
  //     <<", phase : " << tmp_cp.phase_idx<< endl;
  if( tmp_cp.correlationresult > threshhold )
    {
#ifdef GPS_ACQMDL_VERBOSE      
      cout << "Acquisition succeeded, ";
      cout << "with result "<< tmp_cp.correlationresult;
      cout << ", at phase "<< tmp_cp.phase_idx<< endl;
#endif
      return true;
    }
  else
    return false;
}

// After acquisition, phase estimation is known,
// but need confirmation for later pullin and tracking processing

bool GPS_Acquisition_mdl::check_confirmation(corrresult_freqphidx *p_freq_ph)
{
  int try_count, success_count;
  try_count = 10;       // 10 tries and 7 wins can tell .
  success_count = 7;
  list <corrresult_freqphidx>::const_iterator p;
  bool success_flag=false;
  for( p=possible_corr_freqph.begin(); p!=possible_corr_freqph.end(); p++)
    {
      if( Tryconfirmation(p->carrier_freq,p->phase_idx,try_count,success_count))
	{
	  success_flag = true;
	  *p_freq_ph = *p;
#ifdef GPS_ACQMDL_VERBOSE
	  cout <<"Confirmation succeeded,";
	  cout <<" with phase: " <<p->phase_idx<<endl;
#endif
	  break;
	}
    }
  
  return success_flag;
  
}

bool GPS_Acquisition_mdl::Tryconfirmation(double freq, int pha, int t_c,int s_c)
{
  int trynum=0, succnum=0, count;
  bool ms_done_flag;
  double IQ[6],corresult=0.;

  correlator->reset_ch();

  correlator->set_carrier_NCO_step((unsigned int)(freq*nco_norm_factor));  // set carrier frequency for the correlator
  
  correlator->slew_half_chip( pha ); // phase slew for pha half chips
  
  count=0;
  while( trynum<t_c )
    {
      ms_done_flag = false;
      corresult = 0.;

      while (!ms_done_flag)
	{
	  correlator->OneClockUpdate( incoming_data[count]);
	  count++;
	  ms_done_flag = correlator->is_time_to_output();
	}
      
      if( ms_done_flag )
	{
	  correlator->getIQ( IQ );
	  trynum++;
	  corresult= sqrt(IQ[2]*IQ[2] + IQ[3]*IQ[3]);
	  //	  cout <<"cor result: " <<corresult<< endl;
	  if(corresult > threshhold)  // when threshold reached, ++ success count
	    succnum++;
	}
    }
  
  return succnum >= s_c;
}

// Check the existance of GPS signal for all 32 prn number
// The existing sat number and doppler shift,and code phase
// are stored in the list possible_corr_freqph

void GPS_Acquisition_mdl::check_exist_cold(double stepfreq)
{

  int searchbin_num;
  int carrier_frq;
  double *result;
  corrresult_freqphidx tmp_freqphidx;

  searchbin_num = (int)ceil(20000.0/stepfreq);
  result = new double [searchbin_num*2046 +2046];

  exist_satnum =0;

  for( int i=1; i<33; i++)  // totally 32 prn numbers
    {
      correlator->set_prn_num(i);  // set the prn number
      possible_corr_freqph.clear();
      memset((void*)&tmp_freqphidx, 0, sizeof(corrresult_freqphidx));
      cout << "Begin acquisition for prn#" << correlator->get_prn_num()<<flush;      

      for ( int j=0; j<searchbin_num; j++)
	{
	  carrier_frq = carrier_ref+(int)((-10000.0 + j*stepfreq)*nco_norm_factor); 
	  correlator->set_carrier_NCO_step(carrier_frq);
	  possible_corr_ph.clear();
	  
	  if( check_acquisition(result+j*2046) )
	    { // save all the correlation result for this carrier freq 
	      corrresult_freqphidx tmp_corr;

	      list <corrresult_phidx>::const_iterator p;
	      for( p=possible_corr_ph.begin(); p!=possible_corr_ph.end(); p++)
		{
		  tmp_corr.carrier_freq = carrier_frq /nco_norm_factor;
		  tmp_corr.correlationresult = p->correlationresult;
		  tmp_corr.phase_idx = p->phase_idx;
		  possible_corr_freqph.push_back( tmp_corr );
		}
	    }
	  cout <<"#"<<flush;
	}
      cout <<endl;
      // sort all the correlation results for this prn number
      possible_corr_freqph.sort(Greatthanfreq);

      if( possible_corr_freqph.empty() )  // if no result exceed threshold, jump to next prn number
	break;                             

      if( check_confirmation( &tmp_freqphidx))
	{
	  cout << "Add sat num "<< (int) i<<endl;
	  exist_satnum ++;
	  ph_freq_info[exist_satnum-1].prnnum = i;
	  ph_freq_info[exist_satnum-1].freq_corrres_phidx = tmp_freqphidx;
	}
      
    } // end of for( i 
  delete[] result;

}

// Check the existance of GPS signal for certain prn number
// true: acquisition and confirmation passed, allinfo are stored 
// false: either acquisition or confirmation not passed

bool GPS_Acquisition_mdl::check_exist(int prn, double stepfreq)
{

  int searchbin_num;
  int carrier_frq;
  double *result;

#ifdef GPS_ACQMDL_RESULT
  ofstream result_file;
  ostringstream filenm;
  filenm <<"./data/acq_"<< prn <<".m";
  cout<<filenm.str()<<endl;
  result_file.open(filenm.str().c_str());
  if( !result_file.is_open())
    cerr<<"Couldn't open the acqusition result file for prn " <<prn<<endl;
#endif

  corrresult_freqphidx tmp_freqphidx;

  searchbin_num = (int)ceil(20000.0/stepfreq);// -10k---10k search range
  result = new double [searchbin_num*2046 +2046];
 
  correlator->set_prn_num(prn);  // set the prn number
  memset((void*)&tmp_freqphidx, 0, sizeof(corrresult_freqphidx));
  cout << "Begin acquisition for prn#" << correlator->get_prn_num()<<flush;      

  for ( int j=0; j<searchbin_num; j++)
    {
      carrier_frq = carrier_ref+(int)((-10000.0 + j*stepfreq)*nco_norm_factor); 
      correlator->set_carrier_NCO_step(carrier_frq);
      possible_corr_ph.clear();
	  
      if( check_acquisition(result+j*2046) )
	{ // save all the correlation result for this carrier freq 
	  corrresult_freqphidx tmp_corr;

	  list <corrresult_phidx>::const_iterator p;
	  for( p=possible_corr_ph.begin(); p!=possible_corr_ph.end(); p++)
	    {
	      tmp_corr.carrier_freq = carrier_frq /nco_norm_factor;
	      tmp_corr.correlationresult = p->correlationresult;
	      tmp_corr.phase_idx = p->phase_idx;
	      possible_corr_freqph.push_back( tmp_corr );
	    }
	}
      cout <<"#"<<flush;
    }
  cout <<endl;
  // sort all the correlation results for this prn number
  possible_corr_freqph.sort(Greatthanfreq);

#ifdef GPS_ACQMDL_RESULT
  for(int i=0; i<2046; i++)
    {
      for(int j=0; j<searchbin_num; j++)
	result_file<<result[j*2046 +i] <<" ";
      result_file<<endl;
    }
  
  if( result_file.is_open())
    result_file.close();
#endif

  delete[] result;


  if( possible_corr_freqph.empty() )  // if no result exceed threshold, jump to next prn number
    {
      return false;
    }
  //  else
  //      return true;

  if( check_confirmation( &tmp_freqphidx))
    {
      cout << "Add sat num "<< (int) prn<<endl;
      exist_satnum ++;
      ph_freq_info[exist_satnum-1].prnnum = prn;
      ph_freq_info[exist_satnum-1].freq_corrres_phidx = tmp_freqphidx;

      return true;
    }
  else
    return false;
}

void GPS_Acquisition_mdl::check_exist_warm(double stepfreq)
{

  int i=0; 
  if( possible_exist_satnum[0] >0)
    {
      for( i=0; i<possible_exist_satnum[0]; i++)
	{
	  check_exist(possible_exist_satnum[i+1], stepfreq);
	}
    }
}

void GPS_Acquisition_mdl::set_incoming_data(int* in_data)
{
  incoming_data = in_data;

}


void GPS_Acquisition_mdl::print_checkexist()
{
  if( exist_satnum >0)
    {
      cout <<"Existing satellite number are: "<<endl;
      for( int i=0; i<exist_satnum; i++)
	{
	  cout<<setw(12)<<setprecision(2)<<setiosflags(ios::fixed|ios::showpoint) <<ph_freq_info[i].prnnum<<"  @" <<ph_freq_info[i].freq_corrres_phidx.carrier_freq << ": "<<ph_freq_info[i].freq_corrres_phidx.correlationresult << ": " << ph_freq_info[i].freq_corrres_phidx.phase_idx<<endl ;
	}
      cout <<endl;
    }

  // store this info into file for further analysis
#ifdef GPS_ACQMDL_RESULT
  ofstream acqres_file("./data/acq_res.m");
  if( exist_satnum >0)
    {
      acqres_file <<"Existing satellite number are: "<<endl;
      for( int i=0; i<exist_satnum; i++)
	{
	  acqres_file<<setw(12)<<setprecision(2)<<setiosflags(ios::fixed|ios::showpoint) <<ph_freq_info[i].prnnum<<"  @" <<ph_freq_info[i].freq_corrres_phidx.carrier_freq << ": "<<ph_freq_info[i].freq_corrres_phidx.correlationresult << ": " << ph_freq_info[i].freq_corrres_phidx.phase_idx <<endl;
	}
      acqres_file <<endl;
    }

  
  if( acqres_file.is_open())
    acqres_file.close();
#endif


}

