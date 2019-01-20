/*:cacode_module.cc
 *******************************************************
 * CA code module  for L1 GPS signal structure
 * 
 * This class is used to generate the CA prn code for certain PRN number
 * It can output prn code of three phases: early, prompt, delay, 
 * seperated by 1/2 chip, relatively
 *
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2005
 *******************************************************/
#include "./includes/cacode_module.h"
#include "./includes/prncode_generator.h"
#include "./includes/square_nco.h"
#include <iostream>
#include <fstream>

using namespace std;

const unsigned int CAcodeModule::MAX_PHASE = 0xFFFFFFFF;

CAcodeModule::CAcodeModule()
{
  samplingfreq = 5e6; //default sampling freq is 5MHz
  HalfchipOSC = new SquareNCO;
  FullchipOSC = new SquareNCO;
  prn_cg = new PRNcodeGenerator;
  
  prn_cg->set_prnnum(1); //defalut prn num =1
  reset_module();

  FullchipOSC->set_phasestep( MAX_PHASE/2 );
}

CAcodeModule::~CAcodeModule()
{
  delete HalfchipOSC;
  delete FullchipOSC;
  delete prn_cg;
}

void CAcodeModule::reset_module()
{
  prn_cg->reset_generator();
  ca_e = ca_p = ca_l = 0;
  phase_e = phase_p = phase_l = 0;
  old_ph_e = old_ph_p = old_ph_l = 0;
  
  dumpflag = false;
  slewcount = 0;
}

// set the prn number for CA prn code generator
void CAcodeModule::set_prn_num(unsigned int pn)
{
  prn_cg->set_prnnum( pn );
}
unsigned int CAcodeModule::get_prn_num() const
{
  return prn_cg->get_prnnum();
}


// samplingfrey indicate the sampling frequency of AD after RF front-end
void CAcodeModule::set_sampling_freq(double fs)
{
  samplingfreq = fs;
}

double CAcodeModule::get_sampling_freq() const
{
  return samplingfreq;
}

// set frequency of HalfchipOSC
void CAcodeModule::set2046M(double mfs)
{
  unsigned int phase2046M;
  phase2046M = (unsigned int)( (mfs/samplingfreq)*MAX_PHASE );
  set2046M( phase2046M );
}

void CAcodeModule::set2046M( unsigned int mfs_i)
{
  HalfchipOSC->set_phasestep( mfs_i );
}

// one clock update for ca module
// update NCOs, 
// then update CA code generator accordingly
// Here the clock driven this action is the sampling clock of AD converter!!!

void CAcodeModule::OneClockUpdate()
{
  // first store the phase status at previous samples
  old_ph_e = phase_e;
  old_ph_p = phase_p;
  old_ph_l = phase_l;
  
  
  // check if need to reset phase_p_2046
  if(dumpflag)  
    phase_p_2046=0;
  
  // first update HalfChipOSC  
  HalfchipOSC->OneClockUpdate();
  
  // check if halfchiposc rising jumped
  
  if( HalfchipOSC->edge_happened() == 1) 
    {
      /* without the 2046 module there will generate bad measurement when 
         phase_p_2046 goes from 2045 to 0 */
      phase_p_2046 = (phase_p_2046+1)%2046;

      if( slewcount > 0)
	{
	  slewcount--;     // clock is still running, but not update FullchipOSC
                           // namely, slew one half chip
	  
	}
      else
	{
	  // rising jumped, then need to update FullchipOSC
	  FullchipOSC->OneClockUpdate();
	  // check if it is time to update ca generator
	  if( FullchipOSC->edge_happened() == 1)
	    {
	      prn_cg->OneClockUpdate();
	      ca_e = prn_cg->get_currentvalue();
	      phase_e = prn_cg->get_currentphase();
	      
	      ca_l = ca_p;      //ca_l is half-chip late than ca_p
	      phase_l = phase_p;// update phase in the same mode 
	      
	    }
	  else if( FullchipOSC->edge_happened() == -1)
	    { // ca_p is half-chip late than ca_e
	      ca_p = ca_e;  
	      phase_p = phase_e;
	    }
	} // end of if( slewcount > 0)
    }
  
  // when previous phase is 1023 and current phase is 1, then dump
  dumpflag = ( old_ph_p==1023 && phase_p==1 );
   
}

// check the dump flag
bool CAcodeModule::time_to_dump() const
{
  return dumpflag;
}

void CAcodeModule::slew_half_chip( unsigned int slew_c)
{
  slewcount = slew_c;
}

// output early, prompt, late version of CA code
void CAcodeModule::out_EPL_code(int* codeout) const
{
  if( slewcount > 0) // during slew processing, set the code output to zeros
    {
      codeout[0]=codeout[1]=codeout[2]  = 0;
    }
  else
    {
      codeout[0] = ca_e;
      codeout[1] = ca_p;
      codeout[2] = ca_l;
    }
}
// output early, prompt, late version of code phase
void CAcodeModule::out_EPL_phase( unsigned int *pha) const
{
  pha[0] = phase_e;
  pha[1] = phase_p;
  pha[2] = phase_l;
  
  // also output their previous phase ( maybe for debug use only)
  pha[3] = old_ph_e;
  pha[4] = old_ph_p;
  pha[5] = old_ph_l;
}

int CAcodeModule::out_prompt_phase(void) const
{
  return  phase_p_2046;
}
int CAcodeModule::out_subphase(void) const
{
  unsigned int iph = HalfchipOSC->get_currentphase();
  iph = iph >> 22;
  return iph; 
}

// for debug use only
void CAcodeModule::out_clock( int* clockout) const 
{
  clockout[0] = (int)HalfchipOSC->Output();
  clockout[1] = (int)FullchipOSC->Output();
}
