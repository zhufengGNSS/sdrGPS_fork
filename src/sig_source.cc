/*:sig_source.cc
********************************************************
 * GPS signal source module implementation
 * 
 * This class is used to generate GPS signal data sample
 * for GPS acquisition & tracking loop. 
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2005
 *******************************************************/

#include <iostream>
#include <stdlib.h>
#include "./includes/sig_source.h"

SigSource::SigSource( string fname, double sfreq, unsigned int init_p):sourcefile(fname),samplingfreq(sfreq), init_phase(init_p%2046)
{
  cacodefreq = 1.023e6; // the CA prn code clock, 
                        // even after doppler shift, this value changes little
  sfilehandle.open(sourcefile.c_str(), ios::binary);
  if( !sfilehandle )
    {
      cerr<<"Failed to open GPS source file!\n";
      exit(-1);
    }

  sfilehandle.seekg(0, ios::end);
  sig_length = sfilehandle.tellg();
  sfilehandle.seekg(0, ios::beg);

}

SigSource::SigSource( const Source_config& src_cfg ):sourcefile(src_cfg.source_file),samplingfreq(src_cfg.sampling_freq),init_phase(src_cfg.init_ph)
{
  cacodefreq = 1.023e6;
  sfilehandle.open(sourcefile.c_str(), ios::binary);
  if( !sfilehandle )
    {
      cerr<<"Failed to open GPS source file!\n";
      exit(-1);
    }
  sfilehandle.seekg(0, ios::end);
  sig_length = sfilehandle.tellg();
  sfilehandle.seekg(0, ios::beg);

}

SigSource::~SigSource()
{
  sfilehandle.close();
}

void SigSource::Reset()
{
  int tmp_i;
  tmp_i=(int)(init_phase*0.5*samplingfreq/cacodefreq);
  sfilehandle.seekg(tmp_i); // set file pointer to the beginning of the file
}

#ifndef __INLINE_FUNC__
void SigSource::SlewHalfChip( int p_count )
{
  int relativeshift = (int)(p_count*0.5*samplingfreq/cacodefreq);
  sfilehandle.seekg(relativeshift, ios::cur);
}

int SigSource::GetSigData( void )
{
  char tmpc;
  sfilehandle.read(&tmpc,1);
  sigvalue = tmpc;
  return sigvalue;
}

double SigSource::GetSamplingfreq(void)
{
  return samplingfreq;
}
long SigSource::get_length(void) const
{
  return sig_length;
}

bool SigSource::EndofFile( void )
{
  return sfilehandle.eof();
}
#endif
