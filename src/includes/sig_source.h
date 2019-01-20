#ifndef SIG_SOURCE_H
#define SIG_SOURCE_H

#include <fstream>
#include "softgps.h"

using namespace std;

class SigSource{
 public:
  SigSource(string,double, unsigned int );
  SigSource(const Source_config&);
  ~SigSource();
  
  void SlewHalfChip( int p_count );
  void Reset(void);
  int GetSigData(void);
  bool EndofFile(void);
  long get_length(void) const;
  double GetSamplingfreq(void);
 protected:
 private:
  string sourcefile;
  ifstream sfilehandle;

  int sigvalue;
  double samplingfreq, cacodefreq;
  unsigned int init_phase;
  long  sig_length; // signal length in unit of sample

};

#ifdef __INLINE_FUNC__
inline void SigSource::SlewHalfChip( int p_count )
{
  int relativeshift = (int)(p_count*0.5*samplingfreq/cacodefreq);
  sfilehandle.seekg(relativeshift, ios::cur);
}

inline int SigSource::GetSigData( void )
{
  char tmpc;
  sfilehandle.read(&tmpc,1);
  sigvalue = tmpc;
  return sigvalue;
}

inline double SigSource::GetSamplingfreq(void)
{
  return samplingfreq;
}

inline long SigSource::get_length(void) const
{
  return sig_length;
}

inline bool SigSource::EndofFile( void )
{
  return sfilehandle.eof();
}
#endif

#endif
