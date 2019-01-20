#ifndef CACODE_MODULE_H
#define CACODE_MODULE_H

//forward declaration
class SquareNCO;
class PRNcodeGenerator;

class CAcodeModule{
 public:
  static const unsigned int MAX_PHASE; // = 0xFFFFFFFF;
  
  CAcodeModule();
  ~CAcodeModule();

  void reset_module();
  
  void out_clock(int *) const;  // output status of OSCs, for debug
  
  void out_EPL_code( int *) const;// output e,p,l version of CA code
  void out_EPL_phase( unsigned int *) const; // output pahse of e,p,l
  int out_prompt_phase()const; // output the prompt phase
  int out_subphase() const; //output subphase in 1 chip( 1/1023 CAperiod)
  void OneClockUpdate();       // one clock update 
  
  bool time_to_dump() const;    // time to dump the integrations
  void slew_half_chip( unsigned int );

  void set_prn_num( unsigned int );
  unsigned int get_prn_num() const;
  void set2046M(double);       // set frequency of HalfchipOSC
  void set2046M(unsigned int);
  void set_sampling_freq(double);// set sampling frequency
  double get_sampling_freq() const;


 protected:
 private:
  double samplingfreq;
  int ca_p, ca_e, ca_l;    //early, prompt, late value of CA code
  unsigned int phase_e, phase_p, phase_l;// early,prompt, late value of PRN phase
  unsigned int phase_p_2046; 
  unsigned int old_ph_e, old_ph_p, old_ph_l;
  unsigned int slewcount;

  bool dumpflag;   // flag of time to dump
  SquareNCO *HalfchipOSC, *FullchipOSC; // HalfchipOSC is working on 2.046MHz
                                        // FullchipOSC is working on 1.023MHz
  PRNcodeGenerator *prn_cg;             // PRNcodeGenerator pointer
};
#endif
