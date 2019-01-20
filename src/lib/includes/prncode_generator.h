#ifndef  PRNCODE_GENERATOR_H
#define  PRNCODE_GENERATOR_H
#include <assert.h>
#include <string>
#include <iostream>
using namespace std;

class  PRNcodeGenerator{
 public:
  
  PRNcodeGenerator();
  ~PRNcodeGenerator();

  void reset_generator();          // reset the shift registers
  void OneClockUpdate(); // one clock comes, update everything
  void set_prnnum( unsigned int ); // set prn num, 
  int get_currentvalue() const;
  void Save_debuginfo( string, int); // for debug use only
  
  unsigned int get_currentphase() const;
  unsigned int get_prnnum() const;

 protected:

 private:
  unsigned int prn_num;  // which prn code I want to generate
  int G1_shift_reg[10], G2_shift_reg[10];
  int current_value;
  unsigned int current_phase;
  unsigned int G2i_sel[2]; 
  static const unsigned int code_phase_selection[38][2]; // G2i phase selection

  bool reset_phase_flag;
  bool Is_final_step(); // check if current value is the last digit of all 1023 bits 
};


#ifdef __INLINE_FUNC__
inline void PRNcodeGenerator::set_prnnum( unsigned int pn )
{
  assert( pn >= 1 && pn <= 37);
  prn_num = pn;

  // then set G2i_sel accrodingly
  G2i_sel[0] = code_phase_selection[ prn_num ][0];
  G2i_sel[1] = code_phase_selection[ prn_num ][1];
  
}

inline unsigned int PRNcodeGenerator::get_prnnum() const
{
  return prn_num;
}

inline unsigned int PRNcodeGenerator::get_currentphase() const
{
  return current_phase;
}

inline int PRNcodeGenerator::get_currentvalue() const
{
  return current_value;
}
#endif

#endif
