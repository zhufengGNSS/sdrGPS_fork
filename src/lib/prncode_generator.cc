/*:prncode_generator.cc
********************************************************
 * CA code generator for L1 GPS signal structure
 * 
 * This class is used to generate the CA prn code for certain PRN number
 * For more information, refer to ICD-200C
 * 
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2005
 *******************************************************/

#include <iostream>
#include <fstream>

#include "./includes/prncode_generator.h"

using namespace std;
// From ICD-200C 
const unsigned int PRNcodeGenerator::code_phase_selection[38][2] = {
  {0,0}, {2,6}, {3,7}, {4,8}, {5,9}, {1,9}, {2,10}, {1,8}, {2,9},
  {3,10},{2,3}, {3,4}, {5,6}, {6,7}, {7,8}, {8,9}, {9,10}, {1,4},
  {2,5}, {3,6}, {4,7}, {5,8}, {6,9}, {1,3}, {4,6}, {5,7}, {6,8},
  {7,9}, {8,10}, {1,6},{2,7}, {3,8}, {4,9}, {5,10}, {4,10}, {1,7},
  {2,8}, {4,10} 
};


// constructor of class
PRNcodeGenerator::PRNcodeGenerator()
{
  prn_num = 1; //default prn num is 1

  // then set G2i_sel accrodingly
  G2i_sel[0] = code_phase_selection[ prn_num ][0];
  G2i_sel[1] = code_phase_selection[ prn_num ][1];

  reset_generator();

}
// destructor of class
PRNcodeGenerator::~PRNcodeGenerator()
{
  ;  
}

// reset shift regs and current phase
void PRNcodeGenerator::reset_generator()
{
  for( int i=0; i<10; i++)
    {
      G1_shift_reg[i] = -1;
      G2_shift_reg[i] = -1;
    }
  current_phase = 1023;
  reset_phase_flag = true;
  
  int idx1, idx2;
  idx1 = G2i_sel[0] - 1;
  idx2 = G2i_sel[1] - 1;

  //  cout << "idxs: " << idx1 << " " << idx2;
  current_value = G1_shift_reg[9] * ( G2_shift_reg[idx1] * G2_shift_reg[idx2] );
}

#ifndef __INLINE_FUNC__
void PRNcodeGenerator::set_prnnum( unsigned int pn )
{
  assert( pn >= 1 && pn <= 37);
  prn_num = pn;

  // then set G2i_sel accrodingly
  G2i_sel[0] = code_phase_selection[ prn_num ][0];
  G2i_sel[1] = code_phase_selection[ prn_num ][1];
  
}

unsigned int PRNcodeGenerator::get_prnnum() const
{
  return prn_num;
}

unsigned int PRNcodeGenerator::get_currentphase() const
{
  return current_phase;
}

int PRNcodeGenerator::get_currentvalue() const
{
  return current_value;
}
#endif

// clock update for shift regs for G1 and G2i, also current_phase
void PRNcodeGenerator::OneClockUpdate()
{
  // update current value for current phase
  int idx1, idx2, i;
  idx1 = G2i_sel[0] - 1;
  idx2 = G2i_sel[1] - 1;
  
  current_value = G1_shift_reg[9] * ( G2_shift_reg[idx1] * G2_shift_reg[idx2] );
  int G1_temp, G2_temp;
  G1_temp = G1_shift_reg[2]*G1_shift_reg[9];
  G2_temp = G2_shift_reg[1]*G2_shift_reg[2]*
    G2_shift_reg[5]*G2_shift_reg[7]*
    G2_shift_reg[8]*G2_shift_reg[9];
  
  // update G1 shift registers
  for( i=0; i<9; i++)
    {
      G1_shift_reg[9-i] = G1_shift_reg[8-i]; 
      G2_shift_reg[9-i] = G2_shift_reg[8-i];
    }
  G1_shift_reg[0] = G1_temp;
  G2_shift_reg[0] = G2_temp;
    
  if(reset_phase_flag){
    current_phase = 1;  // this is the beginning of CA code
    reset_phase_flag = false; // just reset once, then disable flag
  }
  else
    current_phase++;
  
  int count=0;
  for(i=0; i<10; i++)
    count += G2_shift_reg[i];
  
  reset_phase_flag = (count==-10);
  

  // check if we need reset current phase
  //  if( Is_final_step() )
  //  current_phase = 0;
 
}

bool PRNcodeGenerator::Is_final_step()
{
  /*
  int highbit_count = 10;
  for( int i=0;i<10; i++)
    if( G2_shift_reg[i] == -1 )
      highbit_count--;
  if( highbit_count )
    return false;  // highbit_count >0, means not final step
  else 
    return true;
  */

  int count=0;
  for( int i=0; i<10; i++)
    count += G2_shift_reg[i];

  return (count==-10)?true:false;
}

// for debug purpose, mode indicate different file format
void PRNcodeGenerator::Save_debuginfo(string filename, int mode)
{
  ;
}
