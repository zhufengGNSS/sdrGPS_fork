/*:softrcvr.cc
*******************************************************
* GPS softradio receiver implementation
* 
* This class is used to implement the basic softradio GPS 
* receiver. 
* This module takes the signal sample from the sig_source module and
* finish the basic function that a GPS receiver can provide.
* Including signal acquisition, signal tracking, message decoding,
* measurement foramtion, navigation solution etc.
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Jun, 2007
*******************************************************/

#include "./includes/softrcvr.h"
#include "./includes/gps_controller.h"
#include "./includes/multich_correlator.h"
#include "./includes/acquisition.h"
#include <string.h>
#include <iostream>

// global function to log data

log_circular_buf *log_buf;
void set_log_buf( log_circular_buf *buf_pt)
{
  log_buf = buf_pt;
}
void log_msg(string st)
{

  int len = st.length();
  unsigned char tmpbuf[1024]; // at most store 256 bytes for one call
  unsigned int c_tmp;
  if ( len > 1024-6)
    len = 1024-6; 
    
  int i;
  if( !log_buf )
    return;

  // header is 0xaa55
  tmpbuf[0] = 0xaa;
  tmpbuf[1] = 0x55; 
    
  c_tmp = len + 6;  // overhead: 2 bytes header, 2 bytes length, 2 bytes checksum
  // buf length 
  tmpbuf[2] = (c_tmp>>8)&0xff;
  tmpbuf[3] = c_tmp&0xff;

  memcpy( &tmpbuf[4], st.c_str(), len);

  c_tmp = 0x00;
  for( i=0; i<len+2; i++)
    c_tmp += tmpbuf[2+i];
  //set checksum, total length is (len +4)
  tmpbuf[4+len] = (c_tmp>>8)&0xff;
  tmpbuf[5+len] = c_tmp&0xff;

  // get current index of circular buffer
  i = log_buf->wr_idx;
  if( i+len+6<LOG_BUF_SIZE )
    {
      memcpy((void*)(&log_buf->buf[i]), tmpbuf, len+6);
      log_buf->wr_idx += len+6;
    }
  else
    {
      memcpy((void*)(&log_buf->buf[i]), tmpbuf, LOG_BUF_SIZE-i);
      memcpy((void*)(&log_buf->buf[0]), tmpbuf+LOG_BUF_SIZE-i, i+len+6 - LOG_BUF_SIZE);
      log_buf->wr_idx = i+len+6-LOG_BUF_SIZE;
    }
  log_buf->wr_idx %= LOG_BUF_SIZE;
}


// class constructor
SoftRcvr::SoftRcvr(SOFTGPS_config& sys_config)
{
    correlator = new MultichCorrelator( sys_config.corr_config );
    gps_cntler = new GPSController(*correlator, sys_config.cntl_config);
    gps_acq    = new GPS_Acquisition_mdl(sys_config.cntl_config,
					 sys_config.corr_config);
}

// class destructor, release all resource
SoftRcvr::~SoftRcvr()
{
    delete gps_cntler;
    delete correlator;
    delete gps_acq;
}

void SoftRcvr::process_sample(int sig)
{
    gps_cntler->process_sig( sig );
}

void SoftRcvr::check_exist_warm(int *in_data, double delta_f)
{
  gps_acq->set_incoming_data( in_data);
  gps_acq->check_exist_warm(delta_f);
}
