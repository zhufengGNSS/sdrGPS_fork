/*:main.cc
*******************************************************
* Main function of the softgps application 
* 
* This file is in chage of opening the IF data file, 
* feeding the data sample to softrcvr class to process, 
* reading the processing result to display module
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Aug, 2008
*******************************************************/

#include "./includes/main.h"

using namespace std;

extern void set_log_buf(log_circular_buf*);
log_circular_buf my_log_buf;
static int process_break; 

/*********************************************************
 * Wrapper function for display run func
 * 
 * This function take the Display_GPS_win pointer
 * then run its run() function to monitor user's input
 *
 ********************************************************/
void* display_run(void *disp_p)
{
  Display_GPS_win *display_point;

  display_point = (Display_GPS_win*)disp_p; 
  display_point->run();
  process_break = 1;

  return NULL;
}

/*********************************************************
 * Main function for the software GPS rcvr application
 * 
 *
 ********************************************************/

int main()
{
  // read config info from "config.ini"
  SOFTGPS_config cfginfo;
  pthread_t thread_id;

  int if_data;
  long samp_count=0;
  ifstream cfgfile("config.ini");
  if(!cfgfile.is_open())
    {
      cout <<"Can't open the config file"<<endl;
      exit(-1);
    }
  CConfigFile CFG( cfgfile );
  CFG.ReadConfigInfo( &cfginfo );
  SigSource gpssigsource( cfginfo.srce_config );
  SoftRcvr sdRcvr(cfginfo);
  Display_GPS_win my_display(cfginfo.srce_config);  

  my_log_buf.wr_idx = 0;
  my_log_buf.rd_idx = 0;
  set_log_buf( &my_log_buf );
  my_display.init(gpssigsource.get_length(), 
		  gpssigsource.GetSamplingfreq());
  process_break = 0;

  /* Create a new thread.  The new thread will get the user input */
  pthread_create (&thread_id, NULL, &display_run, &my_display);  

  while( !gpssigsource.EndofFile() && !process_break)
    {
      if_data = gpssigsource.GetSigData();
      sdRcvr.process_sample(if_data);
      samp_count++;
      if(samp_count %1000 == 0)
	{
	  my_display.process_log_buf( &my_log_buf, samp_count);
	  //read_log_buf( &my_log_buf);
	}
      
    }
  
  my_display.release();
}
