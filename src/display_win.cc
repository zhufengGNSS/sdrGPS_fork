/*:display_win.cc
*******************************************************
* Display function 
* 
* This file display the processing result of 
* software GPS receiver.
*
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Aug, 2008
*******************************************************/
#include "./includes/display_win.h"
#include "./includes/softgps.h"
#include <string.h>
#include <stdlib.h>
using namespace std;

#define WRAPBUFLEN(x)   ((x)%LOG_BUF_SIZE) 
const double Display_GPS_win::R2D= (180.0/3.1415926);
const int    Display_GPS_win::ROW_THRD = 25;
const int    Display_GPS_win::COL_THRD = 100;
const int    Display_GPS_win::info_width = 50;
const int    Display_GPS_win::info_height = 13;
const char   Display_GPS_win::jump_char[4]={'-','\\','|','/'};

Display_GPS_win::Display_GPS_win(const Source_config& src_cfg):
  sourcefile(src_cfg.source_file)
{
  max_row = 0; 
  max_col = 0;
  jump_ch_idx = 0;
}
Display_GPS_win::~Display_GPS_win()
{
  ;
}
    
void Display_GPS_win::init(int sample_n, double fs)
{
  char ch;
  sampling_freq = fs; 
  data_duration = sample_n;
  /* Initialize curses */
  initscr();
  getmaxyx(stdscr, max_row, max_col);
  endwin();
  /*check if the col and row meet the threshold */
  if( max_row<ROW_THRD || max_col<COL_THRD )
    {
      cout <<"The display windows requires at least [25,100] to show all the information."<<endl<<
	"Your console window size is [" << 
	max_row<< ","<< max_col<< 
	"],  which could cause some information not able to be displayed."<<endl <<
	"Do you still want to continue? [Y/N]"<<flush;
      //cin >>ch;
      ch = getchar();
      if( (ch!='y')&&(ch!='Y'))
	{
	  exit(0);
	}
    }
  /* redo the initialization */
  initscr();  
  start_color();
  cbreak();
  //timeout(100);
  noecho();
  curs_set(0);//no cursor
  keypad(stdscr, TRUE);
  /* Initialize all the colors */
  init_pair(1, COLOR_RED, COLOR_BLACK);
  init_pair(2, COLOR_GREEN, COLOR_BLACK);
  init_pair(3, COLOR_BLUE, COLOR_BLACK);
  init_pair(4, COLOR_CYAN, COLOR_BLACK);
  
  init_wins();
}

void Display_GPS_win::release(void)
{
  endwin();
}

//decode message output from softgps library
//and display the processing result
int Display_GPS_win::process_log_buf( log_circular_buf *buf, int samp_c)
{
  int i, j, head_idx;
  char flag, tmpbuf[LOG_BUF_SIZE]; 
  int  len, buflen, chksum, c_chksum, leng_read;
  int ret_val = 0;

  current_data_cnt = samp_c;
  flag = 0;

  leng_read = 0;
  //check if buf is empty
  if(buf->wr_idx == buf->rd_idx)
    return 0;

  // if not empty, get the length of the buf
  if( buf->wr_idx>buf->rd_idx )
    buflen = buf->wr_idx - buf->rd_idx ;
  else
    buflen = buf->wr_idx + LOG_BUF_SIZE - buf->rd_idx; 

  flag = 0;
  i= buf->rd_idx;
  while(buf->rd_idx!=buf->wr_idx)
    {
      while( !flag )
	{
	  i=buf->rd_idx;
	  j=WRAPBUFLEN(i+1);
	  if( (buf->buf[i] == (char)0xaa) && (buf->buf[j] == (char)0x55) )
	    {
	      flag = 1;
	      head_idx = i;
	    }
	  else
	    {
	      buf->rd_idx = WRAPBUFLEN(buf->rd_idx+1);
	    }
	}

      if( flag )
	{
	  i=  WRAPBUFLEN(buf->rd_idx+2);
	  j=  WRAPBUFLEN(buf->rd_idx+3);

	  len = buf->buf[i]&0xff;
	  chksum = len;
	  len = len<<8;
	  len += (buf->buf[j]&0xff);
	  chksum += (buf->buf[j]&0xff);

	  buf->rd_idx = WRAPBUFLEN(buf->rd_idx+4);
	  for( i=0; i<len-6; i++)
	    {
	      chksum += buf->buf[ buf->rd_idx ];
	      tmpbuf[i] = buf->buf[ buf->rd_idx ];
	      buf->rd_idx = WRAPBUFLEN(buf->rd_idx+1);
	    }

	  tmpbuf[len-6] = 0x00;

	  i = buf->rd_idx;
	  j = WRAPBUFLEN(buf->rd_idx+1);

	  buf->rd_idx = WRAPBUFLEN(buf->rd_idx+2);

	  c_chksum = buf->buf[i]&0xff;
	  c_chksum = c_chksum <<8;
	  c_chksum += buf->buf[j]&0xff;

	  if( chksum == c_chksum )
	    {
	      buf->buf[head_idx] = 0x00; // invalidate this packet
	      ret_val = display_data( &tmpbuf[0] );
	      memset(tmpbuf, 0, LOG_BUF_SIZE*sizeof(char));
	    }
	  leng_read += len;
	}
      flag = 0;
    }
  return ret_val; 
  
}

int Display_GPS_win::display_data(const char *buf)
{
  int ret_val = 0; 
  /* MSG packet, skip it
     if( buf[0]=='M'&& buf[1]=='S'&& buf[2]=='G')
     {
	
     mvwprintw(track_win,3,1, "%s", &buf[3]);
     refresh();
     update_panels();
     doupdate();
      
     }*/
  if( buf[0]=='D' &&
      buf[1]=='A' &&
      buf[2]=='T' &&
      buf[3]=='A')
    {
      /* this is a data packet */
      if( buf[5]=='C' && buf[6]=='H')
	{
	  show_tracking( &buf[5] );
	}
      if( buf[5]=='L' && buf[6]=='S')
	{
	  show_ls_fix(&buf[5]);
	}
      if( buf[5]=='K' && buf[6]=='F')
	{
	  show_kf_fix(&buf[5]);
	}
    }
  return ret_val;
}

void Display_GPS_win::run(void)
{
  int ch; 

  /*Check user's input */
  while( (ch = getch()) != 'x' && ch!='X'  ) /* Waits for TIME_OUT milliseconds */
    {
      switch(ch)
	{	
	case 9:
	  current_top = (PANEL *)panel_userptr(current_top);
	  top_panel(current_top);
	  break;
	case KEY_F(1):
	  top_panel(track_panel);
	break;
	case KEY_F(2):
	  top_panel(ls_panel);
	break;
	case KEY_F(3):
	  top_panel(kf_panel);
	break;
	case 'm':
	case 'M':
	  show_panel(info_panel);
	  getch();
	  hide_panel(info_panel);
	  break;
	default: 

	  break;
	}

      update_panels();
      doupdate();
    }
  
  
}
void Display_GPS_win::show_tracking(const char *buf)
{
  double prompt_I, prompt_Q, theta, theta_delta, code_freq, carr_freq;
  double mag_e, mag_p, mag_l, dummy1, azim, elev, pRange;
  char chidx[6];
  int process_state, ms_count, prn, frame, z_count; 
  int ch_idx;
  
  sscanf(buf, "%s %d %d %d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	 chidx, 
	 &frame, 
	 &z_count, 
	 &prn, 
	 &process_state, 
	 &ms_count,
	 &prompt_I, 
	 &prompt_Q, 
	 &theta, 
	 &theta_delta,
	 &carr_freq, 
	 &dummy1,
	 &code_freq,
	 &mag_e,
	 &mag_p,
	 &mag_l,
	 &azim,
	 &elev,
	 &pRange);

  ch_idx = chidx[2] - '0';
  
  mvwprintw(track_win, 4+ch_idx,1,"%d", ch_idx);
  mvwprintw(track_win, 4+ch_idx,5,"%d", prn );
  mvwprintw(track_win, 4+ch_idx,9,"%d", process_state);

  mvwprintw(track_win, 4+ch_idx,13, "     ");
  mvwprintw(track_win, 4+ch_idx,13, "%d", frame);

  mvwprintw(track_win, 4+ch_idx,19, "        ");
  mvwprintw(track_win, 4+ch_idx,19, "%d", z_count);

  mvwprintw(track_win, 4+ch_idx,26, "%.2f", code_freq/2.0);
  mvwprintw(track_win, 4+ch_idx,38, "%.2f", carr_freq);

  mvwprintw(track_win, 4+ch_idx,50, "        ");
  mvwprintw(track_win, 4+ch_idx,50, "%.2f", pRange);

  mvwprintw(track_win, 4+ch_idx,62, "        ");
  mvwprintw(track_win, 4+ch_idx,62, "%.1f", prompt_I);

  mvwprintw(track_win, 4+ch_idx,70, "        ");
  mvwprintw(track_win, 4+ch_idx,70, "%.1f", prompt_Q);

  mvwprintw(track_win, 4+ch_idx,78, "%.2f", elev*R2D);
  mvwprintw(track_win, 4+ch_idx,86, "%.2f", azim*R2D);

  mvwprintw(track_win, 16,22, "%.6f",current_data_cnt/sampling_freq);
  mvwprintw(ls_fix_win, 16,22, "%.6f",current_data_cnt/sampling_freq);
  mvwprintw(kf_fix_win, 16,22, "%.6f",current_data_cnt/sampling_freq);

  if(jump_ch_idx==0)
    {
      mvwprintw(track_win,17,20,"waiting...");
    }
  else
    {
      mvwprintw(track_win,17,20,"          ");
      mvwprintw(track_win,17,20,"%c",jump_char[jump_ch_idx-1]);
    }
  refresh();

  update_panels();
  doupdate();

  
}
void Display_GPS_win::show_ls_fix(const char *buf)
{
  double x[3], vx[3], lla[3], vdop, gdop,hdop,pdop;
  double localT,bias,drift;
  int    sv_cnt;
  char   lsidx[8];

  if( ++jump_ch_idx>4)
    jump_ch_idx=1;

  sscanf(buf,"%s %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
	 lsidx,
	 &x[0],&x[1],&x[2],&lla[0],&lla[1],&lla[2],
	 &sv_cnt,&vx[0], &vx[1], &vx[2],&drift, &bias,
	 &hdop, &vdop, &pdop, &gdop, &localT);
                  
  mvwprintw(ls_fix_win, 3, 10,"%.2f", x[0]);
  mvwprintw(ls_fix_win, 3, 40,"%.2f", x[1]);
  mvwprintw(ls_fix_win, 3, 70,"%.2f", x[2]);
  mvwprintw(ls_fix_win, 4, 10, "%.3f ", vx[0]);
  mvwprintw(ls_fix_win, 4, 40,"%.3f ",vx[1]);
  mvwprintw(ls_fix_win, 4, 70,"%.3f ", vx[2]);
  mvwprintw(ls_fix_win, 5, 8, "%.6f", bias);
  mvwprintw(ls_fix_win, 5, 38,"%.6f", drift);
  mvwprintw(ls_fix_win, 5, 70,"%.6f", localT);
  mvwprintw(ls_fix_win, 6, 14, "%.6f",lla[0]);
  mvwprintw(ls_fix_win, 6, 41,"%.6f",lla[1]);
  mvwprintw(ls_fix_win, 6, 71,"%.6f",lla[2]);

  mvwprintw(ls_fix_win, 7, 8, "%.2f",gdop);
  mvwprintw(ls_fix_win, 7, 28,"%.2f",pdop);
  mvwprintw(ls_fix_win, 7, 48,"%.2f",vdop);
  mvwprintw(ls_fix_win, 7, 68,"%.2f",hdop);


  refresh();
  update_panels();
  doupdate();
 
}
void Display_GPS_win::show_kf_fix(const char *buf)
{
  double x[8],dx[8],px[8];
  
  if(buf[3]=='1')
    {
      sscanf(&buf[4],"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
	     &x[0],&x[1],&x[2],&x[3],&x[4],&x[5],&x[6],&x[7],
	     &dx[0],&dx[1],&dx[2],&dx[3],&dx[4],&dx[5],&dx[6],&dx[7],
	     &px[0],&px[1],&px[2],&px[3],&px[4],&px[5],&px[6],&px[7]);
      
      mvwprintw(kf_fix_win, 3, 9, "%.2f",x[0]);
      mvwprintw(kf_fix_win, 3, 27,"%.2f",x[1]);
      mvwprintw(kf_fix_win, 3, 49,"%.2f",x[2]);
      mvwprintw(kf_fix_win, 3, 70,"%.2f",x[3]);
      mvwprintw(kf_fix_win, 4, 10, "%.2f",x[4]);
      mvwprintw(kf_fix_win, 4, 30,"%.2f",x[5]);
      mvwprintw(kf_fix_win, 4, 45,"%.2f",x[6]);
      mvwprintw(kf_fix_win, 4, 70,"%.4f",x[7]);

      mvwprintw(kf_fix_win, 6, 10, "%.2f",dx[0]);
      mvwprintw(kf_fix_win, 6, 30, "%.2f",dx[1]);
      mvwprintw(kf_fix_win, 6, 50,"%.2f", dx[2]);
      mvwprintw(kf_fix_win, 6, 70, "%.2f",dx[3]);
      mvwprintw(kf_fix_win, 7, 10, "%.2f",dx[4]);
      mvwprintw(kf_fix_win, 7, 30,"%.2f ",dx[5]);
      mvwprintw(kf_fix_win, 7, 50,"%.4f", dx[6]);
      mvwprintw(kf_fix_win, 7, 72,"%.4f", dx[7]);

      mvwprintw(kf_fix_win, 9, 10, "%.3f",px[0]);
      mvwprintw(kf_fix_win, 9, 30,"%.3f", px[1]);
      mvwprintw(kf_fix_win, 9, 50,"%.3f", px[2]);
      mvwprintw(kf_fix_win, 9, 70, "%.3f",px[3]);
      mvwprintw(kf_fix_win, 10, 10, "%.3f",px[4]);
      mvwprintw(kf_fix_win, 10, 30,"%.3f",px[5]);
      mvwprintw(kf_fix_win, 10, 50,"%.4f",px[6]);
      mvwprintw(kf_fix_win, 10, 70,"%.4f",px[7]);

      refresh();
      update_panels();
      doupdate();

    }
}

/* Put all the windows */
void Display_GPS_win::init_wins(void)
{	
  char label[80];
  int info_x, info_y;

  track_win = newwin(max_row, max_col, 0,0);
  ls_fix_win = newwin(max_row, max_col, 0,0);
  kf_fix_win = newwin(max_row, max_col, 0,0);

  /* setup info window */
  info_x = (max_col-info_width)>>1;
  info_y = (max_row-info_height)>>1;
  info_win   = newwin(info_height,info_width,info_y,info_x);
  box(info_win,0,0);
  wattron(info_win,COLOR_PAIR(1));
  mvwprintw(info_win, 1,10,"Software IF GPS Rcvr Project");
  wattroff(info_win,COLOR_PAIR(1));
  wattron(info_win, COLOR_PAIR(2));
  mvwprintw(info_win, 3,1,"Author: Yu Lu");
  mvwprintw(info_win, 4,1,"Email:  softwareGNSS@gmail.com");
  mvwprintw(info_win, 5,1,"Date:   Aug,2008");
  mvwprintw(info_win, 7,1,"For more information, please visit the website: ");
  wattron(info_win, A_BOLD);
  mvwprintw(info_win, 8,6,"http://www.softwareGNSS.com/softwareGPS");
  wattroff(info_win,A_BOLD);
  mvwprintw(info_win, info_height-2,5,"Press any key to return.");
  wattroff(info_win,COLOR_PAIR(2));
  

  /* Attach a panel to each window */ 	/* Order is bottom up */
  track_panel = new_panel(track_win); 	/* Push 0, */
  ls_panel    = new_panel(ls_fix_win); 	/* Push 1, */
  kf_panel    = new_panel(kf_fix_win); 	/* Push 2, */
  info_panel  = new_panel(info_win);
  hide_panel(info_panel);

  /* Set up the user pointers to the next panel */
  set_panel_userptr(track_panel, ls_panel);
  set_panel_userptr(ls_panel, kf_panel);
  set_panel_userptr(kf_panel, track_panel);

  //Print title of each window
  sprintf(label,"Tracking Loop Status");
  win_show(track_win, label);
  sprintf(label,"LS Fix Result");
  win_show(ls_fix_win, label);
  sprintf(label,"KF Fix Result");
  win_show(kf_fix_win, label);

  track_show_header();
  ls_show_header();
  kf_show_header();
  top_panel(track_panel);
  current_top = track_panel; 
  /* Update the stacking order.tracking_panel will be on the top*/
  update_panels();
  doupdate();
}

/* Show the window with a border and a label */
void Display_GPS_win::win_show(WINDOW *win, char *label)
{
  int startx, half_width, str_leng;
  
  half_width = max_col>>1;
  str_leng = strlen(label);
  startx = half_width - (str_leng>>1);
  
  box(win, 0, 0);
  mvwaddch(win, 2, 0, ACS_LTEE); 
  mvwhline(win, 2, 1, ACS_HLINE, max_col - 2); 
  mvwaddch(win, 2, max_col - 1, ACS_RTEE); 

  mvwaddch(win, 15, 0, ACS_LTEE); 
  mvwhline(win, 15, 1, ACS_HLINE, max_col - 2); 
  mvwaddch(win, 15, max_col - 1, ACS_RTEE); 

  mvwaddch(win, 18, 0, ACS_LTEE); 
  mvwhline(win, 18, 1, ACS_HLINE, max_col - 2); 
  mvwaddch(win, 18, max_col - 1, ACS_RTEE); 
  wattron(win,COLOR_PAIR(1));
  mvwprintw(win,1,startx, "%s", label);
  wattroff(win,COLOR_PAIR(1));
  doupdate();
  refresh();
}

//print the header info for tracking loop status display
void Display_GPS_win::track_show_header(void)
{
  wattron(track_win, COLOR_PAIR(2));

  mvwprintw(track_win, 3,1,"CH");
  mvwprintw(track_win, 3,5,"PRN" );
  mvwprintw(track_win, 3,9,"ST");
  mvwprintw(track_win, 3,13, "Frame");
  mvwprintw(track_win, 3,19, "Z-Cnt");
  mvwprintw(track_win, 3,26, "Code_Freq");
  mvwprintw(track_win, 3,38, "Carr_Freq");
  mvwprintw(track_win, 3,50, "PRange");
  mvwprintw(track_win, 3,62, "I_out");
  mvwprintw(track_win, 3,70, "Q_out");
  mvwprintw(track_win, 3,78, "Elev");
  mvwprintw(track_win, 3,86, "Azim");

  mvwprintw(track_win, 16,1, "Current Processing: ");
  mvwprintw(track_win, 16,22, "          sec of totally %.6f sec",
	    data_duration/sampling_freq);
  mvwprintw(track_win, 17,1, "PVT resolution: ");

  mvwprintw(track_win, 19,1, "System Config Info:");
  
  mvwprintw(track_win, 20,1, "IF sampling Freq: %.6fMHz", sampling_freq*1E-6);
  mvwprintw(track_win, 21,1, "IF data length:   %d samples",   data_duration);
  mvwprintw(track_win, 22,1, "IF Data File:   %s",   sourcefile.c_str());

  
  mvwprintw(track_win, max_row-1,10, "TAB to switch displays. 'x' to exit. 'm' for more info");
  
  wattroff(track_win, COLOR_PAIR(2));
  refresh();
}

//print the header info for LS fix result display
void Display_GPS_win::ls_show_header(void)
{
  wattron(ls_fix_win, COLOR_PAIR(2));

  mvwprintw(ls_fix_win, 3, 1, "ECEF_x: ");
  mvwprintw(ls_fix_win, 3, 30,"ECEF_y: ");
  mvwprintw(ls_fix_win, 3, 60,"ECEF_z: ");
  mvwprintw(ls_fix_win, 4, 1, "ECEF_Vx: ");
  mvwprintw(ls_fix_win, 4, 30,"ECEF_Vy: ");
  mvwprintw(ls_fix_win, 4, 60,"ECEF_Vz: ");
  mvwprintw(ls_fix_win, 5, 1, "Bias: ");
  mvwprintw(ls_fix_win, 5, 30,"Drift: ");
  mvwprintw(ls_fix_win, 5, 60,"Local_T: ");
  mvwprintw(ls_fix_win, 6, 1, "Longitude:");
  mvwprintw(ls_fix_win, 6, 30,"Latitude:");
  mvwprintw(ls_fix_win, 6, 60,"Altitude: ");

  mvwprintw(ls_fix_win, 7, 1, "GDOP: ");
  mvwprintw(ls_fix_win, 7, 20,"PDOP: ");
  mvwprintw(ls_fix_win, 7, 40,"VDOP: ");
  mvwprintw(ls_fix_win, 7, 60,"HDOP: ");

  mvwprintw(ls_fix_win, 16,1, "Current Processing: ");
  mvwprintw(ls_fix_win, 16,22, "          sec of totally %.6f sec",
	    data_duration/sampling_freq);
  
  mvwprintw(ls_fix_win, 19,1, "System Config Info:");
  
  mvwprintw(ls_fix_win, 20,1, "IF sampling Freq: %.6fMHz", sampling_freq*1E-6);
  mvwprintw(ls_fix_win, 21,1, "IF data length:   %d samples",   data_duration);
  mvwprintw(ls_fix_win, 22,1, "IF Data File:   %s",   sourcefile.c_str());


  mvwprintw(ls_fix_win, max_row-1,10, "TAB to switch displays. 'x' to exit. 'm' for more info");

  wattroff(ls_fix_win, COLOR_PAIR(2));
  refresh();
}
//print the header info for KF fix result display
void Display_GPS_win::kf_show_header(void)
{
  wattron(kf_fix_win, COLOR_PAIR(2));

  mvwprintw(kf_fix_win, 3, 1, "ECEF_x: ");
  mvwprintw(kf_fix_win, 3, 20,"ECEF_y: ");
  mvwprintw(kf_fix_win, 3, 40,"ECEF_z: ");
  mvwprintw(kf_fix_win, 3, 60, "ECEF_Vx: ");
  mvwprintw(kf_fix_win, 4, 1, "ECEF_Vy: ");
  mvwprintw(kf_fix_win, 4, 20,"ECEF_Vz: ");
  mvwprintw(kf_fix_win, 4, 40,"Bias: ");
  mvwprintw(kf_fix_win, 4, 60,"Drift: ");

  mvwprintw(kf_fix_win, 6, 1, "Corr_x: ");
  mvwprintw(kf_fix_win, 6, 20,"Corr_y: ");
  mvwprintw(kf_fix_win, 6, 40,"Corr_z: ");
  mvwprintw(kf_fix_win, 6, 60, "Corr_Vx: ");
  mvwprintw(kf_fix_win, 7, 1, "Corr_Vy: ");
  mvwprintw(kf_fix_win, 7, 20,"Corr_Vz: ");
  mvwprintw(kf_fix_win, 7, 40,"Corr_Bias: ");
  mvwprintw(kf_fix_win, 7, 60,"Corr_Drift: ");

  mvwprintw(kf_fix_win, 9, 1, "Var_x: ");
  mvwprintw(kf_fix_win, 9, 20,"Var_y: ");
  mvwprintw(kf_fix_win, 9, 40,"Var_z: ");
  mvwprintw(kf_fix_win, 9, 60, "Var_Vx: ");
  mvwprintw(kf_fix_win, 10, 1, "Var_Vy: ");
  mvwprintw(kf_fix_win, 10, 20,"Var_Vz: ");
  mvwprintw(kf_fix_win, 10, 40,"Var_Bias: ");
  mvwprintw(kf_fix_win, 10, 60,"Var_Drift: ");

  
  mvwprintw(kf_fix_win, 19,1, "System Config Info:");
  
  mvwprintw(kf_fix_win, 20,1, "IF sampling Freq: %.6fMHz", sampling_freq*1E-6);
  mvwprintw(kf_fix_win, 21,1, "IF data length:   %d samples",   data_duration);
  mvwprintw(kf_fix_win, 22,1, "IF Data File:   %s",   sourcefile.c_str());


  mvwprintw(kf_fix_win, 16,1, "Current Processing: ");
  mvwprintw(kf_fix_win, 16,22, "          sec of totally %.6f sec",
	    data_duration/sampling_freq);

  mvwprintw(kf_fix_win, max_row-1,10, "TAB to switch displays. 'x' to exit. 'm' for more info");

  wattroff(kf_fix_win, COLOR_PAIR(2));
  refresh();
}

