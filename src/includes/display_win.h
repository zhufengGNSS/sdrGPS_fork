#ifndef DISPLAY_WIN_H
#define DISPLAY_WIN_H

//The display use the ncurses library
#include <panel.h>
#include <iostream>
#include "softgps.h"

class Display_GPS_win{
public:
    Display_GPS_win(const Source_config& src_cfg);
    ~Display_GPS_win();
    
    void init(int, double);
    void release(void);
    int display_data(const char *buf);
    int process_log_buf( log_circular_buf *buf, int );
    void run(void);
protected:

private:
    //Three windows to display tracking loop status, 
    // LS fix result, and KF fix resule, respectively
    WINDOW *track_win, *ls_fix_win, *kf_fix_win, *info_win; 
    PANEL  *track_panel, *ls_panel, *kf_panel, *current_top;
    PANEL  *info_panel;
    int max_row, max_col; 
    double sampling_freq;
    string sourcefile;
    int data_duration,current_data_cnt;
    int jump_ch_idx;

    // static const vars
    static const double R2D;
    static const int ROW_THRD, COL_THRD;
    static const int info_width, info_height;
    static const char jump_char[4];

    void init_wins(void);
    void win_show(WINDOW *win, char *label);

    void show_tracking(const char *buf);
    void show_ls_fix(const char *buf);
    void show_kf_fix(const char *buf);
    void track_show_header(void);
    void ls_show_header(void);
    void kf_show_header(void);
};



#endif
