/**************************************************************
 *This header file is used to enable or diasble different compilation
 *switch, thus control the log file generation.
 *
 **************************************************************/

//#define GPS_LOG_FILE

#ifdef GPS_LOG_FILE

//acquisition.h:
#define GPS_ACQMDL_RESULT
//gps_controller.h:
#define  GPS_CNTL_DEBUG
//gps_controller.h:
#define  xGPS_ACQ_RESULT
//gps_nav_fix.h:
#define GPS_NAVFIX_DBG
//gps_nav_fix.h:
#define GPS_NAVFIX_KF_DBG
#define GPS_NAVFIX_SV_DBG
//gps_nav_msg.h:
#define xGPS_MSGNAV_DBG
//kalman_filter.h:
#define GPS_KALMAN_DBG
//pos_math.h:
#define xPOS_MATH_DBG

#endif

//log to console window or not
//#define GPS_LOG_CONSOLE

