#ifndef NAV_KALMAN_FILTER_H
#define NAV_KALMAN_FILTER_H

#include <fstream>
#include <sstream>
#include "softgps.h"
#include "gpslibconfig.h"

//#define GPS_KALMAN_DBG

#define P_IDX               0
#define PV_IDX              1
#define PVA_IDX             2

#define P_MDL_STATE_NUM     5
#define PV_MDL_STATE_NUM    8
#define PVA_MDL_STATE_NUM   11

#define MAX_MEAS_NUM        30

class GPS_pos_math;
class Matrix;

// P model structure
typedef struct{
  double x[P_MDL_STATE_NUM];  // kalman filter state
  double k[P_MDL_STATE_NUM*MAX_MEAS_NUM];  // kalman gain
  double dx[P_MDL_STATE_NUM];// error state
  double p[P_MDL_STATE_NUM*P_MDL_STATE_NUM];  // p
  double p_minus[P_MDL_STATE_NUM*P_MDL_STATE_NUM];  // p-
  double phi[P_MDL_STATE_NUM*P_MDL_STATE_NUM];  // Phi
  double qd[P_MDL_STATE_NUM*P_MDL_STATE_NUM];  // Q
  double residual;
  double res_variance;
  double meas[MAX_MEAS_NUM];  // measurement
  double hm[MAX_MEAS_NUM*P_MDL_STATE_NUM];  // h matrix
  double Rm[MAX_MEAS_NUM*MAX_MEAS_NUM]; // R matrix of measurements
  double Pq_pos_var, Pq_clk_var;
  int    k_dim;
}p_mdl_struct;

// PV model structure
typedef struct{
  double x[PV_MDL_STATE_NUM];  // kalman filter state
  double k[PV_MDL_STATE_NUM*MAX_MEAS_NUM];  // kalman gain
  double dx[PV_MDL_STATE_NUM]; // error state
  double p[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];  // p
  double p_minus[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];  // p-
  double phi[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];  // Phi
  double qd[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];  // Q
  double residual;
  double res_variance;
  double meas[MAX_MEAS_NUM];  // measurement
  double hm[MAX_MEAS_NUM*PV_MDL_STATE_NUM];  // h matrix
  double Rm[MAX_MEAS_NUM*MAX_MEAS_NUM]; // R matrix of measurements
  double PVq_pos_var, PVq_clk_var;
  int    k_dim;
}pv_mdl_struct;

// PVA model structure
typedef struct{
  double x[PVA_MDL_STATE_NUM];  // kalman filter state
  double k[PVA_MDL_STATE_NUM*MAX_MEAS_NUM];  // kalman gain
  double dx[PVA_MDL_STATE_NUM]; // error state
  double p[PVA_MDL_STATE_NUM*PVA_MDL_STATE_NUM];  // p
  double p_minus[PVA_MDL_STATE_NUM*PVA_MDL_STATE_NUM];  // p-
  double phi[PVA_MDL_STATE_NUM*PVA_MDL_STATE_NUM];  // Phi
  double qd[PVA_MDL_STATE_NUM*PVA_MDL_STATE_NUM];  // Q
  double residual;
  double res_variance;
  double meas[MAX_MEAS_NUM];  // measurement
  double hm[MAX_MEAS_NUM*PVA_MDL_STATE_NUM];  // h matrix
  double Rm[MAX_MEAS_NUM*MAX_MEAS_NUM]; // R matrix of measurements
  int    k_dim;
}pva_mdl_struct;


class Kalman{
 public:
  Kalman(char, GPS_pos_math&, Matrix& );
  ~Kalman();
  void set_mdl_id(char id);
  char get_mdl_id(void);
  void init_kf(const nav_state& ls_fix );
  void nav_kf_fix(const kf_group_meas &kf_meas );
  
 protected:

 private:

  void update_phi_q(double);
  void update_state(void);
  void propagate_state(double);
  void calc_correction(void);
  void update_p_(void);
  void set_H_R(const kf_group_meas &);
  void log_kf_data(void);


  // the following functions are for P_model KF
  void Pmodel_update_phi_q(double);
  void Pmodel_update_state(void);
  void Pmodel_propagate_state(double);
  void Pmodel_calc_correction(void);
  void Pmodel_update_p_(void);
  void Pmodel_set_H_R(const kf_group_meas &);
  void Pmodel_log_kf_data(void);

  // the following functions are for PV_model KF
  void PVmodel_update_phi_q(double);
  void PVmodel_update_state(void);
  void PVmodel_propagate_state(double);
  void PVmodel_calc_correction(void);
  void PVmodel_update_p_(void);
  void PVmodel_set_H_R(const kf_group_meas &);
  void PVmodel_log_kf_data(void);

  // the following functions are for PVA_model KF
  void PVAmodel_update_phi_q(double);
  void PVAmodel_update_state(void);
  void PVAmodel_calc_correction(void);
  void PVAmodel_update_p_(void);
  void PVAmodel_set_H_R(const kf_group_meas &);

  GPS_pos_math &posmath_ref;
  Matrix &kf_matrix;
  p_mdl_struct   p_model;
  pv_mdl_struct  pv_model;
  pva_mdl_struct pva_model;

#ifdef GPS_KALMAN_DBG
  //ofstream debughdle;
  ofstream p_debughdle;
  ofstream pv_debughdle;
#endif

  char model_id;
  ostringstream  dbg_str;
  double local_time; 
  static const int mdl_state_num[3]; 
  static const double SpeedOfLight;
  static const double OmegaDotEarth;
};
    
#endif
