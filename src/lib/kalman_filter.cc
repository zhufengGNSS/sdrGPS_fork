/*:nav_kalman_filter.cc
********************************************************
 * Kalman filter for GPS navigation fix solution
 * 
 * This class is used to apply kalman filter to navigation fix 
 * P- PV- and PVA- model are implemented here
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        May, 2007
 *******************************************************/
#include <assert.h>
#include <iostream>
#include <math.h>
#include <iomanip>

#include "includes/kalman_filter.h"
#include "includes/pos_math.h"
#include "includes/matrix.h"

const int Kalman::mdl_state_num[3]= {P_MDL_STATE_NUM, 
				     PV_MDL_STATE_NUM,
				     PVA_MDL_STATE_NUM};
const double Kalman::SpeedOfLight = 2.99792458e8;
const double Kalman::OmegaDotEarth=7.292115146E-5L;

Kalman::Kalman(char id, 
	       GPS_pos_math& pos_math, 
	       Matrix& matrix):posmath_ref(pos_math), kf_matrix(matrix)
{
  model_id = id;
    dbg_str.str("");
  
#ifdef GPS_KALMAN_DBG
    {
      time_t file_time;
      char buff[40];
      ostringstream filenm(" ");
      ostringstream filenm_pv(" ");

      file_time = time(NULL);
      strftime(buff,40, "_%d%m%Y_%H%M.m",
	       localtime(&file_time));
	  filenm<<"./data/kf_P"<<buff; 		
      filenm_pv<<"./data/kf_PV"<<buff;
	  
      p_debughdle.open(filenm.str().c_str());
      if( !p_debughdle.is_open() )
        cerr <<"Failed to open debug file for GPS kf_P_model log" << endl;
      

      pv_debughdle.open(filenm_pv.str().c_str());
      if( !pv_debughdle.is_open() )
        cerr <<"Failed to open debug file for GPS kf_PV_model log" << endl;
    }

#endif
  
}

Kalman::~Kalman(void)
{
#ifdef GPS_KALMAN_DBG
  if( p_debughdle.is_open() )
    p_debughdle.close();

  if( pv_debughdle.is_open() )
    pv_debughdle.close();
  
#endif

}

void Kalman::set_mdl_id(char id)
{
    model_id = id;
}

char Kalman::get_mdl_id(void)
{
    return model_id;
}


void Kalman::init_kf( const nav_state& ls_fix)
{

    int i;
  local_time = ls_fix.gps_time.sec - ls_fix.clk_bias;
  
  ///////////////////////////////////////////////////////////////////
  //init P_model first
  //////////////////////////////////////////////////////////////////
  
  //setup the state vector
  p_model.x[0] = ls_fix.pos.x;
  p_model.x[1] = ls_fix.pos.y;
  p_model.x[2] = ls_fix.pos.z;
  p_model.x[3] = ls_fix.clk_bias;
  p_model.x[4] = ls_fix.clk_drift;

  //init the state covariance matrix
    for( i=0; i<P_MDL_STATE_NUM*P_MDL_STATE_NUM; i++)
    {
      p_model.p[i] = 0.0;
    }
  p_model.p[0] = 1.0E6;
  p_model.p[1*P_MDL_STATE_NUM+1] = 1.0E6;
  p_model.p[2*P_MDL_STATE_NUM+2] = 1.0E6;
  p_model.p[3*P_MDL_STATE_NUM+3] = 1.0E6;
  p_model.p[4*P_MDL_STATE_NUM+4] = 1E6;
  p_model.Pq_clk_var  = 0.04;  // 0.2 m/s
  p_model.Pq_pos_var  = 0.25;  //  0.5 m
  //reset Phi matrix to be I
  kf_matrix.matrix_identity(p_model.phi,
			    P_MDL_STATE_NUM,
			    P_MDL_STATE_NUM);
  
  //reset Qd matrix to be 0
  for( i=0; i<P_MDL_STATE_NUM*P_MDL_STATE_NUM; i++)
    p_model.qd[i] = 0;
  
  p_model.k_dim = 0;

  ///////////////////////////////////////////////////////////////////
  //init P_model first
  //////////////////////////////////////////////////////////////////
  //setup the state vector
  pv_model.x[0] = ls_fix.pos.x;
  pv_model.x[1] = ls_fix.pos.y;
  pv_model.x[2] = ls_fix.pos.z;
  pv_model.x[3] = ls_fix.vel.x;
  pv_model.x[4] = ls_fix.vel.y;
  pv_model.x[5] = ls_fix.vel.z;
  pv_model.x[6] = ls_fix.clk_bias;
  pv_model.x[7] = ls_fix.clk_drift;

  //init the state covariance matrix
    for( i=0; i<PV_MDL_STATE_NUM*PV_MDL_STATE_NUM; i++)
    {
      pv_model.p[i] = 0.0;
    }
  pv_model.p[0] = 1.0E6;
  pv_model.p[1*PV_MDL_STATE_NUM+1] = 1.0E6;
  pv_model.p[2*PV_MDL_STATE_NUM+2] = 1.0E6;
  pv_model.p[3*PV_MDL_STATE_NUM+3] = 1.0E6;
  pv_model.p[4*PV_MDL_STATE_NUM+4] = 1E6;
  pv_model.p[5*PV_MDL_STATE_NUM+5] = 1.0E6;
  pv_model.p[6*PV_MDL_STATE_NUM+6] = 1.0E6;
  pv_model.p[7*PV_MDL_STATE_NUM+7] = 1E6;
  pv_model.PVq_clk_var  = 0.25;   //0.5m/s
  pv_model.PVq_pos_var  = 9.0 ;   // 3 m
  //reset Phi matrix to be I
  kf_matrix.matrix_identity(pv_model.phi,
			    PV_MDL_STATE_NUM,
			    PV_MDL_STATE_NUM);
  
  //reset Qd matrix to be 0
  for( i=0; i<PV_MDL_STATE_NUM*PV_MDL_STATE_NUM; i++)
    pv_model.qd[i] = 0;
  
  pv_model.k_dim = 0;

}


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
// The following functions are for P_model KF
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void Kalman::Pmodel_update_phi_q(double dt)
{
  // first for P_model
  // The F matrix is 
  //    |                |
  //    | 0  0  0  0  0  | x 
  //    | 0  0  0  0  0  | y
  // F =| 0  0  0  0  0  | z 
  //    | 0  0  0  0  1  | bias
  //    | 0  0  0  0  0  | dft
  //    |                |
  //   
  //    | 1  0  0  0  0  |
  //    | 0  1  0  0  0  |
  //Phi=| 0  0  1  0  0  |
  //    | 0  0  0  1  t  |
  //    | 0  0  0  0  1  |

  kf_matrix.matrix_identity(p_model.phi,P_MDL_STATE_NUM, P_MDL_STATE_NUM) ;
  p_model.phi[3*P_MDL_STATE_NUM+4] = dt; 

  // then setup Qd
  kf_matrix.matrix_identity(p_model.qd, 
			    P_MDL_STATE_NUM, 
			    P_MDL_STATE_NUM) ;
  
  p_model.qd[0*P_MDL_STATE_NUM+0] = dt*p_model.Pq_pos_var;
  p_model.qd[1*P_MDL_STATE_NUM+1] = dt*p_model.Pq_pos_var;
  p_model.qd[2*P_MDL_STATE_NUM+2] = dt*p_model.Pq_pos_var;
  p_model.qd[3*P_MDL_STATE_NUM+3] = p_model.Pq_clk_var*dt*dt*dt/3.0;
  p_model.qd[3*P_MDL_STATE_NUM+4] = p_model.Pq_clk_var*dt*dt/2.0;
  p_model.qd[4*P_MDL_STATE_NUM+3] = p_model.Pq_clk_var*dt*dt/2.0;
  p_model.qd[4*P_MDL_STATE_NUM+4] = p_model.Pq_clk_var*dt;

}
  
void Kalman::Pmodel_propagate_state(double dt)
{
	// propagate  bias only
	p_model.x[3] = p_model.x[3] + p_model.x[4]*dt;
  
}
void Kalman::Pmodel_update_state(void)
{
  double tmp_t0[P_MDL_STATE_NUM*P_MDL_STATE_NUM];
  double tmp_t1[P_MDL_STATE_NUM*P_MDL_STATE_NUM];
  double tmp_t2[P_MDL_STATE_NUM*P_MDL_STATE_NUM];
  
  
  // add the corrections
  kf_matrix.matrix_add(p_model.x,
		       p_model.x,
		       p_model.dx,
		       P_MDL_STATE_NUM,
		       1);

  // update P
  // tmp_t0 = K*H,  (5 X 5 )
  kf_matrix.matrix_mul(tmp_t0,
		       p_model.k,
		       p_model.hm,
		       P_MDL_STATE_NUM,
		       p_model.k_dim,
		       P_MDL_STATE_NUM);

  kf_matrix.matrix_identity(tmp_t1,
			    P_MDL_STATE_NUM,
			    P_MDL_STATE_NUM);
  // tmp_t2 = (I - K*H )
  kf_matrix.matrix_sub(tmp_t2,
		       tmp_t1,
		       tmp_t0,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM);
  
  // P = ( I-K*H) * P-
  kf_matrix.matrix_mul(p_model.p,
		       tmp_t2,
		       p_model.p_minus,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM);
		       
  //reset Phi matrix to be I
  kf_matrix.matrix_identity(p_model.phi,
			    P_MDL_STATE_NUM,
			    P_MDL_STATE_NUM);
  
  //reset Qd matrix to be 0
  for( int i=0; i<P_MDL_STATE_NUM*P_MDL_STATE_NUM; i++)
    p_model.qd[i] = 0;

#ifdef xGPS_KALMAN_DBG

  p_debughdle.precision(16);
  p_debughdle<<"P_x: "
	     << p_model.x[0] << " "
	     << p_model.x[1] << " "
	     << p_model.x[2] << " "
	     << p_model.x[3] << " "
	     << p_model.x[4] << " "
	     <<endl;
#endif
 
  
}
void Kalman::Pmodel_calc_correction(void)
{
  double tmp_t0[MAX_MEAS_NUM*MAX_MEAS_NUM];
  double tmp_t1[MAX_MEAS_NUM*MAX_MEAS_NUM];
  double tmp_t2[MAX_MEAS_NUM*MAX_MEAS_NUM];
 
  // tmp_t1 = H' , (5 X k_dim)
  kf_matrix.matrix_transpose(tmp_t1, 
			     p_model.hm,
			     p_model.k_dim,
			     P_MDL_STATE_NUM);
			     
  // tmp_t0  = p_minus*H' , (5 X k_dim )
  kf_matrix.matrix_mul(tmp_t0, 
		       p_model.p_minus, 
		       tmp_t1,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM,
		       p_model.k_dim);
  
  //tmp_t1 = H*P_minux*H', (k_dim X k_dim )
  kf_matrix.matrix_mul(tmp_t1, 
		       p_model.hm, 
		       tmp_t0,
		       p_model.k_dim,
		       P_MDL_STATE_NUM,
		       p_model.k_dim);

  // tmp_t2 = H*P-*H' + R
  kf_matrix.matrix_add(tmp_t2, 
		       tmp_t1,
		       p_model.Rm,
		       p_model.k_dim,
		       p_model.k_dim);
  
  //tmp_t1 = inv( H*P-*H' + R ), (k_dim X k_dim)
  kf_matrix.matrix_inverse(tmp_t1, 
			   tmp_t2,
			   p_model.k_dim);

  //k = P-*H'*(inv(H*P-*H' + R), (5 X k_dim )
  kf_matrix.matrix_mul(p_model.k,
		       tmp_t0,
		       tmp_t1,
		       P_MDL_STATE_NUM,
		       p_model.k_dim,
		       p_model.k_dim);

  //dx = k*dy
  kf_matrix.matrix_mul(p_model.dx,
		       p_model.k,
		       p_model.meas,
		       P_MDL_STATE_NUM,
		       p_model.k_dim,
		       1);
#ifdef xGPS_KALMAN_DBG
  p_debughdle.precision(16);
  p_debughdle<<"P_dx: "
	     << p_model.dx[0] << " "
	     << p_model.dx[1] << " "
	     << p_model.dx[2] << " "
	     << p_model.dx[3] << " "
	     << p_model.dx[4] << " "
	     <<endl;
#endif
    

}

void Kalman::Pmodel_update_p_(void)
{
  double tmp_t0[P_MDL_STATE_NUM*P_MDL_STATE_NUM];
  double tmp_t1[P_MDL_STATE_NUM*P_MDL_STATE_NUM];
  
  //t0 = phi*p
  kf_matrix.matrix_mul(tmp_t0, 
		       p_model.phi, 
		       p_model.p, 
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM);

  //t1=phi'
  kf_matrix.matrix_transpose(tmp_t1, 
			     p_model.phi, 
			     P_MDL_STATE_NUM,
			     P_MDL_STATE_NUM);
  
  //p_=phi*p*phi'
  kf_matrix.matrix_mul(p_model.p_minus,
		       tmp_t0, 
		       tmp_t1,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM);
  
  //p_ = phi*p*phi+qd
  kf_matrix.matrix_add(p_model.p_minus,
		       p_model.p_minus,
		       p_model.qd,
		       P_MDL_STATE_NUM,
		       P_MDL_STATE_NUM);


}
//#define USE_DOPPLER_P_MDL
void Kalman::Pmodel_set_H_R(const kf_group_meas &kf_meas)
{
  unsigned int i, line_count;
  double alpha;
  cartstruc tmp_range_vec;

  // set the K matrix, R matrix dimension
#ifdef USE_DOPPLER_P_MDL
  p_model.k_dim = kf_meas.sv_count*2;
#else
  p_model.k_dim = kf_meas.sv_count;
#endif
  // each SV provides two meas: pseudorange and doppler
  for(i=0; i<kf_meas.sv_count; i++)
    {
      //H vector and bias from pseudorange meas
#ifdef USE_DOPPLER_P_MDL
      p_model.hm[2*i*P_MDL_STATE_NUM+0] = -kf_meas.meas[i].h_vec[0];
      p_model.hm[2*i*P_MDL_STATE_NUM+1] = -kf_meas.meas[i].h_vec[1];
      p_model.hm[2*i*P_MDL_STATE_NUM+2] = -kf_meas.meas[i].h_vec[2];
      p_model.hm[2*i*P_MDL_STATE_NUM+3] = 1.0;
      p_model.hm[2*i*P_MDL_STATE_NUM+4] = 0.0;

      // remove the effect of the earth's rotation
      alpha = (kf_meas.meas[i].pseudo_range/SpeedOfLight)*OmegaDotEarth;

      tmp_range_vec.x = kf_meas.meas[i].sv_pos[0]*cos(alpha) 
	+ kf_meas.meas[i].sv_pos[1]*sin(alpha) 
	- p_model.x[0];

      tmp_range_vec.y = -kf_meas.meas[i].sv_pos[0]*sin(alpha) 
	+ kf_meas.meas[i].sv_pos[1]*cos(alpha)
	- p_model.x[1];

      tmp_range_vec.z = kf_meas.meas[i].sv_pos[2] - p_model.x[2];

      
      p_model.meas[2*i] = kf_meas.meas[i].pseudo_range 
	- posmath_ref.vec_length(tmp_range_vec.x,
				 tmp_range_vec.y,
				 tmp_range_vec.z)
	- p_model.x[3];
      
      //H vector and drift from doppler meas
      p_model.hm[(2*i+1)*P_MDL_STATE_NUM+0] = 0.0;
      p_model.hm[(2*i+1)*P_MDL_STATE_NUM+1] = 0.0;
      p_model.hm[(2*i+1)*P_MDL_STATE_NUM+2] = 0.0;
      p_model.hm[(2*i+1)*P_MDL_STATE_NUM+3] = 0.0;
      p_model.hm[(2*i+1)*P_MDL_STATE_NUM+4] = 1.0;
      p_model.meas[2*i+1] = (-kf_meas.meas[i].doppler - p_model.x[4]);
#else
      p_model.hm[i*P_MDL_STATE_NUM+0] = -kf_meas.meas[i].h_vec[0];
      p_model.hm[i*P_MDL_STATE_NUM+1] = -kf_meas.meas[i].h_vec[1];
      p_model.hm[i*P_MDL_STATE_NUM+2] = -kf_meas.meas[i].h_vec[2];
      p_model.hm[i*P_MDL_STATE_NUM+3] = 1.0;
      p_model.hm[i*P_MDL_STATE_NUM+4] = 0.0;

      // remove the effect of the earth's rotation
      alpha = (kf_meas.meas[i].pseudo_range/SpeedOfLight)*OmegaDotEarth;

      tmp_range_vec.x = kf_meas.meas[i].sv_pos[0]*cos(alpha) 
	+ kf_meas.meas[i].sv_pos[1]*sin(alpha) 
	- p_model.x[0];

      tmp_range_vec.y = -kf_meas.meas[i].sv_pos[0]*sin(alpha) 
	+ kf_meas.meas[i].sv_pos[1]*cos(alpha)
	- p_model.x[1];

      tmp_range_vec.z = kf_meas.meas[i].sv_pos[2] - p_model.x[2];

      
      p_model.meas[i] = kf_meas.meas[i].pseudo_range 
	- posmath_ref.vec_length(tmp_range_vec.x,
				 tmp_range_vec.y,
				 tmp_range_vec.z)
	- p_model.x[3];

#endif
    }

  line_count=0;
  kf_matrix.matrix_identity(p_model.Rm,
			    p_model.k_dim,
			    p_model.k_dim);
  for( i=0; i<kf_meas.sv_count; i++)
    {
#ifdef USE_DOPPLER_P_MDL
      p_model.Rm[2*i*p_model.k_dim + line_count] = 9.0;     // std: 3m
      line_count++;
      p_model.Rm[(2*i+1)*p_model.k_dim + line_count] = 10000000.16;    // std: 0.4m/s
      line_count++;
#else
      p_model.Rm[i*p_model.k_dim + i] = 9.0;     // std: 3m
#endif
      
    }

}

void Kalman::Pmodel_log_kf_data()
{
  int i;
  dbg_str<<setw(12)<<setprecision(4)<<setiosflags(ios::fixed|ios::showpoint);
  //p_debughdle<<"P_model: ";

  // log state first
  for( i=0; i<P_MDL_STATE_NUM; i++)
    dbg_str<< p_model.x[i] << " " ;
  // log correction
  for( i=0; i<P_MDL_STATE_NUM; i++)
    dbg_str<< p_model.dx[i] << " " ;
  // log p_matrix
  for( i=0; i<P_MDL_STATE_NUM; i++)
    dbg_str<< p_model.p[i*P_MDL_STATE_NUM + i] << " " ;
  dbg_str <<endl;

#ifdef GPS_KALMAN_DBG
  p_debughdle<<dbg_str.str();

#endif
  log_msg("DATA KF 0 " + dbg_str.str());
  dbg_str.str("");
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
// The following functions are for PV_model KF
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void Kalman::PVmodel_update_phi_q(double dt)
{

  int i; 
  // first for PV_model
  // The F matrix is 
  //    |                         |
  //    | 0  0  0  1  0  0  0  0  |  x
  //    | 0  0  0  0  1  0  0  0  |  y
  // F =| 0  0  0  0  0  1  0  0  |  z
  //    | 0  0  0  0  0  0  0  0  |  vx
  //    | 0  0  0  0  0  0  0  0  |  vy
  //    | 0  0  0  0  0  0  0  0  |  vz
  //    | 0  0  0  0  0  0  0  1  |  bias
  //    | 0  0  0  0  0  0  0  0  |  dft
  // exmp(F*dT) = I + F*dT 
  //
  //    | 1  0  0  t  0  0  0  0  |
  //    | 0  1  0  0  t  0  0  0  |
  //    | 0  0  1  0  0  t  0  0  |
  //phi=| 0  0  0  1  0  0  0  0  |
  //    | 0  0  0  0  1  0  0  0  |
  //    | 0  0  0  0  0  1  0  0  |
  //    | 0  0  0  0  0  0  1  t  |
  //    | 0  0  0  0  0  0  0  1  |

  kf_matrix.matrix_identity(pv_model.phi,PV_MDL_STATE_NUM, PV_MDL_STATE_NUM) ;
  pv_model.phi[0*PV_MDL_STATE_NUM+3] = dt; 
  pv_model.phi[1*PV_MDL_STATE_NUM+4] = dt; 
  pv_model.phi[2*PV_MDL_STATE_NUM+5] = dt; 
  pv_model.phi[6*PV_MDL_STATE_NUM+7] = dt; 

  // then setup Qd
  kf_matrix.matrix_identity(pv_model.qd, 
			    PV_MDL_STATE_NUM, 
			    PV_MDL_STATE_NUM) ;
  
  // for pos ,vel 
  for( i=0; i<3; i++)
  {
      pv_model.qd[i*PV_MDL_STATE_NUM+i]     = pv_model.PVq_pos_var*dt*dt*dt/3.0;
      pv_model.qd[i*PV_MDL_STATE_NUM+(i+3)] = pv_model.PVq_pos_var*dt*dt/2.0;
      pv_model.qd[(i+3)*PV_MDL_STATE_NUM+i] = pv_model.PVq_pos_var*dt*dt/2.0;
      pv_model.qd[(i+3)*PV_MDL_STATE_NUM+(i+3)] = pv_model.PVq_pos_var*dt;
  }
  // for clk bias, drift
  pv_model.qd[6*PV_MDL_STATE_NUM+6] = pv_model.PVq_clk_var*dt*dt*dt/3.0;
  pv_model.qd[6*PV_MDL_STATE_NUM+7] = pv_model.PVq_clk_var*dt*dt/2.0;
  pv_model.qd[7*PV_MDL_STATE_NUM+6] = pv_model.PVq_clk_var*dt*dt/2.0;
  pv_model.qd[7*PV_MDL_STATE_NUM+7] = pv_model.PVq_clk_var*dt;

}
void Kalman::PVmodel_propagate_state(double dt)
{
	// propagate pos and bias
	pv_model.x[0] = pv_model.x[0] + pv_model.x[3]*dt;
	pv_model.x[1] = pv_model.x[1] + pv_model.x[4]*dt;
	pv_model.x[2] = pv_model.x[2] + pv_model.x[5]*dt;
  
	pv_model.x[6] = pv_model.x[6] + pv_model.x[7]*dt;
  
}
void Kalman::PVmodel_update_state(void)
{
  double tmp_t0[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];
  double tmp_t1[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];
  double tmp_t2[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];
  
  
  // add the corrections
  kf_matrix.matrix_add(pv_model.x,
		       pv_model.x,
		       pv_model.dx,
		       PV_MDL_STATE_NUM,
		       1);

  // update P
  // tmp_t0 = K*H,  (8 X 8 )
  kf_matrix.matrix_mul(tmp_t0,
		       pv_model.k,
		       pv_model.hm,
		       PV_MDL_STATE_NUM,
		       pv_model.k_dim,
		       PV_MDL_STATE_NUM);

  kf_matrix.matrix_identity(tmp_t1,
			    PV_MDL_STATE_NUM,
			    PV_MDL_STATE_NUM);
  // tmp_t2 = (I - K*H )
  kf_matrix.matrix_sub(tmp_t2,
		       tmp_t1,
		       tmp_t0,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM);
  
  // P = ( I-K*H) * P-
  kf_matrix.matrix_mul(pv_model.p,
		       tmp_t2,
		       pv_model.p_minus,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM);
		       
  //reset Phi matrix to be I
  kf_matrix.matrix_identity(pv_model.phi,
			    PV_MDL_STATE_NUM,
			    PV_MDL_STATE_NUM);
  
  //reset Qd matrix to be 0
  for( int i=0; i<PV_MDL_STATE_NUM*PV_MDL_STATE_NUM; i++)
    pv_model.qd[i] = 0;

#ifdef xGPS_KALMAN_DBG

  pv_debughdle.precision(16);
  pv_debughdle<<"PV_x: "
	      << pv_model.x[0] << " "
	      << pv_model.x[1] << " "
	      << pv_model.x[2] << " "
	      << pv_model.x[3] << " "
	      << pv_model.x[4] << " "
	      << pv_model.x[5] << " "
	      << pv_model.x[6] << " "
	      << pv_model.x[7] << " "
	      <<endl;
#endif
 
  
}
void Kalman::PVmodel_calc_correction(void)
{
  double tmp_t0[MAX_MEAS_NUM*MAX_MEAS_NUM];
  double tmp_t1[MAX_MEAS_NUM*MAX_MEAS_NUM];
  double tmp_t2[MAX_MEAS_NUM*MAX_MEAS_NUM];
 
  // tmp_t1 = H' , (8 X k_dim)
  kf_matrix.matrix_transpose(tmp_t1, 
			     pv_model.hm,
			     pv_model.k_dim,
			     PV_MDL_STATE_NUM);
			     
  // tmp_t0  = p_minus*H' , (8 X k_dim )
  kf_matrix.matrix_mul(tmp_t0, 
		       pv_model.p_minus, 
		       tmp_t1,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM,
		       pv_model.k_dim);
  
  //tmp_t1 = H*P_minux*H', (k_dim X k_dim )
  kf_matrix.matrix_mul(tmp_t1, 
		       pv_model.hm, 
		       tmp_t0,
		       pv_model.k_dim,
		       PV_MDL_STATE_NUM,
		       pv_model.k_dim);

  // tmp_t2 = H*P-*H' + R
  kf_matrix.matrix_add(tmp_t2, 
		       tmp_t1,
		       pv_model.Rm,
		       pv_model.k_dim,
		       pv_model.k_dim);
  
  //tmp_t1 = inv( H*P-*H' + R ), (k_dim X k_dim)
  kf_matrix.matrix_inverse(tmp_t1, 
			   tmp_t2,
			   pv_model.k_dim);

  //k = P-*H'*(inv(H*P-*H' + R), (8 X k_dim )
  kf_matrix.matrix_mul(pv_model.k,
		       tmp_t0,
		       tmp_t1,
		       PV_MDL_STATE_NUM,
		       pv_model.k_dim,
		       pv_model.k_dim);

  //dx = k*dy
  kf_matrix.matrix_mul(pv_model.dx,
		       pv_model.k,
		       pv_model.meas,
		       PV_MDL_STATE_NUM,
		       pv_model.k_dim,
		       1);
#ifdef xGPS_KALMAN_DBG
  pv_debughdle.precision(16);
  pv_debughdle<<"PV_dx: "
	      << pv_model.dx[0] << " "
	      << pv_model.dx[1] << " "
	      << pv_model.dx[2] << " "
	      << pv_model.dx[3] << " "
	      << pv_model.dx[4] << " "
	      << pv_model.dx[5] << " "
	      << pv_model.dx[6] << " "
	      << pv_model.dx[7] << " "
	      <<endl;
#endif
    

}

void Kalman::PVmodel_update_p_(void)
{
  double tmp_t0[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];
  double tmp_t1[PV_MDL_STATE_NUM*PV_MDL_STATE_NUM];
  
  //t0 = phi*p
  kf_matrix.matrix_mul(tmp_t0, 
		       pv_model.phi, 
		       pv_model.p, 
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM);

  //t1=phi'
  kf_matrix.matrix_transpose(tmp_t1, 
			     pv_model.phi, 
			     PV_MDL_STATE_NUM,
			     PV_MDL_STATE_NUM);
  
  //p_=phi*p*phi'
  kf_matrix.matrix_mul(pv_model.p_minus,
		       tmp_t0, 
		       tmp_t1,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM);
  
  //p_ = phi*p*phi+qd
  kf_matrix.matrix_add(pv_model.p_minus,
		       pv_model.p_minus,
		       pv_model.qd,
		       PV_MDL_STATE_NUM,
		       PV_MDL_STATE_NUM);


}

void Kalman::PVmodel_set_H_R(const kf_group_meas &kf_meas)
{
  unsigned int i, j, line_count;
  double alpha;
  cartstruc tmp_range_vec;
#ifdef xGPS_KALMAN_DBG
  pv_debughdle.precision(16);
  pv_debughdle<<"KF res: "<<kf_meas.local_t<<" " ;
#endif

  // set the K matrix, R matrix dimension
  pv_model.k_dim = kf_meas.sv_count*2;
  
  // each SV provides two meas: pseudorange and doppler
  for(i=0; i<kf_meas.sv_count; i++)
    {
      // first init the Hm to be all zeros
      for ( j=0; j<PV_MDL_STATE_NUM; j++)
	{
	  pv_model.hm[2*i*PV_MDL_STATE_NUM+j] = 0.0;
	  pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+j] = 0.0;
	}
      
      //H vector and bias from pseudorange meas
      pv_model.hm[2*i*PV_MDL_STATE_NUM+0] = -kf_meas.meas[i].h_vec[0];
      pv_model.hm[2*i*PV_MDL_STATE_NUM+1] = -kf_meas.meas[i].h_vec[1];
      pv_model.hm[2*i*PV_MDL_STATE_NUM+2] = -kf_meas.meas[i].h_vec[2];
      pv_model.hm[2*i*PV_MDL_STATE_NUM+6] = 1.0;


      // remove the effect of the earth's rotation
      alpha = (kf_meas.meas[i].pseudo_range/SpeedOfLight)*OmegaDotEarth;

      tmp_range_vec.x = kf_meas.meas[i].sv_pos[0]*cos(alpha) 
	+ kf_meas.meas[i].sv_pos[1]*sin(alpha) 
	- pv_model.x[0];

      tmp_range_vec.y = -kf_meas.meas[i].sv_pos[0]*sin(alpha) 
	+ kf_meas.meas[i].sv_pos[1]*cos(alpha)
	- pv_model.x[1];

      tmp_range_vec.z = kf_meas.meas[i].sv_pos[2] - pv_model.x[2];

      
      pv_model.meas[2*i] = kf_meas.meas[i].pseudo_range 
	- posmath_ref.vec_length(tmp_range_vec.x,
				 tmp_range_vec.y,
				 tmp_range_vec.z)
	- pv_model.x[6];
      
      //H vector and drift from doppler meas
      pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+3] = -kf_meas.meas[i].h_vec[0];
      pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+4] = -kf_meas.meas[i].h_vec[1];
      pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+5] = -kf_meas.meas[i].h_vec[2];
      pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+7] = 1.0;

      pv_model.meas[2*i+1] = -kf_meas.meas[i].doppler 
	- pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+3]*pv_model.x[3]
	- pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+4]*pv_model.x[4]
	- pv_model.hm[(2*i+1)*PV_MDL_STATE_NUM+5]*pv_model.x[5]
	- pv_model.x[7];

#ifdef xGPS_KALMAN_DBG
  pv_debughdle<<kf_meas.meas[i].prn <<" "<< pv_model.meas[2*i] << " " <<pv_model.meas[2*i+1] <<" " ;

#endif
    }

  line_count=0;
  kf_matrix.matrix_identity(pv_model.Rm,
			    pv_model.k_dim,
			    pv_model.k_dim);
  for( i=0; i<kf_meas.sv_count; i++)
    {
      pv_model.Rm[2*i*pv_model.k_dim + line_count] = 9.0;     // std: 3m
      line_count++;
      pv_model.Rm[(2*i+1)*pv_model.k_dim + line_count] = 0.25;    // std: 0.5m/s
      line_count++;
    }
#ifdef xGPS_KALMAN_DBG
  pv_debughdle<<endl ;
#endif
}
void Kalman::PVmodel_log_kf_data()
{
  int i;
  //pv_debughdle<<"PV_model: " ;

  dbg_str<<setw(12)<<setprecision(4)<<setiosflags(ios::fixed|ios::showpoint);
  // log state first
  for( i=0; i<PV_MDL_STATE_NUM; i++)
    dbg_str<< pv_model.x[i] << " " ;
  // log correction
  for( i=0; i<PV_MDL_STATE_NUM; i++)
    dbg_str<< pv_model.dx[i] << " " ;
  // log p_matrix
  for( i=0; i<PV_MDL_STATE_NUM; i++)
    dbg_str<< pv_model.p[i*PV_MDL_STATE_NUM + i] << " " ;
  dbg_str <<endl;

#ifdef GPS_KALMAN_DBG
  pv_debughdle<<dbg_str.str();

#endif

  log_msg("DATA KF 1 " + dbg_str.str());
  dbg_str.str("");

}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
// The following functions are wrapper function for all models
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void Kalman::update_phi_q(double dt)
{
  Pmodel_update_phi_q(dt);
  PVmodel_update_phi_q(dt);
}
void Kalman::propagate_state(double dt)
{
	Pmodel_propagate_state(dt);
	PVmodel_propagate_state(dt);
}
void Kalman::update_state(void)
{
  Pmodel_update_state();
  PVmodel_update_state();
}
void Kalman::calc_correction(void)
{
  Pmodel_calc_correction();
  PVmodel_calc_correction();
}

void Kalman::update_p_(void)
{
  Pmodel_update_p_();
  PVmodel_update_p_();
}

void Kalman::set_H_R(const kf_group_meas &kf_meas)
{
  Pmodel_set_H_R(kf_meas);
  PVmodel_set_H_R(kf_meas);
}
void Kalman::log_kf_data(void)
{
  Pmodel_log_kf_data();
  PVmodel_log_kf_data();
}

void Kalman::nav_kf_fix(const kf_group_meas &kf_meas )
{
  double delta_t;
  delta_t = kf_meas.local_t - local_time;
  local_time = kf_meas.local_t;

  propagate_state(delta_t);
  set_H_R( kf_meas );
  update_phi_q(delta_t);
  update_p_();
  calc_correction();
  update_state();
  log_kf_data();
}
