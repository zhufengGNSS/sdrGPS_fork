/*:gps_nav_msg.cc
*******************************************************
* GPS navigation message handling functions
* 
* This class is used to handle the nav message from the tracking loop
* including message parity check, decoding ephemeris data, etc
*
* Author:
*        Yu Lu, softwareGNSS@gmail.com
*        Jan, 2005
*******************************************************/

#include "./includes/gps_nav_msg.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <assert.h>
#include <string.h>

const double GPS_msg_nav::SemiMajorAxis=6378137.0L;
const double GPS_msg_nav::EccentrSquared=0.00669437999L;
const double GPS_msg_nav::OneMinusEccentrSquared=0.993305620010000L;
const double GPS_msg_nav::OmegaDotEarth=7.292115146E-5L;
const double GPS_msg_nav::SpeedOfLight = 2.99792458e8;

const double GPS_msg_nav::c_2p12 = 4096;
const double GPS_msg_nav::c_2p4  = 16;
const double GPS_msg_nav::c_2m5  = 0.03125;
const double GPS_msg_nav::c_2m11 = 4.8828125e-4;
const double GPS_msg_nav::c_2m19 = 1.9073486328125e-6;
const double GPS_msg_nav::c_2m20 = 9.5367431640625e-7;
const double GPS_msg_nav::c_2m21 = 4.76837158203125e-7;
const double GPS_msg_nav::c_2m23 = 1.19209289550781e-7;
const double GPS_msg_nav::c_2m24 = 5.96046447753906e-8;
const double GPS_msg_nav::c_2m27 = 7.45058059692383e-9;
const double GPS_msg_nav::c_2m29 = 1.86264514923096e-9;
const double GPS_msg_nav::c_2m30 = 9.31322574615479e-10;
const double GPS_msg_nav::c_2m31 = 4.65661287307739e-10;
const double GPS_msg_nav::c_2m33 = 1.16415321826935E-10;
const double GPS_msg_nav::c_2m38 = 3.63797880709171e-12;
const double GPS_msg_nav::c_2m43 = 1.13686837721616e-13;
const double GPS_msg_nav::c_2m50 = 8.881784197e-16;
const double GPS_msg_nav::c_2m55 = 2.77555756156289e-17;
const double GPS_msg_nav::PI = 3.1415926;
const double GPS_msg_nav::lambda = .1902936728;

const int GPS_msg_nav::pb1=0x3b1f3480;
const int GPS_msg_nav::pb2=0x1d8f9a40;
const int GPS_msg_nav::pb3=0x2ec7cd00;
const int GPS_msg_nav::pb4=0x1763e680;
const int GPS_msg_nav::pb5=0x2bb1f340;
const int GPS_msg_nav::pb6=0x0b7a89c0;

const int GPS_msg_nav::SecPerHour=3600;
const int GPS_msg_nav::SecPerDay=86400;
const int GPS_msg_nav::SecPerWeek=604800;

GPS_msg_nav::GPS_msg_nav()
{

  memset((void*) nav_msg, 0, sizeof(nav_msg) );
  prev_bit[0] = prev_bit[1] = 0;
  memset((void*)p_error, 0 , 12*sizeof(unsigned int) );
  memset((void*)navmsgparity, 0, 12*sizeof(navmsg_par));
  memset((void*)Ephemeris , 0, (SatMax+1)*sizeof(EphInfo));

  for( int i=0; i<12; i++)
    for( int j=0; j<5; j++)
      navmsgparity[i].par[j] = 1;
  eph_set_flag = false;

  load_eph_bin();
  
#ifdef GPS_MSGNAV_DBG
  debughdle.open("./data/gps_navmsg_log.m");
  if( !debughdle.is_open() )
    cerr <<"Failed to open debug file for GPS msgnav class" << endl;
#endif
}

GPS_msg_nav::~GPS_msg_nav()
{
  if( eph_set_flag )
    save_eph_bin();
  
#ifdef GPS_MSGNAV_DBG
  if( debughdle.is_open() )
    debughdle.close();
#endif
}

void GPS_msg_nav::set_prnlist(int ch_idx, unsigned int prn)
{
  ch_prn[ch_idx] = prn;
}

// Copy one subframe msg bit into the nav_msg word buffer (30 bit leng)
//
// unsigned int * src: the pointer to the source msg bit
// idx: the index of current position for the src buffer
// ch_idx: which channel is this called for

void GPS_msg_nav::copy_navmsg(const unsigned int* src, int idx, int ch_idx)
{
  int cur_idx;
  cur_idx = (idx + 1)%302;
  
  // pre_bit[0-1] are used to check paritys
  prev_bit[0] = (src[ cur_idx ])>>29;
  prev_bit[1] = (src[ (++cur_idx)%302 ])>>29;
  
  // re-organize the message bits to form message words
  for( int wi = 0; wi < 10; wi++ )
    {
      nav_msg[ch_idx][wi] = 0;
      for( int j=0; j<30; j++)
	{
	  if( src[(++cur_idx)%302] & 0x20000000 )
	    nav_msg[ch_idx][wi] = nav_msg[ch_idx][wi]*2 + 0x1;
	  else
	    nav_msg[ch_idx][wi] = nav_msg[ch_idx][wi]*2 ;
	}
    }
  
}

//check parity for one gps word
// for details algorithm, refer to ICD_GPS_200c
// Note:
// To successfully test the parity bit, the gpsword must be formatted
// as below:
//     bit31-bit30: the previous two bits;
//     bit29-bit0 : the 30 bit for this GPS word;
int GPS_msg_nav::parity_check_word( unsigned int gpsword)
{
  unsigned int d1, d2, d3, d4, d5, d6, d7, t, parity;

  d1 = gpsword & 0xFBFFBF00;
  d2 = rotl(gpsword,1) & 0x07FFBF01;
  d3 = rotl(gpsword,2) & 0xFC0F8100;
  d4 = rotl(gpsword,3) & 0xF81FFE02;
  d5 = rotl(gpsword,4) & 0xFC00000E;
  d6 = rotl(gpsword,5) & 0x07F00001;
  d7 = rotl(gpsword,6) & 0x00003000;

  t  = d1^d2^d3^d4^d5^d6^d7;

  parity = t^rotl(t,6)^rotl(t,12)^rotl(t,18)^rotl(t,24);
  parity = parity&0x3F;

  if(parity == (gpsword&0x3F))
    return 0;

  else
    return 1;
}

// check parity for one subframe
// for details algorithm, refer to ICD_GPS_200c

void GPS_msg_nav::parity_check_channel(int ch_idx)
{
  int  m_parity;
  unsigned tmp_word;

  p_error[ch_idx] = 0;
    
  for( int i=0; i<10; i++)
    {
      m_parity = nav_msg[ch_idx][i]&0x3F;
        
      if( prev_bit[1] )
	nav_msg[ch_idx][i] = nav_msg[ch_idx][i]^0x3fffffc0;
        
      tmp_word = ((unsigned int)nav_msg[ch_idx][i])|(prev_bit[0]<<31)|(prev_bit[1]<<30);

      p_error[ch_idx] += parity_check_word(tmp_word)<<i;
      /*
	if(parity_check_word(tmp_word))
	p_error[ch_idx] *= 2 ;
	else
	p_error[ch_idx]  = p_error[ch_idx]*2 +1;
      */      
      nav_msg[ch_idx][i] = nav_msg[ch_idx][i]>>6;
      prev_bit[0]=(m_parity & 0x2) >>1;
      prev_bit[1]= m_parity & 0x1;
    }
    
}

// return the frame count: 1-5
// int p_err: the error flag of the previous parity_check

int GPS_msg_nav::read_frame_num(int p_err, int ch_idx)
{

  //  cout<<"p_err : " <<p_err<< " ch_idx: " << ch_idx <<endl;
  
  if( p_err != 0 ){
#ifdef GPS_LOG_CONSOLE
    cout<<"CH " << ch_idx <<" Parity check failed " << endl;
#else
    dbg_str<<"MSG CH " << ch_idx <<" Parity check failed " << endl;
    log_msg(dbg_str.str());
    dbg_str.str("");
#endif    
    return -1;  // parity check failed, return a invalid value
  }
  
  int frmidx = nav_msg[ch_idx][1];
  frmidx = (frmidx>>2) & 0x07;
  if( frmidx >=1 && frmidx<=5 )
    {
      navmsgparity[ch_idx].par[frmidx-1] = 0;
      memcpy((void*)&navmsgparity[ch_idx].msg[frmidx-1][0],
	     (void*)&nav_msg[ch_idx][0],10*sizeof(unsigned int));
      
#ifdef GPS_LOG_CONSOLE
      cout <<endl<<"CH " << ch_idx << ", frame # " << frmidx <<endl;
#else
      dbg_str<<"MSG" <<endl<<"CH " << ch_idx << ", frame # " << frmidx <<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif

    }
  
  return frmidx;
  
}

void GPS_msg_nav::read_allsubframe(int ch_idx)
{
  int iodc,iode,idoe, iweek, iura, ihealth, iaf0, im0, inc0, iomega0,iw;
  int iomegadot, idot, prn;
  char itgd,iaf2;
  unsigned int itoc, ie,isqra, itoe;
  short int iaf1,icrs, idn, icuc,icus,icic,icis,icrc;
  unsigned int *sf;
  double r_sqra,r_inc0,r_ety;

  sf = (unsigned int*)&navmsgparity[ch_idx].msg[0][0];
  prn = ch_prn[ch_idx];

  if( navmsgparity[ch_idx].par[0] == 0 &&
      navmsgparity[ch_idx].par[1] == 0 &&
      navmsgparity[ch_idx].par[2] == 0) // if frm1-3 is correct
    {
      iodc=int(((sf[0*10+2] & 0x3) <<8 ) | ((sf[0*10+7] & 0xFF0000) >>16));
      iode=int(sf[1*10+2]  >> 16);
      idoe=int(sf[2*10+9] >> 16);

      if( (iode == idoe) &&
	  ( (iode!= Ephemeris[ prn ].iode) || (iodc!=Ephemeris[ prn ].iodc) ))
	{

	  // subframe 1

	  iweek= int(sf[0*10+2] >> 14);
	  iura=int(( sf[0*10+2] & 0xF00 ) >> 8);
	  ihealth=int(( sf[0*10+2] & 0xFC ) >> 2);
	  itgd=sf[0*10+6] & 0xFF;
	  itoc=sf[0*10+7] & 0xFFFF;
	  iaf2=sf[0*10+8] >> 16;
	  iaf1=sf[0*10+8] & 0xFFFF;
	  iaf0=sf[0*10+9] >> 2;
	  if( iaf0 & 0x200000) iaf0=iaf0 | 0xFFC00000; // sign extension

	  //   subframe 2
	  icrs=sf[1*10+2] & 0xFFFF;
	  idn =(sf[1*10+3]>>8) & 0xFFFF;
	  im0=((sf[1*10+3] & 0xFF) << 24) | sf[1*10+4];
	  icuc=(sf[1*10+5]>>8) & 0xFFFF;
	  ie=((sf[1*10+5] & 0xFF) << 24) | sf[1*10+6];
	  icus=(sf[1*10+7]>>8) & 0xFFFF;
	  isqra=(((sf[1*10+7] & 0xFF) << 24) | sf[1*10+8]);
	  itoe=int(sf[1*10+9] >> 8);

	  // subframe 3
	  icic=(sf[2*10+2]>>8) & 0xFFFF;
	  icis=(sf[2*10+4]>>8) & 0xFFFF;
	  inc0=((sf[2*10+4] & 0xFF) << 24) | sf[2*10+5];
	  iomega0=((sf[2*10+2] & 0xFF) << 24) | sf[2*10+3];
	  icrc=(sf[2*10+6]>>8) & 0xFFFF;
	  iw=((sf[2*10+6] & 0xFF) << 24) | sf[2*10+7];
	  iomegadot=sf[2*10+8];
	  if (iomegadot&0x800000)
	    iomegadot=iomegadot | 0xFF000000;  // sign extension
	  idot=((sf[2*10+9] & 0xFFFC) >> 2);
	  if (idot&0x2000)
	    idot=idot | 0xFFFFC000; // sign extension

	  r_sqra=isqra*c_2m19;
	  r_inc0=inc0*c_2m31*PI;
	  r_ety=ie*c_2m33;

	  if (   (r_inc0<1.05 && r_inc0>0.873)&&
		 (r_sqra>5100.0 && r_sqra<5200.0)&&
		 (r_ety <.05 && r_ety>0.0)  )
	    {
	      Ephemeris[prn].valid=1;
	      Ephemeris[prn].prn = prn;
	      Ephemeris[prn].iode=iode;
	      Ephemeris[prn].iodc=iodc;
	      Ephemeris[prn].week=iweek;
	      Ephemeris[prn].ura=iura;
	      Ephemeris[prn].health=ihealth;
	      Ephemeris[prn].tgd=itgd*c_2m31;
	      Ephemeris[prn].toc=itoc*16.0;
	      Ephemeris[prn].af2=iaf2*c_2m55;
	      Ephemeris[prn].af1=iaf1*c_2m43;
	      Ephemeris[prn].af0=iaf0*c_2m31;
	      Ephemeris[prn].crs=icrs*c_2m5;
	      Ephemeris[prn].dn=idn*c_2m43*PI;
	      Ephemeris[prn].ma=im0*c_2m31*PI;
	      Ephemeris[prn].cuc=icuc*c_2m29;
	      Ephemeris[prn].ety=r_ety;
	      Ephemeris[prn].cus=icus*c_2m29;
	      Ephemeris[prn].sqra=r_sqra;
	      Ephemeris[prn].wm=19964981.84/pow(r_sqra,3);
	      Ephemeris[prn].toe=itoe*c_2p4;
	      Ephemeris[prn].cic=icic*c_2m29;
	      Ephemeris[prn].cis=icis*c_2m29;
	      Ephemeris[prn].inc0=r_inc0;
	      Ephemeris[prn].w0=iomega0*c_2m31*PI;
	      Ephemeris[prn].crc=icrc*c_2m5;
	      Ephemeris[prn].w=iw*c_2m31*PI;
	      Ephemeris[prn].omegadot=iomegadot*c_2m43*PI;
	      Ephemeris[prn].idot=idot*c_2m43*PI;
	      eph_set_flag = true;       // new ephemeris data is set

#ifdef GPS_LOG_CONSOLE
	      cout <<endl<<"set Eph data for PRN " <<ch_prn[ch_idx] <<endl;
#else
	      dbg_str<<"MSG" << endl<<"set Eph data for PRN " <<ch_prn[ch_idx] <<endl;
	      log_msg(dbg_str.str());
	      dbg_str.str("");
#endif

	    } // end of  if (   (r_inc0<1.05 && r_inc0>0.873)&&
	} // end of if ( (iode == idoe) &&

    } // end of if( navmsgparity[idx].par[0] == 0 &&
  // finally init parity to invalid values back
  for( int j=0; j<5; j++)
    {
      navmsgparity[ch_idx].par[j] = 0x1;  // init parity values for 5 frm of all channels
      memset((void*)&navmsgparity[ch_idx].msg[j][0],0,10*sizeof(unsigned int));
    }
}


// unsigned int * src: the pointer to the source msg bit
// idx: the index of current position for the src buffer
// ch_idx: which channel is this called for
void GPS_msg_nav::process_navmsg(const unsigned int *src, int idx, int ch_idx)
{
  
  copy_navmsg( src, idx, ch_idx );
  parity_check_channel( ch_idx );
  
  if( read_frame_num( p_error[ch_idx], ch_idx )>=3 )
    read_allsubframe(ch_idx);
}


// save the ephemeris data for future use
// File format: binary
// First save the number of Ephemeris, then the Ephemeris data blocks
void GPS_msg_nav::save_eph_bin(void)
{
  ofstream  ephBinHandle;
  int i, satnum;
  int ephCount=0;
    
  ephBinHandle.open("./data/hexeph.bin", ios::binary);
  if( !ephBinHandle.is_open() )
    {
      cerr << "Cannot open eph binary data file." <<endl;
      return ;
    }
  
  satnum = 0;
  for( i=0; i<SatMax+1; i++)
    {
      if(Ephemeris[i].valid == 1)
	satnum++;
    }
  // first write num of ephs
  // then several blocks containing all the ephemeris blocks
  ephBinHandle.write((char*)&satnum, sizeof(int)); 
  for( i=0; i<SatMax+1; i++){
    
    if( Ephemeris[i].valid == 1)
      {
	ephCount++;
	ephBinHandle.write((char*)&Ephemeris[i], sizeof(EphInfo) );
      }
    
  }
#ifdef GPS_LOG_CONSOLE
  cout << "Save "<<ephCount<< "ephemeris data "<<endl;
#endif

  
  ephBinHandle.close();
  
}

void GPS_msg_nav::load_eph_bin(void)
{
  ifstream ephBinHdle;
  ephBinHdle.open("./data/hexeph.bin", ios::binary);
  if( !ephBinHdle.is_open())
    {
#ifdef GPS_LOG_CONSOLE
      cout <<"No valid Ephemeris data file."<<endl;
#else
      dbg_str<<"MSG No valid Ephemeris data file."<<endl;
      log_msg(dbg_str.str());
      dbg_str.str("");
#endif

      return;
    }
  
  EphInfo  tmpeph;
  int satnum, p;
  ephBinHdle.read((char*)&satnum, sizeof(int));
  
  for( int i=0; i<satnum; i++)
    {
      ephBinHdle.read((char*)&tmpeph, sizeof(EphInfo));
      p=tmpeph.prn;
      memcpy((void*)&Ephemeris[p], (void*)&tmpeph, sizeof(EphInfo));
    }
  
#ifdef GPS_LOG_CONSOLE
  cout <<"There are " <<satnum<<" satellites' ephmeris data loaded from file."<<endl;
#else
  dbg_str<<"MSG There are " << satnum << " satellites' eph data loaded from file."<<endl;
  log_msg(dbg_str.str());
  dbg_str.str("");
#endif

  ephBinHdle.close();
  
}

// return true if the ephemeris data for this SV is available
bool GPS_msg_nav::eph_valid( int prn)
{
  if( Ephemeris[prn].valid )
    return true;
  else
    return false;
}

// get the pointer of the ephemeris data for this SV
EphInfo* GPS_msg_nav::get_eph_pt( int prn )
{
  assert(prn <SatMax+1 );
  return (&Ephemeris[prn]);
}
