/*:pos_math.cc
********************************************************
 * GPS-related mathematic libray function calls
 * 
 *
 * Author:
 *        Yu Lu, softwareGNSS@gmail.com
 *        Jan, 2006
 *******************************************************/
#include "./includes/pos_math.h"
#include <iostream>
#include <fstream>
#include <math.h>

const double GPS_pos_math::SemiMajorAxis=6378137.0L;
const double GPS_pos_math::SemiMinorAxis=6356752.3142L; 
const double GPS_pos_math::EarthEcc     = 0.0818191908426L;
const double GPS_pos_math::EccentrSquared=0.00669437999013L;
const double GPS_pos_math::OneMinusEccentrSquared=0.993305620010000L;
const double GPS_pos_math::OmegaDotEarth=7.292115146E-5L;
const double GPS_pos_math::SpeedOfLight = 2.99792458e8;

const double GPS_pos_math::PI = 3.1415926;
const double GPS_pos_math::PI2DEG = 57.295779;
const double GPS_pos_math::lambda = .1902936728;

const int GPS_pos_math::SecPerHour=3600;
const int GPS_pos_math::SecPerDay=86400;
const int GPS_pos_math::SecPerWeek=604800;


GPS_pos_math::GPS_pos_math()
{
  
  // open a file handle for debugging
#ifdef POS_MATH_DBG
  debughdle.open("./data/pos_math_dbg.m");
  if( !debughdle.is_open() )
    cerr <<"Failed to open debug file for GPS pos_math" << endl;
#endif
}

GPS_pos_math::~GPS_pos_math()
{
#ifdef POS_MATH_DBG
  if( debughdle.is_open() )
    debughdle.close();
#endif
}

/****************************************************************************
* Function: cartstruc sub_vector(cartstruc *a, cartstruc *b)
*
* Calculates the vector a - b.
*
* Input: a - the vector a.
*        b - the vector b.
*
* Output: None.
*
* Return Value: The vector a - b.
****************************************************************************/
cartstruc GPS_pos_math::sub_vector(const cartstruc &a, const cartstruc &b)
{
    cartstruc c;

    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;

    return(c);
}

/****************************************************************************
* Function: double vec_length(double x, double y, double z)
*
* Calculates the length of the vector (x,y,z). Note the this can be used for
* 2-D and 3-D vectors.
*
* Input: x - the x cartesian co-ordinate.
*        y - the y cartesian co-ordinate.
*        z - the z cartesian co-ordinate.
*
* Output: None.
*
* Return Value: The vector length.
****************************************************************************/
double GPS_pos_math::vec_length(double x, double y, double z)
{
    return(sqrt(x*x + y*y + z*z));
}


/****************************************************************************
* Function: void llh2ecef(llhstruc *llh, cartstruc *xyz)
*
* Converts from WGS-84 latitude, longitude and height and X, Y and Z
* rectangular co-ordinates. For more information: Simo H. Laurila,
* "Electronic Surveying and Navigation", John Wiley & Sons (1976).
*
* Input: llh - the (lat,lon,hgt) vector.
*
* Output: xyz - (x,y,z) vector.
*         
* Return Value: None.
****************************************************************************/
void GPS_pos_math::llh2ecef(const llhstruc *llh, cartstruc *xyz)
{
    double n;         /* WGS-84 Radius of curvature in the prime vertical. */
    double ome2;                                   /* WGS-84 (1.0E0 - e2). */
    double clat;                                              /* cos(lat). */
    double slat;                                              /* sin(lat). */
    double clon;                                              /* cos(lon). */
    double slon;                                              /* sin(lon). */
    double d,nph;
    double tmp;

    ome2 = 1-EccentrSquared;//0.99330562000987;

    clat = cos(llh->lat);
    slat = sin(llh->lat);
    clon = cos(llh->lon);
    slon = sin(llh->lon);
    d = EarthEcc*slat;

    n = SemiMajorAxis/sqrt(1.0E0-d*d);
    nph = n + llh->hgt;

    tmp = nph*clat;
    xyz->x = tmp*clon;
    xyz->y = tmp*slon;
    xyz->z = (ome2*n + llh->hgt)*slat;
}

/****************************************************************************
* Function: void ecef2llh(cartstruc *xyz, llhstruc *llh)
*
* Converts from WGS-84 X, Y and Z rectangular co-ordinates to latitude,
* longitude and height. For more information: Simo H. Laurila,
* "Electronic Surveying and Navigation", John Wiley & Sons (1976).
*
* Input: xyz - (x,y,z) vector.
*
* Output: llh - the (lat,lon,hgt) vector.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::ecef2llh(const cartstruc *xyz, llhstruc *llh)
{
  double eps;                                      /* Convergence limit. */
    double dpi2;                                     /* Convergence limit. */
    double n,d,nph,rho,latold,hgtold;

    eps = 1.0E-13;
    dpi2 = 1.570796326794897E0;

    
    /* If the position is on the Z axis, the computation must be handled as
       a special case to keep it from blowing up. */

    rho = vec_length(xyz->x,xyz->y,0.0);
    if (rho <= eps)
    {
        llh->lat = dpi2;                 /* Come here if we are on the Z axis. */
        if(xyz->z<0.0)
            llh->lat = -llh->lat;
        llh->lon = 0.0E0;
        llh->hgt = fabs(xyz->z) - SemiMinorAxis;
        return;
    }

    /* Come here in the typical case.  Since latitude and spheroid height
       depend on one another, the solution must be done iteratively. */

    llh->lat = atan2(xyz->z,rho);
    llh->lon = atan2(xyz->y,xyz->x);
    llh->hgt = rho/cos(llh->lat);

    latold = llh->lat + 1.0E0;
    hgtold = llh->hgt + 1.0E0;
    while(fabs(llh->lat - latold)>=eps || fabs(llh->hgt-hgtold)>=0.01)
    {
        /* Require latitude to converge to about the precision of the
           machine, and height to the precision of about a centimeter. */

        latold = llh->lat;
        hgtold = llh->hgt;
        d = EarthEcc*sin(latold);
        n = SemiMajorAxis/sqrt(1.0E0 - d*d);
        llh->hgt = rho/cos(latold) - n;
        nph = n + llh->hgt;
        d = 1.0E0 - EccentrSquared*(n/nph);
        llh->lat = atan2(xyz->z,rho*d);
    }
}

/****************************************************************************
* Function: void cart2sph(cartstruc *xyz, sphstruc *eah)
*
*
* Convert cartesian (x,y,z) to spherical (el,az,hgt) co-ordinates.
*
* Input: xyz - the (x,y,z) cartesian co-ordinates.
*
* Output: eah - the (el,az,hgt) spherical co-ordinates.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::cart2sph(const cartstruc *xyz, sphstruc *eah)
{
    eah->el = atan2(xyz->z,vec_length(xyz->x,xyz->y,0.0));
    eah->az = atan2(xyz->y,xyz->x);
    eah->hgt = vec_length(xyz->x,xyz->y,xyz->z);
}

/****************************************************************************
* Function: void sph2cart(sphstruc *eah, cartstruc *xyz)
*
* Convert spherical (el,az,hgt) to cartesian (x,y,z) co-ordinates.
*
* Input: eah - the (el,az,hgt) spherical co-ordinates.
*
* Output: xyz - the (x,y,z) cartesian co-ordinates.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::sph2cart(const sphstruc *eah, cartstruc *xyz)
{
    double tmp;

    tmp = eah->hgt*cos(eah->el);
    xyz->x = tmp*cos(eah->az);
    xyz->y = tmp*sin(eah->az);
    xyz->z = eah->hgt*sin(eah->el);
}



/****************************************************************************
* Function: void cartV2sphV(sphstruc *eah, cartstruc *velxyz, sphstruc *veleah)
*
* Convert cartesian (x,y,z) velocities to spherical (el,az,hgt) velocities.
*
* Input: eah - the (el,az,hgt) spherical co-ordinates.
*        velxyz - the (x,y,z) cartesian velocity co-ordinates.
*
* Output: veleah - the (el,az,hgt) spherical velocity co-ordinates.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::cartV2sphV(const sphstruc *eah, 
			      const cartstruc *velxyz,
			      sphstruc *veleah)
{
    double sinel, cosel;                           /* sin(el) and cos(el). */
    double sinaz, cosaz;                           /* sin(az) and cos(az). */

    sinel = sin(eah->el);
    cosel = cos(eah->el);
    sinaz = sin(eah->az);
    cosaz = cos(eah->az);

    veleah->el = (velxyz->z*cosel
                  - sinel*(velxyz->y*sinaz + velxyz->x*cosaz))/eah->hgt;
    veleah->az = (velxyz->y*cosaz - velxyz->x*sinaz)/(eah->hgt*cosel);
    veleah->hgt = velxyz->z*sinel
                  + cosel*(velxyz->y*sinaz + velxyz->x*cosaz);
}

/****************************************************************************
* Function: void sphV2cartV(sphstruc *eah, sphstruc *veleah,
*                                cartstruc *velxyz)
*
* Convert spherical (el,az,hgt) velocities to cartesian (x,y,z) velocities.
*
* Input: eah - the (el,az,hgt) spherical co-ordinates.
*        veleah - the (el,az,hgt) spherical velocity co-ordinates.
*
* Output: velxyz - the (x,y,z) cartesian velocity co-ordinates.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::sphV2cartV(const sphstruc *eah, 
			      const sphstruc *veleah, 
			      cartstruc *velxyz)
{
    double sinel, cosel;                           /* sin(el) and cos(el). */
    double sinaz, cosaz;                           /* sin(az) and cos(az). */

    sinel = sin(eah->el);
    cosel = cos(eah->el);
    sinaz = sin(eah->az);
    cosaz = cos(eah->az);

    velxyz->x = cosel*(veleah->hgt*cosaz - eah->hgt*veleah->az*sinaz)
                - eah->hgt*veleah->el*cosaz*sinel;
    velxyz->y = cosel*(veleah->hgt*sinaz + eah->hgt*veleah->az*cosaz)
                - eah->hgt*veleah->el*sinaz*sinel;
    velxyz->z = veleah->hgt*sinel + eah->hgt*veleah->el*cosel;
}


/****************************************************************************
* Function: void ecef_topocentric_matrix(cartstruc *xyz, double t[3][3])
*
* Determines a transformation matrix which rotates vectors in the
* ECEF xyz co-ordinate system to the topocentric (observer centered)
* North-East-Up system.
*
* Input: xyz - the cartesian ECEF co-ordinates.
*
* Output: t - the transform matrix.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::ecef_topocentric_matrix(const cartstruc *xyz, double t[3][3])
{
    sphstruc eah;                      /* Observer spherical co-ordinates. */

    double sel, cel;                                  /* sin(el), cos(el). */
    double saz, caz;                                  /* sin(az), cos(az). */

    cart2sph(xyz,&eah);

    sel = sin(eah.el);
    cel = cos(eah.el);
    saz = sin(eah.az);
    caz = cos(eah.az);

    t[0][0] = -sel*caz;
    t[0][1] = -sel*saz;
    t[0][2] = cel;
    t[1][0] = -saz;
    t[1][1] = caz;
    t[1][2] = 0.0;
    t[2][0] = cel*caz;
    t[2][1] = cel*saz;
    t[2][2] = sel;
}

/****************************************************************************
* Function: void ecef2topo(cartstruc *ecef, double t[3][3], 
*                           carttruc *topo)
*
* Transform ECEF vector to topocentric co-ordinates using the supplied transform
* matrix.
*
* Input: ecef - the ECEF co-ordinates.
*        t - the transform matrix.
*
* Output: topo - the topocentric co-ordinates.
*
* Return Value: None.
****************************************************************************/
void GPS_pos_math::ecef2topo(const cartstruc *ecef, 
			     const double t[3][3], 
			     cartstruc *topo)
{
    topo->x = t[0][0]*ecef->x + t[0][1]*ecef->y + t[0][2]*ecef->z;
    topo->y = t[1][0]*ecef->x + t[1][1]*ecef->y + t[1][2]*ecef->z;
    topo->z = t[2][0]*ecef->x + t[2][1]*ecef->y + t[2][2]*ecef->z;
}

/****************************************************************************
* Function: void calc_elev_azim(cartstruc *user_pos, cartstruc *sv_pos, 
*             double *elev, double *azim )
*
* Calculate SV's elevation and azimuth from observer's position
* matrix.
*
* Input: user_pos - the user's position in ECEF co-ordinates.
*        sv_pos   - the SV's position in ECEF co-ordinate.
*
* Output: elev - SV's elevation 
*         azim - SV's azimuth
*
* Return Value: None.
****************************************************************************/

void GPS_pos_math::calc_elev_azim(const cartstruc &user_pos, 
		    const cartstruc &sv_pos, 
		    double *elev, double *azim)
{
  cartstruc xyz, t_xyz;
  double t[3][3], xy_length;
  xyz = sub_vector(sv_pos, user_pos);
  ecef_topocentric_matrix(&user_pos, t);
  ecef2topo(&xyz, t, &t_xyz);
  xy_length = vec_length(t_xyz.x,t_xyz.y, 0);
  *elev = atan2(t_xyz.z,xy_length);
  *azim = atan2(t_xyz.y, t_xyz.x);
  if(*azim<0.0)
    *azim = *azim + 2.0*PI;
}

