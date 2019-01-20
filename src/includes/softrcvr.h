#ifndef SOFTRCVR_H
#define SOFTRCVR_H

#include "softgps.h"
#include <string>

class GPSController;
class MultichCorrelator;
class GPS_Acquisition_mdl;

class SoftRcvr
{
public:
    SoftRcvr(SOFTGPS_config&);
    ~SoftRcvr();
    void process_sample(int); // the main function to process the signal sample
    void check_exist_warm(int*, double);

protected:
private:
    GPSController* gps_cntler;
    MultichCorrelator* correlator;
    GPS_Acquisition_mdl *gps_acq;
};

#endif
