/*
 * gps_handle.h
 *
 *  Created on: 4-Nov-2009
 *      Author: c25wang
 *
 *      NOTE*** Module primarily focused on Garmin 18x5Hz GPS,
 *      assumption is that its purely outputing GPRMC data (12 in total, including 'GPRMC',
 *      without mode indicator)
 */

#ifndef GPS_HANDLE_H_
#define GPS_HANDLE_H_

//standard libs
#include <string.h>
#include <iostream>
using namespace std;
#include <SerialPort.h>
#include "ros/ros.h"
#include <vector>
#include <algorithm>
//#include <pthread.h>	// Posix multi-threading
#include <time.h>
#include <math.h>

/**
 * GPS RMC Data structure
 */
typedef struct gps_rmc_t {
	unsigned char UTC_hh;	//UTC hour
	unsigned char UTC_mm;	//UTC minute
	double UTC_ss;			//UTC seconds
	char status;			//A = Valid Pos, V = NAV reciever warning
	double lat;				//latitude (degrees)
	char lat_hemi;			//latitude hemisphere (N or S)
	double lon;				//longitude (degrees)
	char lon_hemi;			//longitude hemisphere (E or W)
	double speed;			//speed (knots)
	double heading; 			//heading (degrees from true)
	unsigned char UTC_date_dd;	//UTC day
	unsigned char UTC_date_mm;	//UTC month
	unsigned char UTC_date_yy;	//UTC year
	float mag_var;			//magnetic variation (degrees)
	unsigned char mag_var_dir;	//magnetic variation direction (westerly variation adds to true course)
	char mode;					//Mode indicator (only output if NMEA 0183 version 2.30 active), A = Autonomous,
								//D = Differential, E = Estimated, N = Data not valid
} GPRMC_t;

#define DELIMITER		0x2C	//','
#define	NEW_SENTENCE	0x24	//'$'
#define END_SENTENCE	0x2A	//'*'
#define NMEA_GPRMC		"GPRMC"

class GPSH{

public:

	GPRMC_t rmc;				//rmc data
	string gpsPortName;			//GPS port name
	GPSH(string _portname);		//constructor (requires portname of GPS)
	void close();				//close gps handler class

	//high level commands
	int update();

	//misc commands
	void destinationinfo(double lat1, double lon1, double lat2, double lon2,
						 double &dist, double &bearing, double &rel_x, double &rel_y);

private:

	SerialPort port;				//serial port
	vector<unsigned char> raw_rx;	//raw rx

	//low level commands
	int parseSentence(string s);
	int parseRMC(vector<string> &data);

	//deserialization commands
	int _getLastSentence(string &s);						//returns last sentence from raw_rx
	int _splitSentence(string s, vector<string> &data);		//returns comma delimited strings from sentence
	double _read_double(string s);
	int _read_int(string s);

	//comm. commands (uses raw_rx/raw_tx)
	int comm_read();

	//calc commands
	inline double degtorad(double deg){ return ((deg * M_PI) / 180.0); }
	inline double radtodeg(double rad){ return ((rad * 180.0) / M_PI); }

};

#endif /* GPS_HANDLE_H_ */
