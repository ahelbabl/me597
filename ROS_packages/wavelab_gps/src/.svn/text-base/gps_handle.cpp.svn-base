/*
 * gps_handle.cpp
 *
 *  Created on: 4-Nov-2009
 *      Author: c25wang
 */

#include "wavelab_gps/gps_handle.h"

GPSH::GPSH(string _portname) : port(_portname), gpsPortName(_portname)
{
/*	cout << "GPS Handler Initializer Routine Execution..." << endl << flush;
	cout << "Opening GPS comm. port: " << gpsPortName << "...";*/
	//initialize GPS comm. port
	ROS_INFO("GPS: Port Opening");
	port.Open();
	port.SetBaudRate(SerialPort::BAUD_19200);
	port.SetCharSize(SerialPort::CHAR_SIZE_8);
	port.SetParity(SerialPort::PARITY_NONE);
	port.SetNumOfStopBits(SerialPort::STOP_BITS_1);

	ROS_INFO("GPS:Aligning first frame");
	//align first frame
	do { port.Read(raw_rx,1 ,2000); }
	while ( raw_rx.at(0) != NEW_SENTENCE );
	ROS_INFO("GPS:First frame aligned..sleeping");

	//wait for two seconds to start communications
	sleep(2);

//	cout << "Initializing GPS...";
	//initialize rmc template
	rmc.UTC_hh = 0;
	rmc.UTC_mm = 0;
	rmc.UTC_ss = 0;
	rmc.status = 'V';		//Assume no GPS fix initially
	rmc.lat = 0;
	rmc.lat_hemi = 'N';
	rmc.lon = 0;
	rmc.lon_hemi = 'W';
	rmc.speed = 0;
	rmc.heading = 0;
	rmc.UTC_date_dd = 0;
	rmc.UTC_date_mm = 0;
	rmc.UTC_date_yy = 0;
	rmc.mag_var = 0;
	rmc.mag_var_dir = 0;
	rmc.mode = 'N';

	//clear buffer
	raw_rx.clear();
	ROS_INFO ("GPS:Port Open..Parsing data");
}

void GPSH::close(){
/*	cout << "Terminating Message Handler..." << endl << flush;
	cout << "Closing GPS comm. port: " << gpsPortName << " ...";*/
	port.Close();
//	cout << " Done!" << endl << flush;
}

int GPSH::update(){
	string latest_sentence="";
	comm_read();	//read data buffer
	if (!_getLastSentence(latest_sentence))	//get latest sentence
	{
		return parseSentence(latest_sentence);
	}
//	std::string t( raw_rx.begin(), raw_rx.end() );
//	cout << t << ".." << endl << flush;
//	cout << "&" << latest_sentence << "&" << endl << flush;

	return 5;	//no data to be read
}

int GPSH::parseSentence(string s){
	vector<string> dat;
	dat.clear();

	_splitSentence(s, dat);		//split sentence into data delimited by comma
	return parseRMC(dat);
}

int GPSH::parseRMC(vector<string> &data){
	string stmp="";

	//small checks
	if (data.size()==0)				//we don't have data to work with
		return 1;
	if (data.at(0) != NMEA_GPRMC)	//if the data is not GPRMC...
		return 2;
	if (data.size() < 12)			//at least 12 params
		return 3;
	if (data.at(2) == "V")			//not a valid GPS fix
		return 4;

	//parse latitude info
	stmp = data.at(3);
	if (stmp.length() == 10)		//exactly 10 chars (ddmm.mmmmm)
		rmc.lat = ((double)_read_int(stmp.substr(0,2))) + (_read_double(stmp.substr(2,8))/60.0);
	stmp = data.at(4);
	if (stmp.length() == 1)
		rmc.lat_hemi = stmp[0];

	//parse longitude info
	stmp = data.at(5);
	if (stmp.length() == 11)		//exactly 11 chars (dddmm.mmmmm)
		rmc.lon = ((double)_read_int(stmp.substr(0,3))) + (_read_double(stmp.substr(3,8))/60.0);
	stmp = data.at(6);
	if (stmp.length() == 1)
		rmc.lon_hemi = stmp[0];

	//parse speed
	stmp = data.at(7);
	if (stmp.length() == 6)			//exactly 6 chars (nnn.nn)
		rmc.speed = _read_double(stmp) * 0.514444444;	//convert to m/s

	//parse heading
	stmp = data.at(8);
	if (stmp.length() == 5)			//exactly 5 chars (nnn.n)
		rmc.heading = _read_double(stmp);

	return 0;
}

int GPSH::_splitSentence(string s, vector<string> &data){
	data.clear();
	string s0="";

	for (unsigned int i=0; i < s.length(); i++)
	{
		switch (s[i])
		{
		case DELIMITER:
		{
			string sp = s0;
			data.push_back(sp);
			s0="";
			break;
		}
		default:
			s0 += s[i];
			break;
		}
	}
	data.push_back(s0);		//add last string bit

	return 0;
}

int GPSH::_getLastSentence(string &s){
	string s0 = "", s1 = "";	//confirmed, tmp sentence
	unsigned char c;
	int rxlen = raw_rx.size();	//length of rx buffer
	bool sentence_flag = true;	//true = wait for new sentence, false = continue streaming sentence until end

	for (int i=0; i < rxlen; i++)
	{
		c = raw_rx.at(i);
		if (sentence_flag)		//wait for new sentence
		{
			switch (c)
			{
			case NEW_SENTENCE:			//start new sentence
				sentence_flag = false;	//stream sentence
				break;
			default:
				break;
			}
		}
		else
		{
			switch (c)
			{
			case END_SENTENCE:			//new sentence
				s0 = s1;
				s1 = "";
				sentence_flag = true;
				break;
			default:
				s1 += c;
				break;
			}
		}
	}

	if (s0 != "")
	{
		s = s0;
		return 0;	//SUCCESS
	}
	else
	{
		return 1;	//FAILED
	}
}

double GPSH::_read_double(string s){
	double d = -1.0;

	if (s != "")
		d = atof(s.c_str());

	return d;
}

int GPSH::_read_int(string s){
	int i = -1;

	if (s != "")
		i = atoi(s.c_str());

	return i;
}

int GPSH::comm_read(){


	raw_rx.clear();

	if (port.IsDataAvailable())		//if data is avail
	{
		//flush until line feed
		unsigned char bytecheck=' ';
        	unsigned char inbyte;		
		do
		{
			inbyte = port.ReadByte(0);
			bytecheck = inbyte;
			raw_rx.push_back(inbyte);
		}while(bytecheck != 0x0A);
	}

	return 0;
}

//Calculate distance and bearing between successive waypoints
void GPSH::destinationinfo(double lat1, double lon1, double lat2, double lon2,
						   double &dist, double &bearing, double &rel_x, double &rel_y) {
	const double r = 6368.060547; // Radius of earth at Latitude of track
	double deltalat, deltalon;
	double a, c;
	double alpha;

	lat1 = degtorad(lat1);
	lat2 = degtorad(lat2);
	lon1 = degtorad(lon1);
	lon2 = degtorad(lon2);
	deltalat = lat2 - lat1;
	deltalon = lon2 - lon1;

	//Calculate distance and bearing from 1st waypoint to next waypoint
	a = sin(deltalat / 2.0) * sin(deltalat / 2.0) + cos(lat1) * cos(lat2)
			* (sin(deltalon / 2.0) * sin(deltalon / 2.0));
	c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

	dist = r * c * 1000.0;
	bearing = atan2(sin(deltalon) * cos(lat2), cos(lat1) * sin(lat2)
			- sin(lat1) * cos(lat2) * cos(deltalon));
	bearing = radtodeg(bearing);

	//Convert bearing to absolute degrees with respect to north (North = 0 degrees, East = 90 degress, etc.)
	if ((bearing < 0) && (bearing >= -180))
		bearing = 360.0 + bearing;

	//Measure relative x and y distances between waypoints
	alpha = (90 - bearing);
	rel_x = dist * cos(degtorad(alpha));
	rel_y = dist * sin(degtorad(alpha));
}

