#include "wavelab_gps/gps_thread.h"


double clat=0, clon=0, cgps_speed=0, cgps_heading=0;
char clat_hemi='E',clon_hemi='S',state='V';

int main(int argc, char **argv)
{

	ros::init(argc,argv,"wavelab_gps");
	ros::NodeHandle n;

	GPSH gps(GPS_PORT);
	float gps_update = 2;

	ros::Publisher gps_pub = n.advertise<wavelab_gps::gps_msg>("gps_message", 1000);
	wavelab_gps::gps_msg current_gps_data;


	ros::Rate loop_rate(40);		//20Hz update rate
	while (ros::ok())
	{
		gps_update = gps.update();
		if (gps_update==0)	//updated gps RMC data
		{
			state = gps.rmc.status;
			clat = gps.rmc.lat;
			clat_hemi = gps.rmc.lat_hemi;
			clon = gps.rmc.lon;
			clon_hemi = gps.rmc.lon_hemi;
			cgps_speed = gps.rmc.speed;
			cgps_heading = gps.rmc.heading;
		}
		current_gps_data.status = state;
		current_gps_data.lat = clat;
		current_gps_data.lon = clon;
		current_gps_data.lat_hemi = clat_hemi;
		current_gps_data.lon_hemi = clon_hemi;
		current_gps_data.speed = cgps_speed;
		current_gps_data.heading = cgps_heading;

		gps_pub.publish(current_gps_data);

		ros::spinOnce();	
		loop_rate.sleep();			//used for maintaining rate
	}
	return 0;
}

