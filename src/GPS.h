#include <Math.h>
#include "Arduino.h"
#include <String.h>
#include <Wire.h>

#ifndef GPS_h
#define GPS_h

#define EARTH_RADIUS 6378.137

class GPS
{
public:

	void begin();

  void getRawData();

	void getOrigInfo();

	double getGpsTime();

	double getLatitude();

	double getLongitude();

	double getOriginDistance();

	double getAzimuth();

	double getSpeed();

private:
	String data = "";
	int mark = 0;
	bool Mark_Start = false;
	bool valid = false;
	String PositionValid, RMCUTCtime, RMClatitude, RMClongitude;
	String Speed, Direction, Date, Declination, Mode;
	double numLatitude, numLongitude, numUTCtime, numSpeed;
  double curLatitude, curLongitude;
	double origLatitude, origLongitude, origTime;
	
	String reader();
};

#endif
