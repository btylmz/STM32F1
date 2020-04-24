#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "GPS.h"


GPS_t hgps;

double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}


void GPS_Process(char * nmea_sentence)
{
	
	// Finds the first occurence of the '"$GPGGA,' string
	char * str = strstr(nmea_sentence,"$GPGGA,");
	if(str!=NULL)
	{
		//memset(&hgps.GPGGA,0,sizeof(hgps.GPGGA));	
		sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&hgps.GPGGA.UTC_Hour,&hgps.GPGGA.UTC_Min,&hgps.GPGGA.UTC_Sec,&hgps.GPGGA.UTC_MicroSec,&hgps.GPGGA.Latitude,&hgps.GPGGA.NS_Indicator,&hgps.GPGGA.Longitude,&hgps.GPGGA.EW_Indicator,&hgps.GPGGA.PositionFixIndicator,&hgps.GPGGA.SatellitesUsed,&hgps.GPGGA.HDOP,&hgps.GPGGA.MSL_Altitude,&hgps.GPGGA.MSL_Units,&hgps.GPGGA.AgeofDiffCorr,hgps.GPGGA.DiffRefStationID,hgps.GPGGA.CheckSum);
		
		if(hgps.GPGGA.NS_Indicator==0) hgps.GPGGA.NS_Indicator='-';
		if(hgps.GPGGA.EW_Indicator==0) hgps.GPGGA.EW_Indicator='-';
		if(hgps.GPGGA.Geoid_Units==0)  hgps.GPGGA.Geoid_Units='-';
		if(hgps.GPGGA.MSL_Units==0)    hgps.GPGGA.MSL_Units='-';
		
		hgps.GPGGA.LatitudeDecimal  = convertDegMinToDecDeg(hgps.GPGGA.Latitude);
		hgps.GPGGA.LongitudeDecimal = convertDegMinToDecDeg(hgps.GPGGA.Longitude);
	}
	memset(hgps.rxBuffer,0,sizeof(hgps.rxBuffer));
}
