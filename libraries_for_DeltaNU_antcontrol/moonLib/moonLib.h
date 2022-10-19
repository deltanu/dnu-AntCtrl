void grid2deg(char *grid0,double *dlong,double *dlat);

void moon2(int y,int m,int Day,
  double UT,
  double lon,double lat,
  double *RA,double *Dec,
  double *topRA,double *topDec,
  double *LST,double *HA,
  double *Az,double *El,double *dist);

void MoonDop(int nyear, int month, int nday, double uth4, 
	double lon4, double lat4, 
	double *RAMoon4, double *DecMoon4,
	double *LST4,double *HA4,
	double *AzMoon4,double *ElMoon4, 
	double *vr4, double *dist4);

