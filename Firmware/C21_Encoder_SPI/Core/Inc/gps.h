#include <stdlib.h>
#include <string.h>
typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;

	char		    Pos;
	uint8_t			Lat_Deg;
	float			Lat_Minute;

	char            Lat_Dir;
	uint8_t         Lon_Deg;
	float           Lon_Minute;
	char            Lon_Dir;
	float			Speed;
	float			Track;
	unsigned int    date;

	float			mag_v;
	char			var_dir;
	char			mode_ind;

	uint8_t         sat_num;

	float			altitude;
	float			err_lat;
	float			err_lon;
	float			err_alt;
}GPS;

void substring(char s[], char sub[], int p, int l) {
   int c = 0;

   while (c < l) {
      sub[c] = s[p+c-1];
      c++;
   }
   sub[c] = '\0';
}

void process_NMEA(volatile GPS * gps,char Rx_Buffer[]){
	char * pch;

	char buffer[10];
	char temp_1[20];
	char time[8];
    // read GNRMC
    if(strstr (Rx_Buffer,"RMC")!= NULL ){

    char * n = " ";
    //HAL_UART_Transmit(&huart2, (uint8_t *) Rx_Buffer, sizeof(Rx_Buffer),1000);

    pch = strtok (Rx_Buffer,",");

    int i = 0;

    while (pch != NULL)
      {
    	i++;
    	char tem[2];
    	char deg[8];
    	strcpy((char*)temp_1, pch);
    	switch(i){
    	       case 2:
                       for(int j= 0; j<8;j++){
                    	   time[j] = NULL;
                       }
                       substring(temp_1, tem, 1, 2);
    	               gps->UTC_Hour = atoi(tem);
    	               itoa(gps->UTC_Hour,(char*) tem,10);
    	               strcat(time, tem);
    	               strcat(time, ":");

    	               substring(temp_1, tem, 3, 2);
    	               strcat(time, tem);
    	               strcat(time, ":");
    	               gps->UTC_Min = atoi((char*)tem);

    	               substring(temp_1, tem, 5, 2);
    	               strcat(time, tem);
    	               gps->UTC_Sec = atoi((char*)tem);
    	       break;
    	       case 3:
    	    	       gps->Pos = temp_1[0];
    	       break;
    	       case 4:

    	    	       substring(temp_1, tem, 1, 2);
    	               gps->Lat_Deg = atoi(tem);

    	               substring(temp_1, deg, 3, 4);
    	               gps->Lat_Minute = atoi(deg);
    	               substring(temp_1, deg, 6, 8);
    	               gps->Lat_Minute += (float) atoi(deg)/100000.0;

    	       break;
    	       case 5:
    	               gps->Lat_Dir = temp_1[0];
    	               break;
    	       case 6:
    	    	       for(uint8_t i = 0; i<8; i++){
    	    	    	   deg[i] = 0;
    	    	       }
    	    	       substring(temp_1, tem, 1, 3);
    	               gps->Lon_Deg = atoi(tem);

    	               substring(temp_1, deg, 4, 5);
    	               gps->Lon_Minute = atoi(deg);
    	               substring(temp_1, deg, 7, 8);
    	               gps->Lon_Minute += (float) atoi(deg)/100000.0;

    	              break;
    	       case 7:
    	                gps->Lon_Dir = temp_1[0];
    	              break;
    	       case 8:
    	                gps->Speed = atof(temp_1) * 0.514444856;
    	              break;
    	       case 9:
    	           	    gps->Track = atof(temp_1);
    	           	  break;

    	}
    	pch = strtok (NULL, ",");
      }
    }


    if(strstr (Rx_Buffer,"THS")!= NULL ){

    pch = strtok (Rx_Buffer,",");

    int i = 0;

    while (pch != NULL)
      {
       for(int i= 0; i<20;i++){
    	   temp_1[i] = 0;
    	}
    	i++;
    	char tem[2];
    	char deg[8];
    	strcpy(temp_1, pch);
    	switch(i){

    	       case 2:

    	       break;
//    	       case 4:
////    	    	       gps.Pos = temp_11[0];
//    	    	   gps.sat_num = atoi(temp_11);
//    	       break;

    	}
    	pch = strtok (NULL, ",");
      }
    }

    for(int i= 0; i<20;i++){
             temp_1[i] = 0;
        }

    if(strstr (Rx_Buffer,"GGA")!= NULL ){

     pch = strtok (Rx_Buffer,",");

     int i = 0;

     while (pch != NULL)
       {
     	i++;
     	char tem[2];
     	char deg[8];
     	strcpy(temp_1, pch);
     	switch(i){

     	       case 8:
     	//    	    	       gps.Pos = temp_1[0];
     	    	  gps->sat_num = atoi(temp_1);
     	       break;
     	       case 10:
 //    	    	       gps->Pos = temp_1[0];
     	    	   gps->altitude = atof(temp_1);
     	       break;

     	}
     	pch = strtok (NULL, ",");
       }
     }

}
