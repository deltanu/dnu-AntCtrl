// Adapted from sgp4 library
// Predicts time of next pass and start azimuth for satellites 
long Predict(int many, double *azStart, double *maxEl){
    passinfo overpass;                      //structure to store overpass info
    sat.initpredpoint( timeNow , 0.0 );     //finds the startpoint
    
    bool error;
    for (int i = 0; i < many ; i++){
        error = sat.nextpass(&overpass,20);     //search for the next overpass, if there are more than 20 maximums below the horizon it returns false
        delay(0);
        
        if ( error == 1){ //no error, prints overpass information
          nextpassEpoch = (overpass.jdstart-2440587.5) * 86400;
          // Added to handle overflow number
          if((nextpassEpoch-timeNow) > 86400) {
            return timeNow; // Return the current time, it is either not valid satellite data or a Geostationary satellite
          }
          AZstart = overpass.azstart;
          invjday(overpass.jdstart ,timeZone ,true , year1, mon, day1, hr, min1, sec);   // Convert Julian date to print in serial.
          *azStart = overpass.azstart;    // Az start
          *maxEl = overpass.maxelevation; // max Elevation
          #ifdef DEBUG
            // Serial.println("Next pass for: " + String(satnames[SAT]) + " In: " + String(nextpassEpoch-timeNow));
            Serial.println("Next pass for: " + String(satelName[SAT]) + " In: " + String(nextpassEpoch-timeNow));
            Serial.println("Start: az=" + String(overpass.azstart) + "° " + String(hr) + ':' + String(min1) + ':' + String(sec));
            Serial.println("Max Elev=" + String(overpass.maxelevation) + "°");
          #endif
        }else{
            #ifdef DEBUG
              Serial.println("Prediction error");
            #endif
            while(true);
        }
        delay(0);
    }  
    return nextpassEpoch;
}
