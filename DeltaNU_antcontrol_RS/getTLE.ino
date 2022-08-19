void getTLE(int SAT) {
  // Make HTTP request and get TLE for satellite//
    String serverName = "http://celestrak.org";
    String serverPath = serverName + String(satURL[SAT]);
    http.begin(serverPath.c_str());

  // Send HTTP GET request
    int httpResponseCode = http.GET();
    if (httpResponseCode>0) {
        String payload = "";
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        payload = http.getString();
        Serial.println(payload);
        int j = 0;
        // payload.toCharArray(TLE, sizeof(TLE));
        payload.toCharArray(TLE, 200);
        j = sizeof(TLE);
        Serial.println("j index: " + String(j));
        k = 0;
        for (j = 0; j < 25; j++) { //TLE line 0 spans characters 0 - 25, Satellite Name
          satNameTLE[SAT][k] = TLE[j];
          k++;
        }
        //
        if (k > 0) {
          Serial.println("satNameTLE[SAT]: " + String(satNameTLE[SAT]));
        }
        //
        k = 0;
        for (j = 26; j < 96; j++) { //TLE line 1 spans characters 26 - 96
          TLE1[SAT][k] = TLE[j];
          k++;
        }
        k = 0;
        for (j = 97; j < 167; j++) { //TLE line 2 spans characters 97 - 167
          TLE2[SAT][k] = TLE[j];
          k++;
        }
        
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
}
