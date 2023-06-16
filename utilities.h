
/*
 * Given a byte array of length (n), return the ASCII hex representation
 * and properly zero pad values less than 0x10.
 * String(0x08, HEX) will yield '8' instead of the expected '08' string
 */
String hexToStr(uint8_t* arr, int n)
{
  String result;
  result.reserve(2*n);
  for (int i = 0; i < n; ++i) {
    if (arr[i] < 0x10) {result += '0';}
    result += String(arr[i], HEX);
  }
  return result;
}




#include "time.h"

/*
 * Extra libraries installed from the Library Manager
 */
// https://github.com/arduino-libraries/NTPClient
#include <NTPClient.h>

/*
 * Return a string in RFC3339 format of the current time.
 * Will return a placeholder if there is no network connection to an
 * NTP server.
 */
String getIsoTime()
{
  char timeStr[21] = {0};  // NOTE: change if strftime format changes

  time_t time_now = ntpClient.getEpochTime();
  localtime_r(&time_now, &timeinfo);

  if (timeinfo.tm_year <= (2016 - 1900)) {
    return String("YYYY-MM-DDTHH:MM:SSZ");
  } else {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(timeStr);
  }
}
