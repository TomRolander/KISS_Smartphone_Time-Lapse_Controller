/**************************************************************************
  KISS Time-Lapse Camera Controller

  Original Code:  2022-11-01

  Tom Rolander, MSEE
  Mentor, Circuit Design & Software
  Miller Library, Fabrication Lab
  Hopkins Marine Station, Stanford University,
  120 Ocean View Blvd, Pacific Grove, CA 93950
  +1 831.915.9526 | rolander@stanford.edu

  To Do:

  Notes: 

    Setup for Huge APP:
      Tools -> Partition Scheme -> Huge APP

    ESP32_BLE_Keyboard from GitHub
      https://github.com/T-vK/ESP32-BLE-Keyboard
          Download ESP32-BLE-Keyboard-master.zip
          copy to arduino/libraries

    Board Manager:
    esp32 by Espressif Systems
    2.0.17

    Libraries:
    Using library ESP32 BLE Keyboard at version 0.3.2 in folder: C:\Users\Tom Rolander\Documents\Arduino\libraries\ESP32_BLE_Keyboard
    Using library NimBLE-Arduino at version 1.4.1 in folder: C:\Users\Tom Rolander\Documents\Arduino\libraries\NimBLE-Arduino
    Using library EEPROM at version 2.0.0 in folder: C:\Users\Tom Rolander\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\EEPROM
    Using library WiFi at version 2.0.0 in folder: C:\Users\Tom Rolander\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\WiFi
    Using library ESPAsyncWebServer at version 3.1.0 in folder: C:\Users\Tom Rolander\Documents\Arduino\libraries\ESPAsyncWebServer
    Using library FS at version 2.0.0 in folder: C:\Users\Tom Rolander\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\libraries\FS
    Using library AsyncTCP at version 1.1.4 in folder: C:\Users\Tom Rolander\Documents\Arduino\libraries\AsyncTCP 


 **************************************************************************/

#define PROGRAM "KISS Time-Lapse Camera Controller"
#define VERSION "Ver 0.9 2024-09-04"

#define DEBUG_OUTPUT  1

// NOTE: this define is required inside BleKeyboard.h to work on iPhones and Androids!
// NOTE: update the following Arduino library
//         ..\Documents\Arduino\libraries\ESP32_BLE_Keyboard\BleKeyboard.h
//       uncomment the following line to use NimBLE library
//       #define USE_NIMBLE
#define USE_NIMBLE

#include <BleKeyboard.h>    //https://github.com/T-vK/ESP32-BLE-Keyboard
//BleKeyboard bleKeyboard("KISS 00 Time-Lapse BLE Kybd", "Hopkins Miller Fab Lab", 100);
BleKeyboard bleKeyboard("", "Hopkins Miller Fab Lab", 100);

#include <EEPROM.h>
// EEPROM.write(address, value);
// EEPROM.commit();
// EEPROM.read(address);

// EEPROM state variables
#define EEPROM_SIZE 41
#define EEPROM_SIGNATURE                    0 // 000-003  Signature 'KISS'
#define EEPROM_NUMBER_OF_LOOPS              4 // 004-005  iNumberOfLoops
#define EEPROM_TIME_DELAY_SECONDS           6 // 006-007  iTimeDelaySeconds
#define EEPROM_VIDEO_RECORD_SECONDS         8 // 008-009  iVideoRecordSeconds
#define EEPROM_LIGHT_DELAY_BEFORE_SECONDS  10 // 010-011  iLightDelayBeforeSeconds
#define EEPROM_LIGHT_DELAY_AFTER_SECONDS   12 // 012-013  iLightDelayAfterSeconds
#define EEPROM_CAMERA_ID                   14 // 014-015  cCameraID
#define EEPROM_ACTIVE_HOURS                16 // 016-039  bActiveHours
#define EEPROM_CONTINUOUS_MODE             40 // 040-040  bContinuousMode

#define LIGHT_CONTROL_PIN 4

#define NUMBER_OF_LOOPS             2
#define TIME_DELAY_SECONDS          60
#define VIDEO_RECORD_SECONDS        60
#define LIGHT_DELAY_BEFORE_SECONDS  0
#define LIGHT_DELAY_AFTER_SECONDS   0

#define MAX_OPERATIONS          10
#define OP_PHOTO                1
#define OP_STARTVIDEO           2
#define OP_STOPVIDEO            3
#define OP_STARTPHOTOTIMELAPSE  4
#define OP_STARTVIDEOTIMELAPSE  5
#define OP_STOPTIMELAPSE        6
#define OP_SETTINGS             7
#define OP_CANCEL               8
#define OP_REBOOT               9
#define OP_STATUSUPDATE        10
#define OP_LIGHTON             11
#define OP_LIGHTOFF            12


#define TIMEDELAYSECONDS        "TIMEDELAYSECONDS"
#define NUMBEROFLOOPS           "NUMBEROFLOOPS"
#define VIDEORECORDSECONDS      "VIDEORECORDSECONDS"
#define LIGHTDELAYBEFORESECONDS "LIGHTDELAYBEFORESECONDS"
#define LIGHTDELAYAFTERSECONDS  "LIGHTDELAYAFTERSECONDS"
#define CAMERAID                "CAMERAID"

// NOTE: I switched to noInterrupts() and interrupts() around critical sections

static int iOperations[MAX_OPERATIONS];
static int iOpIn = 0;
static int iOpOut = 0;
static int iOpCount = 0;

#include <WiFi.h>

#define WIFI_TIMEOUT  10000

#include <ESPAsyncWebServer.h>

const char* wifi_network_ssid     = "Stanford";
const char* wifi_network_password =  "";

char soft_ap_ssid[]               = "KISS 00 Time-Lapse Camera";
//                                   0123456
//const char *soft_ap_password      = "KISS-HMS";
const char *soft_ap_password      = "";

// Set web server port number to 80
AsyncWebServer server(80);
//WiFiServer server(80);

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -(8*3600);
int   daylightOffset_sec = 3600;  //  = 0;

#define STATUSUPDATE_INITIAL  0
#define STATUSUPDATE_RUNNING  1
#define STATUSUPDATE_FINISHED 2

static int iStatusUpdateState = STATUSUPDATE_INITIAL;

#define NO_TIMELAPSE    0
#define PHOTO_TIMELAPSE 1
#define VIDEO_TIMELAPSE 2

#define NO_CONTINUOUS_MODE    0
#define PHOTO_CONTINUOUS_MODE 1
#define VIDEO_CONTINUOUS_MODE 2

static int iTimeLapse = NO_TIMELAPSE;
static bool bTimeLapseFinished = false;

static int iTimeDelaySeconds = TIME_DELAY_SECONDS;
static int iNumberOfLoops = NUMBER_OF_LOOPS;
static int iLoopCounter = 1;
static int iVideoRecordSeconds = VIDEO_RECORD_SECONDS;
static long lVideoRecordMilliseconds = 0;
static int iLightDelayBeforeSeconds = LIGHT_DELAY_BEFORE_SECONDS;
static int iLightDelayAfterSeconds = LIGHT_DELAY_AFTER_SECONDS;
static char cCameraID[3] = {'0','1','\0'};   // 01
static byte bActiveHours[24] = {1, 1, 1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1,
                                1, 1, 1, 1, 1, 1
                               };

static byte bContinuousMode = NO_CONTINUOUS_MODE;       

static byte bFirstLoop = true;

char sKeyboardName[] = "KISS 00 Time-Lapse BLE Kybd";
//                      0123456

static long lTimeMillisecondsPhoto = 0L;
static long lTimeMillisecondsVideo = 0L;

static bool bUseNTP = true;
struct tm timeinfo;


// Auxiliar variables to store the current output state
int bRecordingVideo = false;

static bool bReady = true;

static char sTimeBuffer[80] = "";

static char index_html[6144] = "";

// HTML web page

const char index_html_preamble[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head><meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
.button { background-color: #4CAF50; border: none; color: white; padding: 4px 10px;
text-decoration: none; font-size: 16px; margin: 2px; cursor: pointer;}
.button2 {background-color: #555555;}
h1 {font-size: 20px;}</style>)rawliteral";

const char index_html_postamble[] PROGMEM = R"rawliteral(
<p><a href="/PHOTO"><button class="button button2">TAKE PHOTO</button></a></p>
<p><a href="/STARTVIDEO"><button class="button button2">START VIDEO</button></a></p>
<p><a href="/STATUSUPDATEPHOTOTIMELAPSE"><button class="button button2">PHOTO TIME LAPSE</button></a></p>
<p><a href="/STATUSUPDATEVIDEOTIMELAPSE"><button class="button button2">VIDEO TIME LAPSE</button></a></p>
<p><a href="/SETTINGS"><button class="button button2">SETTINGS</button></a></p>
<p>LIGHT <a href="/LIGHTON"><button class="button button2">ON</button></a> <a href="/LIGHTOFF"><button class="button button2">OFF</button></a></p>
</body></html>)rawliteral";

const char index_html_video[] PROGMEM = R"rawliteral(
<p><a href="/STOPVIDEO"><button class="button button2">STOP VIDEO</button></a></p>
</body></html>)rawliteral";

const char index_html_refresh[] PROGMEM = R"rawliteral(
<meta http-equiv="refresh" content="10">)rawliteral";


const char index_html_statusupdate[] PROGMEM = R"rawliteral(
<p><a href="/STOPTIMELAPSE"><button class="button button2">STOP TIME LAPSE</button></a></p>
<p><a href="/STATUSUPDATEPHOTOTIMELAPSE"><button class="button button2">STATUS UPDATE</button></a></p>
)rawliteral";

const char index_html_statusupdatefinished[] PROGMEM = R"rawliteral(
<p><a href="/CONTINUE"><button class="button button2">Continue</button></a></p>
</body></html>
)rawliteral";

const char index_html_settingspreamble[] PROGMEM = R"rawliteral(
<form action="/GET">
<center>
<table style="border-collapse: collapse;">
)rawliteral";

/*
  <tr>
    <td style="text-align: left;"><label>Time Delay Seconds:</label></td>
    <td style="text-align: left;"><input type="text" id="TIMEDELAYSECONDS"
maxlength="3" size="3" name="TIMEDELAYSECONDS" value="6"></td>
  </tr>
  <tr>
    <td style="text-align: left;"><label>Number of Loops:</label></td>
    <td style="text-align: left;"><input type="text" id="NUMBEROFLOOPS"
maxlength="3" size="3" name="NUMBEROFLOOPS"></td>
  </tr>
  <tr>
    <td style="text-align: left;"><label>Video Record Seconds:</label></td>
    <td style="text-align: left;"><input type="text" id="VIDEORECORDSECONDS"
maxlength="3" size="3" name="VIDEORECORDSECONDS"></td>
  </tr>
  <tr>
    <td style="text-align: left;"><label>Light Delay Before Seconds:</label></td>
    <td style="text-align: left;"><input type="text" id="LIGHTDELAYBEFORESECONDS"
maxlength="3" size="3" name="LIGHTDELAYBEFORESECONDS"></td>
  </tr>
*/
  
const char index_html_settingspostamble[] PROGMEM = R"rawliteral(
  <tr>
    <td style="text-align: left;"><label>Camera ID:</label></td>
    <td style="text-align: left;"><input type="text" id="CAMERAID"
maxlength="3" size="3" name="CAMERAID"></td>
  </tr>
</table>
)rawliteral";

#if 0
const char index_html_settingscheckboxes[] PROGMEM = R"rawliteral(
Active Hours
    <table>
      <tbody>
        <tr>
      <td>00 to 05</td>
          <td><input type="checkbox" name="checkbox1" checked></td>
          <td><input type="checkbox" name="checkbox2" checked></td>
          <td><input type="checkbox" name="checkbox3" checked></td>
          <td><input type="checkbox" name="checkbox4" checked></td>
          <td><input type="checkbox" name="checkbox5" checked></td>
          <td><input type="checkbox" name="checkbox6" checked></td>
        </tr>
        <tr>
      <td>06 to 11</td>
          <td><input type="checkbox" name="checkbox7" checked></td>
          <td><input type="checkbox" name="checkbox8" checked></td>
          <td><input type="checkbox" name="checkbox9" checked></td>
          <td><input type="checkbox" name="checkbox10" checked></td>
          <td><input type="checkbox" name="checkbox11" checked></td>
          <td><input type="checkbox" name="checkbox12" checked></td>
        </tr>
        <tr>
      <td>12 to 17</td>
          <td><input type="checkbox" name="checkbox13" checked></td>
          <td><input type="checkbox" name="checkbox14" checked></td>
          <td><input type="checkbox" name="checkbox15" checked></td>
          <td><input type="checkbox" name="checkbox16" checked></td>
          <td><input type="checkbox" name="checkbox17" checked></td>
          <td><input type="checkbox" name="checkbox18" checked></td>
        </tr>
        <tr>
      <td>18 to 23</td>
          <td><input type="checkbox" name="checkbox19" checked></td>
          <td><input type="checkbox" name="checkbox20" checked></td>
          <td><input type="checkbox" name="checkbox21" checked></td>
          <td><input type="checkbox" name="checkbox22" checked></td>
          <td><input type="checkbox" name="checkbox23" checked></td>
          <td><input type="checkbox" name="checkbox24" checked></td>
        </tr>
      </tbody>
    </table>
)rawliteral";
#endif

const char index_html_settingsupdate[] PROGMEM = R"rawliteral(
</center>
<input type="submit" value="UPDATE">
</form>
<p><a href="/CANCEL"><button class="button button2">CANCEL</button></a></p>
<p><a href="/REBOOT"><button class="button button2">REBOOT</button></a></p>
</body></html>
)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

char sH1[] = "</head><body><h1>KISS 01 Time-Lapse<br>Camera Controller<br>&nbsp;<br>";
//            012345678901234567890123456789
//                      1         2 
void DoH1(char *sTitle)
{
  sH1[22] = cCameraID[0];
  sH1[23] = cCameraID[1];
  strcat(index_html, sH1);
  strcat(index_html, sTitle);
  strcat(index_html, "</h1>");
}

void doSettingsValue(char *sDescription, char *sID, int iCurrentValue)
{
  strcat(index_html, "<tr><td style=\"text-align: left;\"><label>");
  strcat(index_html, sDescription);
  strcat(index_html, "</label></td><td style=\"text-align: left;\"><input type=\"text\" id=\"");
  strcat(index_html, sID);     
  strcat(index_html, "\"maxlength=\"4\" size=\"4\" name=\"");
  strcat(index_html, sID);
  strcat(index_html, "\" value=\"");
  char sParam[10] = "";
  itoa(iCurrentValue,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "\"></td></tr>");
}

void doCheckBoxes()
{
  strcat(index_html, "Active Photo Hours\r\n<table><tbody>\r\n");
  for (int iHour=1; iHour<25; iHour=iHour+6)
  {
    char sParam[128];
    sprintf(sParam, "<tr><td>%02d to %02d</td>\r\n", iHour-1, iHour+4);
    strcat(index_html, sParam);
    for (int i=0; i<6; i++)
    {
      // <td><input type="checkbox" name="checkbox1" checked></td>
      sprintf(sParam, "<td><input type=\"checkbox\" name=\"checkbox%d\" ", iHour+i);      
      strcat(index_html, sParam);
      if (bActiveHours[(iHour-1)+i])
        strcat(index_html, "checked");
      strcat(index_html, "></td>\r\n");
    }
    strcat(index_html, "</tr>\r\n");    
  }
  strcat(index_html, "</tbody></table>\r\n");
}


void SendOpToLoop(AsyncWebServerRequest *request, int iOp, char *index_html)
{
  //Access KISS and perform operations
 
  noInterrupts();
  
  if (iOpCount < MAX_OPERATIONS)
  {
    iOpCount = iOpCount + 1;
    iOperations[iOpIn] = iOp;
    iOpIn = iOpIn + 1;
    if (iOpIn >= MAX_OPERATIONS)
      iOpIn = 0;
  }   
  
  //Enable interrupts once insertion is done
  interrupts();
  
  request->send_P(200, "text/html", index_html);  
}

void Format_index_html(bool bRoot)
{
  char sParam[10] = "";

  printLocalTime();
  
  strcpy(index_html, index_html_preamble);
  DoH1("");
  
  if (bRoot)
    strcat(index_html, VERSION);
  else
    strcat(index_html, sTimeBuffer);
  
  strcat(index_html, "<p>");  
  strcat(index_html, "Time Delay Seconds = ");
  itoa(iTimeDelaySeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Number of Loops = ");
  itoa(iNumberOfLoops,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Video Record Seconds = ");
  itoa(iVideoRecordSeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Light Delay Before Seconds = ");
  itoa(iLightDelayBeforeSeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Light Delay After Seconds = ");
  itoa(iLightDelayAfterSeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");

  strcat(index_html, "</p>");  
  strcat(index_html, index_html_postamble);
}

void ShowSettings()
{
  char sParam[10] = "";

  strcat(index_html, "Time Delay Seconds = ");
  itoa(iTimeDelaySeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Number of Loops = ");
  itoa(iNumberOfLoops,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Video Record Seconds = ");
  itoa(iVideoRecordSeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Light Delay Before Seconds = ");
  itoa(iLightDelayBeforeSeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");
  strcat(index_html, "Light Delay After Seconds = ");
  itoa(iLightDelayAfterSeconds,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "<br>");  
  strcat(index_html, "&nbsp;<br>");  
}

void StatusUpdate()
{ 
  char sParam[10] = "";

  strcpy(index_html, index_html_preamble);
  strcat(index_html, index_html_refresh);
  DoH1("TIME LAPSE");
  ShowSettings();
  strcat(index_html, "<p>");
  strcat(index_html, sTimeBuffer);
  strcat(index_html, "</p>");  
  strcat(index_html, "<p>&nbsp;&nbsp;On loop ");
  itoa(iLoopCounter,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, " of ");
  itoa(iNumberOfLoops,sParam,10);
  strcat(index_html, sParam);
  strcat(index_html, "</p>");  
  strcat(index_html, index_html_statusupdate);
  strcat(index_html, "</body></html>");
}

void KISS_HTTP_Handler()
{
#if DEBUG_OUTPUT
Serial.println("KISS_HTTP_Handler");
#endif
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Format_index_html(true);
    request->send_P(200, "text/html", index_html);
  });

  server.on("/PHOTO", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("PHOTO");
#endif
    Format_index_html(false);
    SendOpToLoop(request, OP_PHOTO, (char *) index_html);
  });
 
  server.on("/STARTVIDEO", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("STARTVIDEO");
#endif
    strcpy(index_html, index_html_preamble);
    DoH1("");
    strcat(index_html, index_html_video);

    SendOpToLoop(request, OP_STARTVIDEO, (char *) index_html);
  });
 
  server.on("/STOPVIDEO", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("STOPVIDEO");
#endif
    Format_index_html(false);
    SendOpToLoop(request, OP_STOPVIDEO, (char *) index_html);
  });

  server.on("/STATUSUPDATEPHOTOTIMELAPSE", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("STATUSUPDATEPHOTOTIMELAPSE");
#endif
    printLocalTime();
    if (iStatusUpdateState == STATUSUPDATE_INITIAL)
    {
      StatusUpdate();
      SendOpToLoop(request, OP_STARTPHOTOTIMELAPSE, (char *) index_html);
      iStatusUpdateState = STATUSUPDATE_RUNNING;
      return;
    }

    if (iStatusUpdateState == STATUSUPDATE_RUNNING &&
        bTimeLapseFinished == false)
    {
      StatusUpdate();
      SendOpToLoop(request, OP_STATUSUPDATE, (char *) index_html);
      return;
    }
      
    iStatusUpdateState = STATUSUPDATE_FINISHED;
    iTimeLapse = NO_TIMELAPSE;
    strcpy(index_html, index_html_preamble);
    strcat(index_html, index_html_refresh);  
    DoH1("TIME LAPSE FINISHED");
    strcat(index_html, index_html_statusupdatefinished);
    SendOpToLoop(request, OP_STATUSUPDATE, (char *) index_html);   
  });

  server.on("/STATUSUPDATEVIDEOTIMELAPSE", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("STATUSUPDATEVIDEOTIMELAPSE");
#endif
    printLocalTime();

    if (iStatusUpdateState == STATUSUPDATE_INITIAL)
    {
      StatusUpdate();
      SendOpToLoop(request, OP_STARTVIDEOTIMELAPSE, (char *) index_html);
      iStatusUpdateState = STATUSUPDATE_RUNNING;
      return;
    }

    if (iStatusUpdateState == STATUSUPDATE_RUNNING &&
        bTimeLapseFinished == false)
    {
      StatusUpdate();
      SendOpToLoop(request, OP_STATUSUPDATE, (char *) index_html);
      return;
    }
      
    iStatusUpdateState = STATUSUPDATE_FINISHED;
    iTimeLapse = NO_TIMELAPSE;
    strcpy(index_html, index_html_preamble);
    DoH1("TIME LAPSE FINISHED");
    strcat(index_html, index_html_statusupdatefinished);
    SendOpToLoop(request, OP_STATUSUPDATE, (char *) index_html); 
  });

  server.on("/CONTINUE", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("CONTINUE");
#endif
    iStatusUpdateState = STATUSUPDATE_INITIAL;
    Format_index_html(false);
    SendOpToLoop(request, OP_CANCEL, (char *) index_html);
  }); 
 
  server.on("/STOPTIMELAPSE", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("STOPTIMELAPSE");
#endif
    iLoopCounter = 1;
    bTimeLapseFinished = false;
    iStatusUpdateState = STATUSUPDATE_INITIAL;
    Format_index_html(false);
    SendOpToLoop(request, OP_STOPTIMELAPSE, (char *) index_html);
  });
 
  server.on("/SETTINGS", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("SETTINGS");
#endif
    strcpy(index_html, index_html_preamble);
    DoH1("SETTINGS");
    //ShowSettings();
    strcat(index_html, index_html_settingspreamble);

    doSettingsValue("Time Delay Seconds:", "TIMEDELAYSECONDS", iTimeDelaySeconds);
    doSettingsValue("Number of Loops:", "NUMBEROFLOOPS", iNumberOfLoops);
    doSettingsValue("Video Record Seconds:", "VIDEORECORDSECONDS", iVideoRecordSeconds);
    doSettingsValue("Light Delay Before Seconds:", "LIGHTDELAYBEFORESECONDS", iLightDelayBeforeSeconds);
    doSettingsValue("Light Delay After Seconds:", "LIGHTDELAYAFTERSECONDS", iLightDelayAfterSeconds);
    
    strcat(index_html, index_html_settingspostamble);
    doCheckBoxes();
    strcat(index_html, index_html_settingsupdate);
    SendOpToLoop(request, OP_SETTINGS, (char *) index_html);
  });
 
  server.on("/LIGHTON", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("LIGHTON");
#endif
    iStatusUpdateState = STATUSUPDATE_INITIAL;
    Format_index_html(false);
    SendOpToLoop(request, OP_LIGHTON, (char *) index_html);
  }); 
 
  server.on("/LIGHTOFF", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("LIGHTOFF");
#endif
    iStatusUpdateState = STATUSUPDATE_INITIAL;
    Format_index_html(false);
    SendOpToLoop(request, OP_LIGHTOFF, (char *) index_html);
  }); 
 
  server.on("/CANCEL", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("CANCEL");
#endif
    Format_index_html(false);
    SendOpToLoop(request, OP_CANCEL, (char *) index_html);
  });
 
  server.on("/REBOOT", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
#if DEBUG_OUTPUT
Serial.println("REBOOT");
#endif
    Format_index_html(false);
    SendOpToLoop(request, OP_REBOOT, (char *) index_html);
  });
 
  server.on("/GET", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;

#if DEBUG_OUTPUT
Serial.println("/GET");        
#endif
    if (request->hasParam(TIMEDELAYSECONDS)) {
      inputMessage = request->getParam(TIMEDELAYSECONDS)->value();
      if (inputMessage.length() > 0)
      {
        iTimeDelaySeconds = inputMessage.toInt();
        if (iTimeDelaySeconds < 1)
          iTimeDelaySeconds = 1;
      }
    }
    
    if (request->hasParam(NUMBEROFLOOPS)) {
      inputMessage = request->getParam(NUMBEROFLOOPS)->value();
      if (inputMessage.length() > 0)
      {
        iNumberOfLoops = inputMessage.toInt();
        //if (iNumberOfLoops < 2)
        //  iNumberOfLoops = 2;
      }
    }
    
    if (request->hasParam(VIDEORECORDSECONDS)) {
      inputMessage = request->getParam(VIDEORECORDSECONDS)->value();
      if (inputMessage.length() > 0)
      {
        iVideoRecordSeconds = inputMessage.toInt();
        if (iVideoRecordSeconds < 1)
          iVideoRecordSeconds = 1;
        lVideoRecordMilliseconds = 1000L * (long) iVideoRecordSeconds;
      }
    }
    
    if (request->hasParam(LIGHTDELAYBEFORESECONDS)) {
      inputMessage = request->getParam(LIGHTDELAYBEFORESECONDS)->value();
      if (inputMessage.length() > 0)
      {
        iLightDelayBeforeSeconds = inputMessage.toInt();
        //if (iLightDelayBeforeSeconds < 1)
        //  iLightDelayBeforeSeconds = 1;
      }
    }
    
    if (request->hasParam(LIGHTDELAYAFTERSECONDS)) {
      inputMessage = request->getParam(LIGHTDELAYAFTERSECONDS)->value();
      if (inputMessage.length() > 0)
      {
        iLightDelayAfterSeconds = inputMessage.toInt();
        //if (iLightDelayAfterSeconds < 1)
        //  iLightDelayAfterSeconds = 1;
      }
    }

    bool bDoReboot = false;
    
    if (request->hasParam(CAMERAID)) {
      inputMessage = request->getParam(CAMERAID)->value();
      if (inputMessage.length() > 0)
      {
        int iCameraID = inputMessage.toInt();
        if (iCameraID > 0 && iCameraID < 100)
        {
          cCameraID[0] = (iCameraID/10) + '0';
          cCameraID[1] = (iCameraID%10) + '0';
          bDoReboot = true;
        }
      }
    }
    
    for (int i=1; i<25; i++)
    {
      char sParam[20];
      String stringOn = "on";
      sprintf(sParam, "checkbox%d", i);
      if (request->hasParam(sParam))
      {
        bActiveHours[i-1] = true;
      }
      else
      {
        bActiveHours[i-1] = false;
      }
    }
    
    SetupEEPROM();

    Format_index_html(false);
    request->send(200, "text/html", index_html);
    
    if (bDoReboot)
      ESP.restart();                      

  });
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(PROGRAM);
  Serial.println(VERSION);
  Serial.println();

  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_SIGNATURE+0) == 'K' &&
      EEPROM.read(EEPROM_SIGNATURE+1) == 'I' &&
      EEPROM.read(EEPROM_SIGNATURE+2) == 'S' &&
      EEPROM.read(EEPROM_SIGNATURE+3) == 'S')
  {
    iNumberOfLoops = readUnsignedIntFromEEPROM(EEPROM_NUMBER_OF_LOOPS);
    iTimeDelaySeconds = readUnsignedIntFromEEPROM(EEPROM_TIME_DELAY_SECONDS);
    iVideoRecordSeconds = readUnsignedIntFromEEPROM(EEPROM_VIDEO_RECORD_SECONDS);
    lVideoRecordMilliseconds = 1000L * (long) iVideoRecordSeconds;
    iLightDelayBeforeSeconds = readUnsignedIntFromEEPROM(EEPROM_LIGHT_DELAY_BEFORE_SECONDS);
    iLightDelayAfterSeconds = readUnsignedIntFromEEPROM(EEPROM_LIGHT_DELAY_AFTER_SECONDS);
    cCameraID[0] = EEPROM.read(EEPROM_CAMERA_ID+0);
    cCameraID[1] = EEPROM.read(EEPROM_CAMERA_ID+1);
    if (cCameraID[0] < '0' || cCameraID[0] > '9' ||
        cCameraID[1] < '0' || cCameraID[1] > '9')
    {
      cCameraID[0] = '0';
      cCameraID[1] = '1';   // 01
    }
    sKeyboardName[5] = cCameraID[0];
    sKeyboardName[6] = cCameraID[1];
    for (int i=0; i<24; i++)
    {
      bActiveHours[i] = EEPROM.read(EEPROM_ACTIVE_HOURS+i);
    }
    bContinuousMode = EEPROM.read(EEPROM_CONTINUOUS_MODE);
  }
  else
  {
#if DEBUG_OUTPUT
Serial.println("EEPROM KISS not found");
#endif
    SetupEEPROM();
  }
#if DEBUG_OUTPUT
Serial.println("EEPROM KISS found");
Serial.print("iTimeDelaySeconds = ");
Serial.println(iTimeDelaySeconds);
Serial.print("iNumberOfLoops = ");
Serial.println(iNumberOfLoops);
Serial.print("iVideoRecordSeconds = ");
Serial.println(iVideoRecordSeconds);
Serial.print("iLightDelayBeforeSeconds = ");
Serial.println(iLightDelayBeforeSeconds);
Serial.print("iLightDelayAfterSeconds = ");
Serial.println(iLightDelayAfterSeconds);
Serial.print("cCameraID = ");
Serial.println(cCameraID);
#endif    
//  EEPROM.end();
  
  pinMode(LIGHT_CONTROL_PIN, OUTPUT);
  digitalWrite(LIGHT_CONTROL_PIN, LOW);

#if DEBUG_OUTPUT
  Serial.print("ESP Board MAC Address =  ");
  Serial.println(WiFi.macAddress());  
  Serial.println("\n[*] Creating ESP32 AP");
#endif
  WiFi.mode(WIFI_AP_STA);  

  soft_ap_ssid[5] = cCameraID[0];
  soft_ap_ssid[6] = cCameraID[1];
  WiFi.softAP(soft_ap_ssid, soft_ap_password);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());
  Serial.println(soft_ap_ssid);
  
  WiFi.begin(wifi_network_ssid, wifi_network_password);
  Serial.println("\n[*] Connecting to WiFi Network");

  int iTimeoutCounter = 0;
  while(WiFi.status() != WL_CONNECTED)
  {
      Serial.print(".");
      delay(100);
      iTimeoutCounter += 100;
      if (iTimeoutCounter > WIFI_TIMEOUT)
      {
        bUseNTP = false;
        break;
      }
  }

  if (bUseNTP)
  {
    Serial.print("\n[+] Connected to the WiFi network with local IP : ");
    Serial.println(WiFi.localIP());
  
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
  }
  else
  {
    Serial.println("Failed Connection to the WiFi!");
    Serial.println("NTP access suppressed and Active Hours ignored");
  }

  KISS_HTTP_Handler();
    
  server.onNotFound(notFound);
  server.begin();

  // Start the Bluetooth Keyboard
  sKeyboardName[5] = cCameraID[0];
  sKeyboardName[6] = cCameraID[1];
  bleKeyboard.setName(sKeyboardName);
  bleKeyboard.begin();
  Serial.print(sKeyboardName);
  Serial.println(" -> BLE Bluetooth initiated");
}

void loop()
{
  if (bFirstLoop)
  {
#if DEBUG_OUTPUT
Serial.println("FirstLoop");
#endif
    bFirstLoop = false;
    if (bContinuousMode == PHOTO_CONTINUOUS_MODE)
    {
      TakePhotoWithLightControl(true);
      //if (iNumberOfLoops > 1)          
      {
        iTimeLapse = PHOTO_TIMELAPSE;
        lTimeMillisecondsPhoto = millis();
        iLoopCounter = 1;
      }      
    }
    else
    if (bContinuousMode == VIDEO_CONTINUOUS_MODE)
    {
      StartVideoWithLightControl();
      //if (iNumberOfLoops > 1)          
      {
        iTimeLapse = VIDEO_TIMELAPSE;
        lTimeMillisecondsPhoto = millis();
        iLoopCounter = 1;
      }      
    }
  }
  
  if (iTimeLapse != NO_TIMELAPSE)
  {
    if (bRecordingVideo)
    {
      if (millis() > (lTimeMillisecondsVideo + lVideoRecordMilliseconds))
      {
        StopVideoWithLightControl();
        if (iNumberOfLoops != 0 &&
            iLoopCounter >= iNumberOfLoops)
        {
          bTimeLapseFinished = true;
        }
      }       
    }
    
    if (millis() > (lTimeMillisecondsPhoto + (1000L * (long) iTimeDelaySeconds)))
    {
      if (iTimeLapse == PHOTO_TIMELAPSE)
        TakePhotoWithLightControl(true);
      else
        StartVideoWithLightControl();
        
      iLoopCounter++;
      lTimeMillisecondsPhoto = millis();
      if (iNumberOfLoops != 0 &&
          iLoopCounter >= iNumberOfLoops &&
          iTimeLapse == PHOTO_TIMELAPSE)
      {
        bTimeLapseFinished = true;
      }
    }
  }
  
  noInterrupts();

  //Access KISS and perform operations
  if (iOpCount > 0)
  {
    interrupts();
    switch (iOperations[iOpOut])
    {
      case OP_PHOTO:
#if DEBUG_OUTPUT
Serial.println("OP_PHOTO");
#endif
        TakePhotoWithLightControl(false);
        bRecordingVideo = false;
        break;

      case OP_STARTVIDEO:
#if DEBUG_OUTPUT
Serial.println("OP_STARTVIDEO");
#endif
        digitalWrite(LIGHT_CONTROL_PIN, HIGH);
        delay(iLightDelayBeforeSeconds * 1000L);          
        bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
        bRecordingVideo = true;
        break;

      case OP_STOPVIDEO:
#if DEBUG_OUTPUT
Serial.println("OP_STOPVIDEO");
#endif
        bRecordingVideo = false;
        bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
        delay(iLightDelayAfterSeconds * 1000L);      
        digitalWrite(LIGHT_CONTROL_PIN, LOW);
        break;

      case OP_STARTPHOTOTIMELAPSE:
#if DEBUG_OUTPUT
Serial.println("OP_STARTPHOTOTIMELAPSE");
#endif
        if (iNumberOfLoops == 0)
        {
          bContinuousMode = PHOTO_CONTINUOUS_MODE;
          SetupEEPROM();
        }
        
        TakePhotoWithLightControl(true);
        //if (iNumberOfLoops > 1)          
        {
          iTimeLapse = PHOTO_TIMELAPSE;
          lTimeMillisecondsPhoto = millis();
          iLoopCounter = 1;
        }
        break;

      case OP_STARTVIDEOTIMELAPSE:
#if DEBUG_OUTPUT
Serial.println("OP_STARTVIDEOTIMELAPSE");
#endif

        if (iNumberOfLoops == 0)
        {
          bContinuousMode = VIDEO_CONTINUOUS_MODE;
          SetupEEPROM();
        }

        StartVideoWithLightControl();
        //if (iNumberOfLoops > 1)          
        {
          iTimeLapse = VIDEO_TIMELAPSE;
          lTimeMillisecondsPhoto = millis();
          iLoopCounter = 1;
        }
        break;

      case OP_STOPTIMELAPSE:
#if DEBUG_OUTPUT
Serial.println("OP_STOPTIMELAPSE");
#endif
        iLoopCounter = 0;      
        //if (iTimeLapse == VIDEO_TIMELAPSE)
        if (bRecordingVideo)
        {
          StopVideoWithLightControl();
        }
        iTimeLapse = NO_TIMELAPSE;
        bTimeLapseFinished = false;
        iStatusUpdateState = STATUSUPDATE_INITIAL;
        
        bContinuousMode = NO_TIMELAPSE;
        SetupEEPROM();
        break;

      case OP_SETTINGS:
#if DEBUG_OUTPUT
Serial.println("OP_SETTINGS");
#endif
        break;

      case OP_LIGHTON:
#if DEBUG_OUTPUT
Serial.println("OP_LIGHTON");
#endif
        digitalWrite(LIGHT_CONTROL_PIN, HIGH);
        break;

      case OP_LIGHTOFF:
#if DEBUG_OUTPUT
Serial.println("OP_LIGHTOFF");
#endif
        digitalWrite(LIGHT_CONTROL_PIN, LOW);
        break;

      case OP_CANCEL:
#if DEBUG_OUTPUT
Serial.println("OP_CANCEL");
#endif
        bTimeLapseFinished = false;
        break;
        
      case OP_REBOOT:
#if DEBUG_OUTPUT
Serial.println("OP_REBOOT");
Serial.println("REBOOT KISS Time-Lapse 01 Camera Controller");
#endif
        ESP.restart();                      
        break;

      case OP_STATUSUPDATE:
#if DEBUG_OUTPUT
Serial.println("OP_STATUSUPDATE");
#endif
        break;
        
    }
    iOpCount = iOpCount - 1;
    iOpOut = iOpOut + 1;
    if (iOpOut >=  MAX_OPERATIONS)
      iOpOut = 0;     
  }
  else
  { 
    //Unlock once operations are done
    interrupts();
  }
}

void printLocalTime()
{
  if (bUseNTP == false)
  {
    sTimeBuffer[0] = '\0';
    return;
  }

  if(!getLocalTime(&timeinfo))
  {
    bUseNTP = false;
#if DEBUG_OUTPUT
    Serial.println("Failed to obtain time");
#endif  
    return;
  }
  strftime(sTimeBuffer,sizeof(sTimeBuffer),"%Y-%m-%d %H:%M:%S", &timeinfo);
#if DEBUG_OUTPUT
  Serial.println(sTimeBuffer);
#endif   
}

void TakePhotoWithLightControl(bool bCheckActiveHours)
{
  //Serial.println("TakePhotoWithLightControl");
  if (bCheckActiveHours)
  {
//////////////////
#if 0
Serial.print("bUseNTP=");
Serial.println(bUseNTP);
Serial.print("getLocalTime(&timeinfo)=");
Serial.println(getLocalTime(&timeinfo));        
Serial.print("timeinfo.tm_hour=");
Serial.println(timeinfo.tm_hour);        
Serial.print("bActiveHours[timeinfo.tm_hour]=");
Serial.println(bActiveHours[timeinfo.tm_hour]);
#endif
        if (bUseNTP == true &&
            getLocalTime(&timeinfo) == true &&
            bActiveHours[timeinfo.tm_hour] == false)
{
#if DEBUG_OUTPUT
Serial.println("PHOTO NOT TAKEN, NOT ACTIVE HOUR");
#endif
          return;
        }
//////////////////    
  }
  
#if DEBUG_OUTPUT
Serial.println("TAKE PHOTO");
#endif

  digitalWrite(LIGHT_CONTROL_PIN, HIGH);
  delay(iLightDelayBeforeSeconds * 1000L);
  
  bleKeyboard.write(KEY_MEDIA_VOLUME_UP);

  delay(iLightDelayAfterSeconds * 1000L);      
  digitalWrite(LIGHT_CONTROL_PIN, LOW);
}

void StartVideoWithLightControl()
{
#if DEBUG_OUTPUT
Serial.println("START VIDEO");
#endif

  digitalWrite(LIGHT_CONTROL_PIN, HIGH);
  delay(iLightDelayBeforeSeconds * 1000L);
  
  bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
  
  lTimeMillisecondsVideo = millis();
  bRecordingVideo = true;
}

void StopVideoWithLightControl()
{
#if DEBUG_OUTPUT
Serial.println("StopVideoWithLightControl");
#endif

  bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
  
  bRecordingVideo = false;
  delay(iLightDelayAfterSeconds * 1000L);      
  digitalWrite(LIGHT_CONTROL_PIN, LOW);
}

void writeUnsignedIntIntoEEPROM(int address, unsigned int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int readUnsignedIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void SetupEEPROM()
{
  EEPROM.begin(100);    
  EEPROM.write(EEPROM_SIGNATURE+0, 'K');
  EEPROM.write(EEPROM_SIGNATURE+1, 'I');
  EEPROM.write(EEPROM_SIGNATURE+2, 'S');
  EEPROM.write(EEPROM_SIGNATURE+3, 'S');
  writeUnsignedIntIntoEEPROM(EEPROM_NUMBER_OF_LOOPS, iNumberOfLoops);
  writeUnsignedIntIntoEEPROM(EEPROM_TIME_DELAY_SECONDS, iTimeDelaySeconds);
  writeUnsignedIntIntoEEPROM(EEPROM_VIDEO_RECORD_SECONDS, iVideoRecordSeconds);
  writeUnsignedIntIntoEEPROM(EEPROM_LIGHT_DELAY_BEFORE_SECONDS, iLightDelayBeforeSeconds);
  writeUnsignedIntIntoEEPROM(EEPROM_LIGHT_DELAY_AFTER_SECONDS, iLightDelayAfterSeconds); 
  EEPROM.write(EEPROM_CAMERA_ID+0, cCameraID[0]);
  EEPROM.write(EEPROM_CAMERA_ID+1, cCameraID[1]);
  for (int i=0; i<24; i++)
  {
    EEPROM.write(EEPROM_ACTIVE_HOURS+i, bActiveHours[i]);
  }
  EEPROM.write(EEPROM_CONTINUOUS_MODE, bContinuousMode);
  EEPROM.commit();    

#if DEBUG_OUTPUT
Serial.println("EEPROM KISS initialized");
#endif
}
