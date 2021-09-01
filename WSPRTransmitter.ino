/* 
 *  Arduino Sketch WSPR transmitter with Si5351 on Adafruit Feather M0
 */
#include <DogGraphicDisplay.h>
#include <ArduinoNmeaParser.h>
#include "dense_numbers_8.h"
#include "ubuntumono_b_16.h"
#include <TimeLib.h>

#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>

#include "Wire.h"
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#include "SAMDTimerInterrupt.h"

// Mode defines
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

struct freq_set_t
{
  unsigned long long freq;
  unsigned int channel;
  unsigned int pre_tune;
};

freq_set_t wsprfreqs[]={
// freq         channel       pre_tune   
//  { 28123800ULL,   2,             0},     // 10 meter band, 1600 Hz higher than dial freq, freq corrected
  { 50290220ULL,   0,             0},     //  6 meter band, 1600 Hz higher than dial freq, freq corrected
  { 70086470ULL,   0,             0},     //  4 meter band, 1600 Hz higher than dial freq, freq corrected
  {144478400ULL,   1,             1},     //  2 meter band, 1600 Hz higher than dial freq, freq corrected
  {144478400ULL,   1,             1}      //  2 meter band, 1600 Hz higher than dial freq, freq corrected
};

// Class instantiation
Si5351 si5351;
JTEncode jtencode;

// Global variables
unsigned long long freq;
unsigned int channel;
unsigned int pre_tune;
char call[] = "N0CALL";
char loc[] = "AA00";
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

#define BACKLIGHTPIN 10

void onRmcUpdate(nmea::RmcData const);

ArduinoNmeaParser parser(onRmcUpdate, NULL);
DogGraphicDisplay DOG;
volatile time_t global_timestamp=0;
volatile bool flag_rmc=0;
volatile bool flag_timer=0;
nmea::RmcData global_rmc;
const char *locatorbuf;

SAMDTimer ITimer0(TIMER_TC3);


const char *maidenhead(float lon, float lat)
{
  static char locator[7]="000000";  // create buffer
  int x, y;
  locator[0]=(int)(lon+180.0)/20+65;
  locator[1]=(int)(lat+90.0)/10+65;
  locator[2]=((int)(lon+180.0)%20)/2+48;
  locator[3]=(int)(lat+90.0)%10+48;
  locator[4]=(int)(((lon/2+90.0)-(int)(lon/2+90.0))*24.0)+65;
  locator[5]=(int)(((lat+90.0)-(int)(lat+90.0))*24.0)+65;
  return locator;
}

// Loop through the string, transmitting one character at a time.
bool handle_wspr_tx(bool start_new)
{
  static uint8_t i;
  static uint8_t channelint=0;

  if(start_new==true)
  {
    i=0;
    channelint=channel;
    // Reset the tone to the base frequency and turn on the output
    if(channelint==0) si5351.output_enable(SI5351_CLK0, 1);
    if(channelint==1) si5351.output_enable(SI5351_CLK1, 1);
  //  digitalWrite(LED_BUILTIN, HIGH);
  }

  if(i < symbol_count)
  {
    unsigned long long int jf=(freq * 100ULL) + (unsigned long long)(tx_buffer[i] * tone_spacing);
    if(channelint==0) si5351.set_freq(jf, SI5351_CLK0);
    if(channelint==1) si5351.set_freq(jf, SI5351_CLK1);
//    delay(tone_delay);
    i++;
    return true;
  }
  else
  {
  // Turn off the output
    if(channelint==0) si5351.output_enable(SI5351_CLK0, 0);
    if(channelint==1) si5351.output_enable(SI5351_CLK1, 0);
//  digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
}
// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
//  digitalWrite(LED_BUILTIN, HIGH);

  for(i = 0; i < symbol_count; i++)
  {
    unsigned long long int jf=(freq * 100ULL) + (unsigned long long)(tx_buffer[i] * tone_spacing);
    si5351.set_freq(jf, SI5351_CLK0);
    delay(tone_delay);
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
//  digitalWrite(LED_BUILTIN, LOW);
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
}

/* ---------------- functions to format and print time and date ------------------ */
char * totimestrt(time_t t)
{
  tmElements_t someTime;
  breakTime(t, someTime);

  static char timestr[]="00:00:00";
  timestr[0]=((someTime.Hour/10)%10)+48;
  timestr[1]=((someTime.Hour)%10)+48;
  timestr[3]=((someTime.Minute/10)%10)+48;
  timestr[4]=((someTime.Minute)%10)+48;
  timestr[6]=((someTime.Second/10)%10)+48;
  timestr[7]=((someTime.Second)%10)+48;
  return timestr;
}
char * todatestrt(time_t t)
{
  tmElements_t someTime;
  breakTime(t, someTime);

  static char timestr[]="00.00.0000";
  timestr[0]=((someTime.Day/10)%10)+48;
  timestr[1]=((someTime.Day)%10)+48;
  timestr[3]=((someTime.Month/10)%10)+48;
  timestr[4]=((someTime.Month)%10)+48;
  timestr[6]=((tmYearToCalendar(someTime.Year)/1000)%10)+48;
  timestr[7]=((tmYearToCalendar(someTime.Year)/100)%10)+48;
  timestr[8]=((tmYearToCalendar(someTime.Year)/10)%10)+48;
  timestr[9]=((tmYearToCalendar(someTime.Year))%10)+48;
  return timestr;
}

/* ---------------- arduino functions ------------------ */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  while(!Serial);
  Serial.println("WSPR");
  Serial1.begin(9600);

  // set Time clock to Jan. 1, 2000
  setTime(SECS_YR_2000);

  pinMode(BACKLIGHTPIN, OUTPUT);   // set backlight pin to output
  digitalWrite(BACKLIGHTPIN, HIGH);

  DOG.begin(A1,0,0,A3,A2,DOGM132);   //CS = A1, 0,0= use Hardware SPI, A0 = A3, RESET = A2, EA DOGM132-5 (=132x32 dots)
  DOG.clear();
  DOG.createCanvas(128, 64, 0, 0, 1);  // Canvas in buffered mode

  DOG.string(0,0,UBUNTUMONO_B_16,"WSPR init ",ALIGN_CENTER); // print "SunPathClock" in line 3, centered
  DOG.string(0,2,UBUNTUMONO_B_16,"data not valid",ALIGN_CENTER); // print "not valid" in line 5 
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set the proper frequency, tone spacing, symbol count, and
  // tone delay depending on mode
  freq = wsprfreqs[0].freq;
  channel = wsprfreqs[0].channel;
  pre_tune = wsprfreqs[0].pre_tune;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // Set CLK1 output
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK1, 0); // Disable the clock initially

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
//  set_tx_buffer();
}

void loop() {
  // put your main code here, to run repeatedly:
  static int state=0;
  static int freq_cycle=0;

  while (Serial1.available()) {
    parser.encode((char)Serial1.read());
  }
  if(flag_timer)
  {
    flag_timer=0;
    if(state==3) 
    {
      handle_wspr_tx(1);  // init wspr transmission
      state=4;
    }
    else if(state==4)
    {
      if(!(handle_wspr_tx(0))) // send more data
      {
        state=2; // stop transmission     
        ITimer0.disableTimer(); // stop timer
        freq_cycle++;
        if(freq_cycle>=4) freq_cycle=0;
        freq =  wsprfreqs[freq_cycle].freq;   // get settings from struct defined at the beginning of the code
        channel =  wsprfreqs[freq_cycle].channel;
        pre_tune =  wsprfreqs[freq_cycle].pre_tune;
        if(pre_tune==1)
        {
          if(channel==0) si5351.output_enable(SI5351_CLK0, 1);
          if(channel==1) si5351.output_enable(SI5351_CLK1, 1);
          if(channel==0) si5351.set_freq(freq*100ULL, SI5351_CLK0);
          if(channel==1) si5351.set_freq(freq*100ULL, SI5351_CLK1);
        }
      }
    }
  }
  if(flag_rmc)
  {
    flag_rmc=0;
    if(global_rmc.time_utc.hour>0)
    {
      global_timestamp = nmea::toPosixTimestamp(global_rmc.date, global_rmc.time_utc);
      setTime(global_timestamp);

      if (global_rmc.is_valid&&state==1)
      {
        locatorbuf=maidenhead(global_rmc.longitude,global_rmc.latitude);
        loc[0]=locatorbuf[0];
        loc[1]=locatorbuf[1];
        loc[2]=locatorbuf[2];
        loc[3]=locatorbuf[3];
        set_tx_buffer();
        state=2;
      }

      if(state==0) state=1;
//      if(state==3) state=4;
      if(global_rmc.time_utc.second==0&&state>1)   // start of minute
      {
        if((global_rmc.time_utc.minute%2)==0)   // every second minute
        {
          ITimer0.attachInterruptInterval(WSPR_DELAY * 1000, TimerHandler0);
          state=3;    // start WSPR transmission
        }
      }
    }
//    if(state==3) encode();    // send WSPR data

    DOG.clear();
    if(state>0)
    {
      DOG.string(0,3,DENSE_NUMBERS_8,totimestrt(global_timestamp), ALIGN_LEFT); // print time
      DOG.string(0,3,DENSE_NUMBERS_8,todatestrt(global_timestamp), ALIGN_RIGHT); // print date
    }
    else DOG.string(0,2,UBUNTUMONO_B_16,"data not valid",ALIGN_CENTER); // print "not valid" in line 2 
    if(state>1)
    {
      String freq_str((unsigned int)freq);
      DOG.string(0,2,DENSE_NUMBERS_8,freq_str.c_str(),ALIGN_CENTER);
      String channel_str((unsigned int)channel);
      DOG.string(0,2,DENSE_NUMBERS_8,channel_str.c_str(),ALIGN_RIGHT);
      DOG.string(0,0,UBUNTUMONO_B_16,locatorbuf, ALIGN_RIGHT); // print locator
    }
    if(state==1) DOG.string(0,0,UBUNTUMONO_B_16," GPS wait ",ALIGN_LEFT); // print status in line 0 
    if(state==2) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR wait ",ALIGN_LEFT); // print status in line 0 
    if(state==3) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR start",ALIGN_LEFT); // print status in line 0 
    if(state==4) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR send ",ALIGN_LEFT); // print status in line 0 
  }
}

/* ---------------- interrupt functions ------------------ */
void onRmcUpdate(nmea::RmcData const rmc)
{
  global_rmc=rmc;
  flag_rmc=1;
}

void TimerHandler0()
{
  flag_timer=1;
}
