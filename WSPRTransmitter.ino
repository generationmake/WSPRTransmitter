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

// Mode defines
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

#define WSPR_DEFAULT_FREQ       28123800ULL    // 1600 Hz higher than dial freq, freq corrected
//#define WSPR_DEFAULT_FREQ       28126200ULL    // 1600 Hz higher than dial freq
//#define WSPR_DEFAULT_FREQ       50294600ULL    // 1600 Hz higher than dial freq
#define DEFAULT_MODE            MODE_WSPR

// Class instantiation
Si5351 si5351;
JTEncode jtencode;

// Global variables
unsigned long long freq;
char call[] = "N0CALL";
char loc[] = "AA00";
uint8_t dbm = 27;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

#define BACKLIGHTPIN 10

void onRmcUpdate(nmea::RmcData const);

ArduinoNmeaParser parser(onRmcUpdate, NULL);
DogGraphicDisplay DOG;
volatile time_t global_timestamp=0;
volatile bool flag_rmc=0;
nmea::RmcData global_rmc;

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(LED_BUILTIN, HIGH);

  for(i = 0; i < symbol_count; i++)
  {
    unsigned long long int jf=(freq * 100ULL) + (unsigned long long)(tx_buffer[i] * tone_spacing);
    si5351.set_freq(jf, SI5351_CLK0);
    delay(tone_delay);
  }

  // Turn off the output
 // si5351.output_enable(SI5351_CLK0, 0); // do not turn off so that there is less drift
  digitalWrite(LED_BUILTIN, LOW);
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
  freq = WSPR_DEFAULT_FREQ;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
}

void loop() {
  // put your main code here, to run repeatedly:
  static int state=0;

  while (Serial1.available()) {
    parser.encode((char)Serial1.read());
  }
  if(flag_rmc)
  {
    flag_rmc=0;
    if(global_rmc.time_utc.hour>0)
    {
      global_timestamp = nmea::toPosixTimestamp(global_rmc.date, global_rmc.time_utc);
      setTime(global_timestamp);
      if(state==0) state=1;
      if(state==2) state=3;
      if(global_rmc.time_utc.second==0)   // start of minute
      {
        if((global_rmc.time_utc.minute%2)==0)   // every second minute
        {
          state=2;    // start WSPR transmission
        }
      }
    }
    if(state==2) encode();    // send WSPR data

    DOG.clear();
    if(state>0)
    {
      DOG.string(0,2,DENSE_NUMBERS_8,totimestrt(global_timestamp), ALIGN_CENTER); // print time
      DOG.string(0,3,DENSE_NUMBERS_8,todatestrt(global_timestamp), ALIGN_CENTER); // print date
    }
    else DOG.string(0,2,UBUNTUMONO_B_16,"data not valid",ALIGN_CENTER); // print "not valid" in line 2 
    if(state==1) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR wait ",ALIGN_CENTER); // print status in line 0 
    if(state==2) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR start",ALIGN_CENTER); // print status in line 0 
    if(state==3) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR send ",ALIGN_CENTER); // print status in line 0 
  }
}

/* ---------------- interrupt functions ------------------ */
void onRmcUpdate(nmea::RmcData const rmc)
{
  global_rmc=rmc;
  flag_rmc=1;
}
