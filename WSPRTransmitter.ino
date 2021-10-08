/*
 *  Arduino Sketch WSPR transmitter with Si5351 on Adafruit Feather M0
 *
 *  WSPR transmit state machine
 *  state 0: after startup, no GPS fix
 *    |
 *    | GPS clock fix
 *    V
 *  state 1: GPS clock fix, show time and date in display
 *    |
 *    | GPS location fix
 *    V
 *  state 2: generate WSPR message
 *    |
 *    | second 0 of every second minute
 *    V
 *  state 3: start WSPR transmission
 *    |
 *    | WSPR transmission initialized
 *    V
 *  state 4: WSPR transmission
 *    |
 *    | WSPR transmission finished
 *    V
 *  state 2: load next frequency settings
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

#include <SPI.h>
#include "Wire.h"
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#include "SAMDTimerInterrupt.h"

// Mode defines
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

#define MENU_SLEEP 80

#define BACKLIGHTPIN 13

struct freq_set_t
{
  unsigned long long freq;
  enum si5351_clock clk;
  unsigned int pre_tune;
  bool active;
};

freq_set_t wsprfreqs[]={
// freq          clk          pre_tune  active
  { 21094600ULL, SI5351_CLK2,        0,      1},     // 15 meter band, 1600 Hz higher than dial freq, freq corrected
  { 24924600ULL, SI5351_CLK2,        0,      1},     // 12 meter band, 1600 Hz higher than dial freq, freq corrected
  { 28124600ULL, SI5351_CLK2,        0,      1},     // 10 meter band, 1600 Hz higher than dial freq, freq corrected
  { 50291620ULL, SI5351_CLK1,        0,      1},     //  6 meter band, 1600 Hz higher than dial freq, freq corrected
  { 70088670ULL, SI5351_CLK1,        1,      1},     //  4 meter band, 1600 Hz higher than dial freq, freq corrected
  {144482200ULL, SI5351_CLK0,        1,      1}      //  2 meter band, 1600 Hz higher than dial freq, freq corrected
};

// Class instantiation
Si5351 si5351;
JTEncode jtencode;

// Global variables
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint8_t symbol_count_state=0;

void onRmcUpdate(nmea::RmcData const);

ArduinoNmeaParser parser(onRmcUpdate, NULL);
DogGraphicDisplay DOG;
volatile time_t global_timestamp=0;
volatile bool flag_rmc=0;
volatile bool flag_timer=0;
nmea::RmcData global_rmc;

SAMDTimer ITimer0(TIMER_TC3);

// define some values used by the panel and buttons
#define AIN_KEYPAD A5
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// read the buttons
int read_LCD_buttons()
{
  int adc_key_in = analogRead(AIN_KEYPAD);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // my buttons when read are centered at these values (MKR1010): 0, 11, 162, 354, 531, 763
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 250)  return btnUP;
  if (adc_key_in < 450)  return btnDOWN;
  if (adc_key_in < 650)  return btnLEFT;
  if (adc_key_in < 850)  return btnSELECT;

  return btnNONE;  // when all others fail, return this...
}

// Loop through the string, transmitting one character at a time.
bool handle_wspr_tx(bool start_new, unsigned long long freq, enum si5351_clock clk)
{
  static uint8_t i;
  static enum si5351_clock clkint=clk;
  unsigned long long freqint=freq;
  uint16_t tone_spacing = WSPR_TONE_SPACING;

  if(start_new==true)
  {
    i=0;
    clkint=clk;
    freqint=freq;
    // Reset the tone to the base frequency and turn on the output
    si5351.output_enable(clkint, 1);
  }
  symbol_count_state=i;

  if(i < symbol_count)
  {
    unsigned long long int jf=(freqint * 100ULL) + (unsigned long long)(tx_buffer[i] * tone_spacing);
    si5351.set_freq(jf, clkint);
    i++;
    return true;
  }
  else
  {
  // Turn off the output
    si5351.output_enable(clkint, 0);
    return false;
  }
}

void set_tx_buffer(char* tx_call, char *loc, int dbm)
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  jtencode.wspr_encode(tx_call, loc, dbm, tx_buffer);
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

  pinMode(SCK, OUTPUT);   // set spi pin to output
  pinMode(MOSI, OUTPUT);   // set spi pin to output
  DOG.begin(&SPI,A1,A3,A2,DOGM132);   //Hardware-SPI, CS = A1, A0 = A3, RESET = A2, EA DOGM132-5 (=132x32 dots)
  DOG.clear();
//  DOG.createCanvas(128, 64, 0, 0, 1);  // Canvas in buffered mode

  DOG.string(0,0,UBUNTUMONO_B_16,"WSPR init ",ALIGN_CENTER); // print "SunPathClock" in line 3, centered
  DOG.string(0,2,UBUNTUMONO_B_16,"data not valid",ALIGN_CENTER); // print "not valid" in line 5
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);

  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // Set CLK1 output
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK1, 0); // Disable the clock initially

  // Set CLK2 output
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK2, 0); // Disable the clock initially
}

void generatefullcall(char *call, int prefix, int suffix, char *fullcall)
{
  const char* prefixstr[]={"OE","OK","ON"};
  int j=0;
  if(prefix>0)
  {
    fullcall[0]=prefixstr[prefix-1][0];
    fullcall[1]=prefixstr[prefix-1][1];
    if(fullcall[1]==0) j=1;
    else j=2;
    fullcall[j]='/';
    j++;
  }
  for(int i=0;i<6;i++)
  {
    fullcall[j]=call[i];
    j++;
  }
  if(prefix==0&&suffix>0)
  {
    fullcall[j]='/';
    j++;
    if(suffix==1) fullcall[j]='P';
    if(suffix==2) fullcall[j]='M';
    j++;
  }
  fullcall[j]=0;
}

void generatebracketcall(char *call, char *bracketcall)
{
  bracketcall[0]='<';
  for(int i=0;i<6;i++)
  {
    bracketcall[i+1]=call[i];
  }
  bracketcall[7]='>';
  bracketcall[8]=0;
}

void loop() {
  // put your main code here, to run repeatedly:
  static int state=0;
  static int freq_cycle=0;
  static char locatorbuf[]={"000000"};
  static char fullcall[]={"00000000000000"};
  static char bracketcall[]={"00000000000000"};
  static int backlight_timeout=0;
  int lcd_key=btnNONE;
  static int millis_flag=0;
  static int menu=0;
  static int menu_pointer=0;
  static uint8_t prefix=0;
  static uint8_t suffix=0;
  static int type2_3_count=0;
  char call[] = "N0CALL";
  uint8_t dbm = 10;
  static unsigned long long freq;
  static enum si5351_clock clk;
  static unsigned int pre_tune;

  generatefullcall(call, prefix, suffix, fullcall);
  generatebracketcall(call, bracketcall);

  while (Serial1.available()) {
    parser.encode((char)Serial1.read());
  }
  if(flag_timer) // this is called every 683 ms when WSPR transmission is active
  {
    flag_timer=0;
    if(state==3)
    {
      handle_wspr_tx(1,freq,clk);  // init wspr transmission
      state=4;
    }
    else if(state==4)
    {
      if(!(handle_wspr_tx(0,0,SI5351_CLK0))) // send more data
      {
        state=2; // stop transmission
        ITimer0.disableTimer(); // stop timer
        if(type2_3_count==0)
        {
          do
          {
            freq_cycle++;
            if(freq_cycle>=sizeof(wsprfreqs)/sizeof(freq_set_t)) freq_cycle=0;
          } while(wsprfreqs[freq_cycle].active==false);
        }
        freq =  wsprfreqs[freq_cycle].freq;   // get settings from struct defined at the beginning of the code
        clk =  wsprfreqs[freq_cycle].clk;
        pre_tune =  wsprfreqs[freq_cycle].pre_tune;
        if(pre_tune==1)
        {
          si5351.output_enable(clk, 1);
          si5351.set_freq(freq*100ULL, clk);
        }
  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
        if(prefix>0||suffix>0)
        {
          if(type2_3_count==0)  // type 2 message
          {
            set_tx_buffer(fullcall,locatorbuf,dbm);
            type2_3_count++;
          }
          else    // type 3 message
          {
            set_tx_buffer(bracketcall,locatorbuf,dbm);
            type2_3_count=0;
          }

        }
        else set_tx_buffer(call,locatorbuf,dbm);  // Type 1 message
      }
    }
  }
  if(flag_rmc) // this is called every time a GNSS message is decoded
  {
    flag_rmc=0;
    if(global_rmc.time_utc.hour>0)
    {
      global_timestamp = nmea::toPosixTimestamp(global_rmc.date, global_rmc.time_utc);
      setTime(global_timestamp);

      if (global_rmc.is_valid&&state==1)
      {
        // Set the proper frequency, tone spacing, symbol count, and
        // tone delay depending on mode
        freq = wsprfreqs[0].freq;
        clk = wsprfreqs[0].clk;
        pre_tune = wsprfreqs[0].pre_tune;
        symbol_count = WSPR_SYMBOL_COUNT; // From the library defines

        jtencode.latlon_to_grid(global_rmc.latitude,global_rmc.longitude,locatorbuf);
        state=2;
      }

      if(state==0) state=1;   // gps clock is fixed
      if(global_rmc.time_utc.second==0&&state>1)   // start of minute
      {
        if((global_rmc.time_utc.minute%2)==0)   // every second minute
        {
          ITimer0.attachInterruptInterval(WSPR_DELAY * 1000, TimerHandler0);
          state=3;    // start WSPR transmission
        }
      }
    }
  }
  if((millis()%100)==0)
  {
    if(millis_flag==0) millis_flag=1;
  }
  else millis_flag=0;
  if(millis_flag==1) // this is called every 100 ms
  {
    millis_flag=2;
    lcd_key = read_LCD_buttons();  // read the buttons
    if(lcd_key!=btnNONE) backlight_timeout=MENU_SLEEP;
    if(backlight_timeout>0)   // backlight active, menu functions active
    {
      backlight_timeout--;
      digitalWrite(BACKLIGHTPIN, HIGH);
      if((lcd_key==btnUP||lcd_key==btnDOWN)&&menu==0) menu=1;
      switch (lcd_key)               // depending on which button was pushed, we perform an action
      {
        case btnUP:               // up
          {
            if(menu_pointer>0) menu_pointer--;
            break;
          }
        case btnDOWN:               // down
          {
            if(menu_pointer<sizeof(wsprfreqs)/sizeof(freq_set_t)) menu_pointer++;
            break;
          }
        case btnRIGHT:               // right
          {
            if(menu==2&&menu_pointer==0) if(prefix<3) prefix++;
            if(menu==2&&menu_pointer==1) if(suffix<2) suffix++;
            if(menu==4&&(menu_pointer<sizeof(wsprfreqs)/sizeof(freq_set_t))) wsprfreqs[menu_pointer].active=true;
            break;
          }
        case btnLEFT:               // left
          {
            if(menu==2&&menu_pointer==0) if(prefix>0) prefix--;
            if(menu==2&&menu_pointer==1) if(suffix>0) suffix--;
            if(menu==4&&(menu_pointer<sizeof(wsprfreqs)/sizeof(freq_set_t))) wsprfreqs[menu_pointer].active=false;
            break;
          }
        case btnSELECT:               // select
          {
            if(menu==1&&menu_pointer==0) menu=2;
            else if(menu==1&&menu_pointer==1) menu=3;
            else if(menu==1&&menu_pointer==2) menu=4;
            else if(menu==1&&menu_pointer==3) menu=0;
            else if(menu==2&&menu_pointer>1) menu=1;
            else if(menu==3&&menu_pointer>1) menu=1;
            else if(menu==4&&(menu_pointer>=sizeof(wsprfreqs)/sizeof(freq_set_t))) menu=1;
            menu_pointer=0;
            break;
          }
      }
    }
    else
    {
      digitalWrite(BACKLIGHTPIN, LOW);
      menu=0;
      menu_pointer=0;
    }

    DOG.clear();
    if(menu==1) // main menu
    {
      if(menu_pointer==0) DOG.string(0,0,DENSE_NUMBERS_8,"SETTINGS", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,0,DENSE_NUMBERS_8,"SETTINGS", ALIGN_LEFT);
      if(menu_pointer==1) DOG.string(0,1,DENSE_NUMBERS_8,"GNSS INFOS", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,1,DENSE_NUMBERS_8,"GNSS INFOS", ALIGN_LEFT);
      if(menu_pointer==2) DOG.string(0,2,DENSE_NUMBERS_8,"FREQUENCIES", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,2,DENSE_NUMBERS_8,"FREQUENCIES", ALIGN_LEFT);
      if(menu_pointer==3) DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT);
    }
    else if(menu==2)  // settings
    {
      if(menu_pointer==0) DOG.string(0,0,DENSE_NUMBERS_8,"PREFIX", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,0,DENSE_NUMBERS_8,"PREFIX", ALIGN_LEFT);
      if(menu_pointer==1) DOG.string(0,1,DENSE_NUMBERS_8,"SUFFIX", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,1,DENSE_NUMBERS_8,"SUFFIX", ALIGN_LEFT);
      String prefix_str(prefix);
      DOG.string(40,0,DENSE_NUMBERS_8,prefix_str.c_str());
      String suffix_str(suffix);
      DOG.string(40,1,DENSE_NUMBERS_8,suffix_str.c_str());

      DOG.string(0,2,DENSE_NUMBERS_8,"CALL", ALIGN_LEFT);

      DOG.string(40,2,DENSE_NUMBERS_8,fullcall); // print call
      if(prefix>0||suffix>0) DOG.string(90,2,DENSE_NUMBERS_8,"TYPE 2+3"); // print type 1
      else DOG.string(90,2,DENSE_NUMBERS_8,"TYPE 1"); // print type 1
      if(menu_pointer>1) DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT);
    }
    else if(menu==3)  // gnss infos
    {
      DOG.string(0,0,DENSE_NUMBERS_8,"LAT", ALIGN_LEFT);
      DOG.string(0,1,DENSE_NUMBERS_8,"LON", ALIGN_LEFT);
      DOG.string(0,2,DENSE_NUMBERS_8,totimestrt(global_timestamp), ALIGN_LEFT); // print time
      DOG.string(0,2,DENSE_NUMBERS_8,todatestrt(global_timestamp), ALIGN_RIGHT); // print date
      String lat_str(global_rmc.latitude);
      DOG.string(30,0,DENSE_NUMBERS_8,lat_str.c_str());
      String lon_str(global_rmc.longitude);
      DOG.string(30,1,DENSE_NUMBERS_8,lon_str.c_str());

      if(menu_pointer>1) DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT, STYLE_INVERSE);
      else DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT);
    }
    else if(menu==4)  // frequencies
    {
      for(int i=0; i<4; i++)
      {
        if((i+4*(menu_pointer/4))<(sizeof(wsprfreqs)/sizeof(freq_set_t)))
        {
          String freq_str((unsigned int)wsprfreqs[i+4*(menu_pointer/4)].freq);
          if((menu_pointer%4)==i) DOG.string(0,i,DENSE_NUMBERS_8,freq_str.c_str(), ALIGN_LEFT, STYLE_INVERSE);
          else DOG.string(0,i,DENSE_NUMBERS_8,freq_str.c_str());
          String channel_str((unsigned int)wsprfreqs[i+4*(menu_pointer/4)].clk);
          DOG.string(60,i,DENSE_NUMBERS_8,channel_str.c_str());
          String pre_tune_str((unsigned int)wsprfreqs[i+4*(menu_pointer/4)].pre_tune);
          DOG.string(70,i,DENSE_NUMBERS_8,pre_tune_str.c_str());
          String active_str((unsigned int)wsprfreqs[i+4*(menu_pointer/4)].active);
          DOG.string(100,i,DENSE_NUMBERS_8,active_str.c_str());
        }
        else
        {
          if(menu_pointer>=(sizeof(wsprfreqs)/sizeof(freq_set_t))) DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT, STYLE_INVERSE);
          else DOG.string(0,3,DENSE_NUMBERS_8,"<- BACK", ALIGN_LEFT);
          break;
        }
      }
    }
    else
    {
      DOG.string(0,0,DENSE_NUMBERS_8,locatorbuf, ALIGN_RIGHT); // print locator
      DOG.string(0,1,DENSE_NUMBERS_8,fullcall, ALIGN_RIGHT); // print call
      if(state==0) DOG.string(0,2,UBUNTUMONO_B_16,"data not valid",ALIGN_CENTER); // print "not valid" in line 2
      if(state>=1)    // clock fix
      {
        DOG.string(0,3,DENSE_NUMBERS_8,totimestrt(global_timestamp), ALIGN_LEFT); // print time
        DOG.string(0,3,DENSE_NUMBERS_8,todatestrt(global_timestamp), ALIGN_RIGHT); // print date
      }
      if(state>=2)    // ready to send
      {
        String freq_str((unsigned int)freq);
        DOG.string(0,2,DENSE_NUMBERS_8,freq_str.c_str(),ALIGN_CENTER);
        String channel_str((unsigned int)clk);
        DOG.string(0,2,DENSE_NUMBERS_8,channel_str.c_str(),ALIGN_RIGHT);
        String state_str((unsigned int)symbol_count_state);
        DOG.string(0,2,DENSE_NUMBERS_8,state_str.c_str());
        String statec_str((unsigned int)symbol_count);
        DOG.string(20,2,DENSE_NUMBERS_8,statec_str.c_str());
        DOG.string(15,2,DENSE_NUMBERS_8,"/");
      }
      if(state==0) DOG.string(0,0,UBUNTUMONO_B_16," CLK wait ",ALIGN_LEFT); // print status in line 0
      if(state==1) DOG.string(0,0,UBUNTUMONO_B_16," GPS wait ",ALIGN_LEFT); // print status in line 0
      if(state==2) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR wait ",ALIGN_LEFT); // print status in line 0
      if(state==3) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR start",ALIGN_LEFT); // print status in line 0
      if(state==4) DOG.string(0,0,UBUNTUMONO_B_16,"WSPR send ",ALIGN_LEFT); // print status in line 0
    }
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
