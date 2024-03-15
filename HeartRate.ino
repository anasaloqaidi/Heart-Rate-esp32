#define BLYNK_TEMPLATE_ID "TMPL6E9Uuw0S7"
#define BLYNK_TEMPLATE_NAME "heart rate monitoring"
#define BLYNK_AUTH_TOKEN "5Ch5Gn6rdM3aUzf1gHfQo2SkFN2Xx1v_"

#define BLYNK_PRINT Serial


#include <Wire.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal.h>
#include "MAX30105.h"    // sparkfun MAX3010X library
MAX30105 particleSensor;

#include "arduinoFFT.h"
arduinoFFT FFT;


LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
#define PULSE_SAMPLES 256
#define SAMPLE_FREQ   50

double avered       = 0; 
double aveir        = 0;
double sumirrms     = 0;
double sumredrms    = 0;
int    i            = 0;
int    Num          = 100;  // calculate SpO2 by this sampling interval
int    Temperature;
int    temp;
float  ESpO2;               // initial value of estimated SpO2
double FSpO2        = 0.7;  // filter factor for estimated SpO2
double frate        = 0.95; // low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT    3000  // wait for this time(msec) to output SpO2
#define SCALE         88.0  // adjust to display heart beat and SpO2 in the same scale
#define SAMPLING      100   //25 //5     // if you want to see heart beat more precisely, set SAMPLING to 1
#define FINGER_ON     30000 // if red signal is lower than this, it indicates your finger is not on the sensor
#define USEFIFO
#define PULSE_SAMPLES 256
#define SAMPLE_FREQ   50

// --- For Heart Rate ---
byte   rateSpot         = 0;
long   lastBeat         = 0;  // Time at which the last beat occurred
int    beatAvg          = 0;
bool   detect_high      = 0;
// ----------------------

double redArray[PULSE_SAMPLES]; // array to store samples from the sensor
double vReal[PULSE_SAMPLES];
double vImag[PULSE_SAMPLES];
double beatsPerMinute = 0;

char auth[] = BLYNK_AUTH_TOKEN;
BlynkTimer timer;

bool flag = true;


byte Heart[8] = {
0b00000,
0b01010,
0b11111,
0b11111,
0b01110,
0b00100,
0b00000,
0b00000
};



byte start[8] =
{
0b00000,
0b00000,
0b00000,
0b00000,
0b11111,
0b00000,
0b00000,
0b00000
};


byte end[8] =
{
0b00000,
0b00000,
0b00000,
0b11111,
0b00000,
0b00000,
0b00000,
0b00000
};

byte up[8] =
{
0b00000,
0b00010,
0b00101,
0b01000,
0b10000,
0b00000,
0b00000,
0b00000
};

byte down[8] =
{
0b00000,
0b00000,
0b00000,
0b10000,
0b01000,
0b00101,
0b00010,
0b00000
};

void heartUp(){

  for(int p =0; p<=15; p++){

    if(p==0){
        lcd.setCursor(p, 1);
      	lcd.write(byte(1));
    }else if (
      p==15
    ){
        lcd.setCursor(p, 1);
      	lcd.write(byte(2));
    }else if (
      p%2 !=0
    ){
        lcd.setCursor(p, 1);
      	lcd.write(byte(3));
    }else{
        lcd.setCursor(p, 1);
      	lcd.write(byte(4));
    }
  }
}
void heartDown(){

  for(int p =0; p<=15; p++){

    if(p==0){
        lcd.setCursor(p, 1);
      	lcd.write(byte(2));
    }else if (
      p==15
    ){
        lcd.setCursor(p, 1);
      	lcd.write(byte(1));
    }else if (
      p%2 !=0
    ){
        lcd.setCursor(p, 1);
      	lcd.write(byte(4));
    }else{
        lcd.setCursor(p, 1);
      	lcd.write(byte(3));
    }
  }
}

void heartNo(){

  for(int p =0; p<=15; p++){
        lcd.setCursor(p, 1);
      	lcd.write(byte(1));
  }
}
void setup()
{
     WiFi.mode(WIFI_STA);
     WiFiManager wm;
	lcd.begin(16, 2);

	// create a new character
	lcd.createChar(0, Heart);
	// create a new character
	lcd.createChar(1, start);
	// create a new character
	lcd.createChar(2, end);
	// create a new character
	lcd.createChar(3, up);
  	// create a new character
	lcd.createChar(4, down);

     lcd.clear();              
     lcd.setCursor(0, 0);    
     lcd.print("Choose Your WIFI");  

    bool res = wm.autoConnect("Heart Rate"); // anonymous ap

 
    if(!res) {

        lcd.clear();              // clear display
        lcd.setCursor(0, 0);      // move cursor to   (0, 0)
        lcd.print("Failed to connect");       // print message at (0, 0)
        // ESP.restart();
    } 
    else {

        if(WiFi.isConnected()){
         Blynk.begin(auth,(wm.getWiFiSSID()).c_str(),( wm.getWiFiPass()).c_str());
         while (!Blynk.connected()) {
                  lcd.clear();              // clear display
                  lcd.setCursor(0, 0);      // move cursor to   (0, 0)
                  lcd.print("Connecting to Blynk..."); 
           }
           
        lcd.clear();              // clear display
        lcd.setCursor(0, 0);      // move cursor to   (0, 0)
        lcd.print("Connected to Blynk");
        delay(3000);  
      }else{
        lcd.clear();              // clear display
        lcd.setCursor(0, 0);      // move cursor to   (0, 0)
        lcd.print("No internet!!");
        delay(3000);
      }

    }


   // Initialize sensor
   while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
   {
      //while (1);
   }

   //Setup to sense a nice looking saw tooth on the plotter
   byte ledBrightness = 0x7F;  // Options: 0=Off to 255=50mA
   byte sampleAverage = 4;     // Options: 1, 2, 4, 8, 16, 32
   byte ledMode       = 2;     // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
   //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
   int sampleRate     = 200;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
   int pulseWidth     = 411;   // Options: 69, 118, 215, 411
   int adcRange       = 16384; // Options: 2048, 4096, 8192, 16384
  
   // Set up the wanted parameters
   particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
   particleSensor.enableDIETEMPRDY();
}

void loop()
{
   uint32_t ir, red, green;
   double fred, fir;
   double SpO2 = 0; //raw SpO2 before low pass filtered
   float red_beat = 0;
   
#ifdef USEFIFO
   particleSensor.check();               // Check the sensor, read up to 3 samples

   while (particleSensor.available()) 
   {  // Do we have new data
#ifdef MAX30105
      red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
      ir  = particleSensor.getFIFOIR();  // Sparkfun's MAX30105
#else
      red = particleSensor.getFIFOIR();  // why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
      ir  = particleSensor.getFIFORed(); // why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif

      i++;
      i = i % PULSE_SAMPLES; // wrap around every 256 samples
      fred = (double)red;

      if (ir >= 50000 && i == 110){
               beatsPerMinute +=random(0,6);
               Blynk.virtualWrite(V0, beatsPerMinute);  
               lcd.clear();              // clear display
               lcd.setCursor(5, 0);      // move cursor to   (0, 0)    // move cursor to   (2, 1)
               lcd.print(beatsPerMinute);
               lcd.setCursor(11, 0);
      	       lcd.write(byte(0));
               if (flag){
                 heartUp();
                 flag=false;
               }else{
                 heartDown();
                 flag=true;
               }
           }
      redArray[i] = fred; // populate the array

      particleSensor.nextSample(); // We're finished with this sample so move to next sample

      if (i == 0) // execute every PULSE_SAMPLES
      {
        
         for (int idx=0; idx < PULSE_SAMPLES; idx++)
         {
            vReal[idx] = redArray[idx];
            vImag[idx] = 0.0;

            //Serial.println(redArray[idx]);
         }

         FFT = arduinoFFT(vReal, vImag, PULSE_SAMPLES, SAMPLE_FREQ); /* Create FFT object */
         FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
         FFT.Compute(FFT_FORWARD); /* Compute FFT */
         FFT.ComplexToMagnitude(); /* Compute magnitudes */

         double peak = FFT.MajorPeak();
         //Serial.println(peak, 6);

         // print in beats per minute
         beatsPerMinute = peak * 60;
         if (ir >= 50000){
               Blynk.virtualWrite(V0, beatsPerMinute);  
               lcd.clear();              // clear display
               lcd.setCursor(5, 0);      // move cursor to   (0, 0)    // move cursor to   (2, 1)
               lcd.print(beatsPerMinute);
               lcd.setCursor(11, 0);
      	       lcd.write(byte(0));
               if (flag){
                 heartUp();
                 flag=false;
               }else{
                 heartDown();
                 flag=true;
               }
           }else{
           Blynk.virtualWrite(V0, 0);  
           lcd.clear();              // clear display
           lcd.setCursor(7, 0);      // move cursor to   (0, 0)    // move cursor to   (2, 1)
           lcd.print(0);
           lcd.setCursor(8, 0);
      	   lcd.write(byte(0));
           heartNo();
           }


      }

  }
#endif

                    
  Blynk.run();


}