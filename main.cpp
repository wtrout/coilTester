//Ford Model T coil tester
//
// Writen and constructed by wTrout 2020
// - highly inspired and conceptually derived from the work of Luke P and Robert R and modifications of Matt of mtfca.com forums
// - https://www.mtfca.com/phpBB3/viewtopic.php?f=2&t=9072
//
// Released under the GNU General Public License


// The code is designed to run on an Atmel SAMD21 chip: https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
// It is tested on an Adafruit Qt Pi board: https://www.adafruit.com/product/4600
// Auxillary hardware used:
// - RA8875 TFT Driver board: https://www.adafruit.com/product/1590
// - 480x272 pixel TFT: https://www.adafruit.com/product/1591
// - ACS712 current sensor: https://www.allegromicro.com/en/products/sense/current-sensor-ics/zero-to-fifty-amp-integrated-conductor-sensor-ics/acs712
// - a cheap generic solid state relay for DC switching

// Ford Model Ts have 4 trembler coils used in the ignition system (one per cylinder).
// These are electro-mechanical devices and much be adjusted such that each has a similar response to the others.
// This code operates as a sort of purpose built oscilloscope to detect the time from powering the coil to the spark firing.


// The chart below is a typical profile of a coil firing. When the relay is switched on, current begins to rise until the coil switches off
// the current subsequently rapidly falls in the inductor creating a high voltage spark which arcs over the spark gap

//                             | Time to fire / max current
//                             |
//                             |
//                           ...
// ^                       ..
// a                     ..
// m                   ..       .
// p                 ..
// s               ..
//               ..
// time -> ......                .................... 

// Ford Model T : the original mass produced car from Henry Fond; the "any color as long as it's black" car **
// coil : trembler coil used in Ford Model T ignition systems; https://en.wikipedia.org/wiki/Trembler_coil
// TTF = time to fire : the time from powering the coil until the spark fires; reported in mS or µS


// ===================================================================
// ===================================================================

#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_GFX.h"
// Graphics class for drawing/text 
// https://github.com/adafruit/Adafruit-GFX-Library

#include "Adafruit_RA8875.h"
// RA8875 display driver library
// https://github.com/adafruit/Adafruit_RA8875

#include <avr/dtostrf.h>
// String manipulation for printing text

#include wiring_private.h
// pinPeripheral for analogReading

// ===================================================================
// ===================================================================

// Device Connection Configuration
#define RA8875_CS 4 // SDA for display
#define RA8875_RESET 5 // SCL for display
const byte currentSensorPin = A0; // analog in for ACS712 current sensor
const byte _AREF = A1 // voltage divider to 5V rail
const byte relayPin = A6; // TX pin on Qt Py for triggering SSR
const byte userInput = A7; // RX pin on Qt Py for user input buttons


//chart setup and scaling data
const byte readResolutionSetting = 12;
const int ADCscale = 4096; //full range of 12bit ADC (2^readResolutionSetting)
const int yPixels = 272; // display height
const int xPixels = 480; // display width
const int xOrigin = 21; // chart origin (x-axis)
const int yOrigin = yPixels -16; // chart origin (y-axis)
const int chartHeight = 200; // top of chart (y-axis)
const int chartWidth = xPixels - xOrigin - 5; // rightmost edge of chart (x-axis)
const int maxmA = 4000; // set top of chart to top out at 4 amps (coils generally test ~3 to 3.5 amps at 12V)
const byte firePeriod = 133; // mS period between sparks on multi-test (60,000 ms / 900 rpm * 2)
const int numReads = 450; // 450 data points are recorded for each coil fire (~10 uS per data point)
const int multiFireCount = 50; // multi-fire tests perform 50 runs (each with 450 points)
int adcZeroOffset; // value for zeroing the ACS712 output

// Call the TFT using adafruit's library
Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

// Data buffers
volatile int analogVals[numReads]; // buffer for raw analogReads of ACS712 Vout
volatile int multiFireData[multiFireCount][2]; //stores the key data (time to fire and peak current) for each run in a multi-fire test (ttf and maxCurrent)


// ===================================================================
// ===================================================================


// Function for resetting the µC from software using the watch dog timer; not implemented but retained for possible future use
void systemReset() {
  //use WDT to reset since EMI won't allow for direct connection to RST
  WDT->CTRL.reg = 0; // disable watchdog
  while (WDT->STATUS.bit.SYNCBUSY == 1); // sync is required

  WDT->CONFIG.reg = min(0, 11); // see Table 17-5 Timeout Period (valid values 0-11)
  //250 uS timout

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  while (WDT->STATUS.bit.SYNCBUSY == 1);

  WDT->CLEAR.reg= 0x00; // system reset via WDT if timer hasn't already expired
  while (WDT->STATUS.bit.SYNCBUSY == 1); 
}

// ===================================================================

// debounce reset button; not implemented; see systemReset()
void resetButton() {
  static unsigned int lastPress = millis();
  if(millis() - lastPress > 100) {

    if(millis() - lastPress < 1200){
      systemReset();
    }
    lastPress = millis();
  }
}

// ===================================================================

// Primary function for performing fast analog reads
// ADC is set up, a timer started, the coil activated then analog reads performed as quickly as possible
//
// LiveFire = true activates the coil as normal; false will run the ADC without switching the coil on (for testing sample rate)
//returns the time (in uS) to record <numReads> number of samples
int singleFireData(bool liveFire) {

  byte pin = currentSensorPin;

  // ADC configuration
  // Excellent info here: https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/

  ADC->CTRLA.bit.ENABLE = 0;              // disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // wait for synchronization

  analogReadResolution(readResolutionSetting); // ensure read resolution is set correctly
  ADC->CTRLB.reg &= 0b1111100011111111;          // mask PRESCALER bits
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV32;   // divide Clock by 32

  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_2 |   // take 2 samples
                     ADC_AVGCTRL_ADJRES(0x01ul); // adjusting result by 2; this gives us 2x averaging for each data point
  ADC->SAMPCTRL.reg = 0x00;  

  ADC->CTRLB.bit.CORREN = 0;

  if (pin < A0) {
    pin += A0;
  }

  // Set ADC to read the proper pin
  pinPeripheral(pin, PIO_ANALOG);
  while (DAC->STATUS.bit.SYNCBUSY == 1);
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;
  while (DAC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while (DAC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;
  while (ADC->INTFLAG.bit.RESRDY == 0);

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  //start the timer
  unsigned int now = micros();
  //turn the coil on
  digitalWrite(relayPin, liveFire);

  //record ADC vaules to the analogVals buffer as quickly as possible for each <numReads>
  for(int i=0; i<numReads; i++) {
    while (DAC->STATUS.bit.SYNCBUSY == 1);
    ADC->SWTRIG.bit.START = 1;
    while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  
    analogVals[i] = ADC->RESULT.reg;
  }

  //turn the coil off
  digitalWrite(relayPin,LOW);

  //return the total time taken to collect all the data points
  return (micros() - now);
}

// ===================================================================

//checks the resting output of the ACS712 sensor
void zeroADC() {
  const int samples = 100;
  int average = 0;

  for(int i=0; i<samples; i++) {
    average += analogRead(currentSensorPin);
  }

  average /= samples;
  adcZeroOffset = average - (ADCscale / 2);
}

// ===================================================================

// draws the chart axis, labels ect and scales the axis appropriately
// no data is plotted here
void drawChart(int sampleTime) {

  sampleTime /= numReads; // time for a single ADC read; generalyl ~10 µS
  const int XTickScale = 1000; // 1 millisecond ticks

  int yTickInterval = chartHeight / (maxmA / 1000); //1000 mA to 1 A
  int xTickInterval = XTickScale / sampleTime;

  // draw graphics using Adafruit GFX and RA8875
  tft.graphicsMode();


  //draws xAxis
  tft.drawFastHLine(xOrigin, yOrigin,   xPixels - (xOrigin + 5), 0x0000);
  tft.drawFastHLine(xOrigin, yOrigin+1, xPixels - (xOrigin + 5), 0x0000);

  //draws yAxis
  tft.drawFastVLine(xOrigin,     yOrigin, -chartHeight, 0x0000);
  tft.drawFastVLine(xOrigin - 1, yOrigin+1, -chartHeight, 0x0000);

  //y axis ticks
  for(byte i=1; i<5; i++) {
    tft.drawFastHLine(xOrigin, yOrigin - (i * yTickInterval),   xPixels - (xOrigin + 5), 0xe73c);
    tft.drawFastHLine(xOrigin, yOrigin - (i * yTickInterval) -1, xPixels - (xOrigin + 5), 0xe73c);
    
    tft.drawFastHLine(xOrigin - 3, yOrigin - (i * yTickInterval), 6, 0x0000);
    tft.drawFastHLine(xOrigin - 3, yOrigin - (i * yTickInterval) -1, 6, 0x0000);
  }

  //x axis ticks; ignores if sampleTime is out of range
  if(sampleTime < 30 && sampleTime > 5) {
    for(int i=xTickInterval; i<=chartWidth; i+=xTickInterval) {
      tft.drawFastVLine(xOrigin + (i),     yOrigin, -chartHeight, 0xe73c);
      tft.drawFastVLine(xOrigin + (i) +1, yOrigin+1, -chartHeight, 0xe73c);
      
      tft.drawFastVLine(xOrigin + (i), yOrigin -3, 6, 0x0000);
      tft.drawFastVLine(xOrigin + (i) +1, yOrigin -3, 6, 0x0000);
    }
  }

  // Draw box for printing data above the chart
  tft.fillRect(0, 0, xPixels-1, 35, 0xf7be);
  tft.drawFastHLine(0, 35, xPixels-1, 0xad55);

  //draw chart labels
  char strBuff[50]; // char array for manipulating text
  tft.textMode(); // textMode instead of graphics
  tft.setTextColor(0);
  tft.textEnlarge(0);
  tft.textSetCursor(10, 10);
  tft.textTransparent(0);


  // Draw y-axis labels
  for(byte i=1; i<5; i++) {
    tft.textSetCursor(xOrigin - 10, yOrigin - (i * yTickInterval));
    sprintf(strBuff, "%d", i);
    tft.textWrite(strBuff);
  }

  // draw x-axis labels
  if(sampleTime < 30 && sampleTime > 5) {
    for(int i=xTickInterval; i<=chartWidth; i+=xTickInterval) {
      tft.textSetCursor(xOrigin + (i) - 9, yOrigin); //-7 for character width (5) and space (2)
      sprintf(strBuff, "%d", i/xTickInterval);
      tft.textWrite(strBuff);
    }
  }

  // y-axis units
  tft.textSetCursor(xOrigin, yOrigin - chartHeight - 20); //-9 for character height (7) and space (2)
  tft.textWrite("Amps");

  // x-axis units
  tft.textSetCursor(xOrigin, yOrigin);
  tft.textWrite("mS");

}

// ===================================================================

// Plotting function used during multi-fire
// very simplistic (fast); pure black pixel for each data point plotted over the preceeding spark data
// <133 mS interval to record the spark, process the data, plot it on the display and prepare for the next spark
void quickPlot() {
  tft.graphicsMode();
  //simply plot a black pixel for each data point
  for(int i=0; i<numReads; i++) {
    tft.drawPixel(xOrigin+(i), yOrigin-analogVals[i], 0);
  }
}

// ===================================================================

// Plotting function for single-fire tests
// do a bit of anti-alasing + 'transparency' with a "+" shaped data point to make the single-fire test data look nicer; too slow for multi-fire tests
void plotData(int ttf, int maxCurrent, int sampleTime) {

  tft.graphicsMode();
  
  // Example 1:                     Example 2:                         Example 3:
  // a single data point looks      two data points nearby             pixels can reach a maximum of 7
  // like this when plotted:        overlap to appear like this:       which is 100% saturation:
  //  
  //  00100                         0010100                            0011100
  //  01210                         0122210                            0134310
  //  12321  (+)                    1244421   (+ +)                    1367631     (+++)
  //  01210                         0122210                            0134310
  //  00100                         0010100                            0011100
  //
  //

  // for loop cycles through each column of pixels on the chart and determines the fill color
  for(int x=2; x<numReads-2; x++) { //wraps a little at the limits but reading is generally zero there so this isn't a problem
    const byte weight[13] = {1,2,1,1,3,1,2,2,1,2,1,1,1}; // saturation intensity map of the pixels for one data point ("+" shape, example 1 above composed of 13 pixels)
    byte yVals[13]; // at most 13 pixels can be colored in column; the width of each data point is 5 and looking at the "+" shape the number of effected pixels is 1,2,3,2,1 from the current column and +-2 adjacent
                    // this array stores the yValue of the pixels which are shaded in the column
                    // we could just use an array of size chartHeight for simplicity but it would be wasteful of memory
    byte color[13] = {7,7,7,7,7,7,7,7,7,7,7,7,7}; //this array stores the shade of the pixel referenced in the array yVals

    //the y axis position of each shaded pixel in the column is stored in yVals (13 possible, usually there will be overlaps)
    yVals[0] = analogVals[x-2];

    yVals[1] = analogVals[x-1];
    yVals[2] = yVals[1] + 1;
    yVals[3] = yVals[1] - 1;

    yVals[4] = analogVals[x];
    yVals[5] = yVals[4] + 2;
    yVals[6] = yVals[4] + 1;
    yVals[7] = yVals[4] - 1;
    yVals[8] = yVals[4] + 2;

    yVals[9] = analogVals[x+1];
    yVals[10] = yVals[9] + 1;
    yVals[11] = yVals[9] - 1;

    yVals[12] = analogVals[x+2];


    color[0] -= weight[0];


    // now we go through and find overlaps; when two yVals are the same we add the values together and store the result in one of them (the darker)
    // the other is set to 0 (white) which is ignored in the plotting loop below
    for(byte i=0; i<13; i++){
        for(byte j=0; j<i; j++) {
            if(yVals[i] == yVals[j]) {
                color[i] = 7;
                color[j] >= weight[i] ? color[j] -= weight[i] : color[j] = 0;
            }
        }
    }


    // run through the array of shaded pixels in the column
    for(byte i=0; i<13; i++) {
        if(color[i] < 7 && yVals[i] <= 200) {
            // convert the 3 bit color depth to 16 bit color for the GFX library
            byte red   = map(color[i], 0, 7, 0, 31);
            byte blue  = 31; //map(color[i], 0, 7, 0, 31); // I chose to plot pixels in blue instead of black; purely stylistic
            byte green = map(color[i], 0, 7, 0, 63);
            int colorBlend = (red << 11) + (green << 5) + blue; // convert our color componnts to hicolor
            tft.drawPixel(x + xOrigin, yOrigin - yVals[i], colorBlend); //plot that
        }
    }
} // Finally close the loop that interates through each column of the chart

  // find the pixel location of the peak current
  int xPeak = xOrigin + ttf;
  int yPeak = yOrigin - maxCurrent;
  // circle it to make it stand out
  tft.drawCircle(xPeak, yPeak, 2, 0xF800);
  tft.drawCircle(xPeak, yPeak, 3, 0xF800);

  // _ PRINT STATISTICS _

  char strBuff[50]; // character manipulation buffer; larger than it needs to be

  //print the peak current and time data
  tft.textMode();
  tft.textSetCursor(10, 10);
  tft.textTransparent(0);

  tft.textWrite("time to fire = ");
  ttf *= (sampleTime/numReads); // ttf is the sample number corresponding to peak current this converts it to µS
  float output = ttf; // cast it to a float as design request was for reading in mS (not µS)
  output /= 1000;
  dtostrf(output,5, 3, strBuff); // convert float to string for GFX output
  tft.textWrite(strBuff);
  tft.textWrite(" ms    "); // extra spaces for lining up the text


  tft.textWrite("peak current = ");
  maxCurrent *= (maxmA / chartHeight); //maxCurrent is chart pixel position this converts it to mA
  output = maxCurrent; // cast it to a float as design request was for reading in A (not mA)
  output /= 1000;
  dtostrf(output, 5, 3, strBuff);
  tft.textWrite(strBuff);
  tft.textWrite(" A");

}

// ===================================================================

// Function to print the statistics for a multi-fire test on the display above the chart similarly to single fire test's plotData()
void multiFireStats(int sampleTime) {
  int meanTTF = 0; // average time to fire across all 50 sparks
  int meanCur = 0; // average peak current across all 50 sparks
  float devTTF = 0; // standard deviation of the time to fire across all 50 sparks
  int minTTF = 10000; // range of TTF across all 50 sparks; initallized to values outside of possible limits
  int maxTTF = 0;

  char strBuff[50]; // buffer for string manipulation

  //find the average time to fire and current as well as range of TTF
  for(int i=0; i<multiFireCount; i++) {
    meanTTF += multiFireData[i][0];
    meanCur += multiFireData[i][1];

    maxTTF = (multiFireData[i][0] > maxTTF) ? multiFireData[i][0] : maxTTF;
    minTTF = (multiFireData[i][0] < minTTF) ? multiFireData[i][0] : minTTF;
  }
  meanTTF /= multiFireCount;
  meanCur /= multiFireCount;


  //calculate the standard deviation of the time to fire
  for(int i=0; i<multiFireCount; i++) {
    devTTF += sq(float(multiFireData[i][0]) - meanTTF);
  }
  devTTF /= multiFireCount - 1;
  devTTF  = sqrt(devTTF);


  //Print out all the data:
  tft.textMode();
  tft.textSetCursor(10, 0);
  tft.textTransparent(0);

  tft.textWrite("avg time to fire = ");
  meanTTF *= (sampleTime/numReads); //mean TTF is in x-axis pixels, convert to µS
  float output = meanTTF; // cast to float, convert to mS and convert to string for GFX
  output /= 1000;
  dtostrf(output,5, 3, strBuff);
  tft.textWrite(strBuff);
  tft.textWrite(" ms  ");

  tft.textWrite("avg peak current = ");
  meanCur *= (maxmA / chartHeight); //convert current from pixel height to mA
  output = meanCur; // cast to float, convert to A and convert to string for GFX
  output /= 1000;
  dtostrf(output, 5, 3, strBuff);
  tft.textWrite(strBuff);
  tft.textWrite(" A");

  tft.textSetCursor(10, 18);
  tft.textWrite("         std dev = ");
  devTTF *= (sampleTime / numReads);
  devTTF /= 1000;
  dtostrf(devTTF, 5, 3, strBuff);
  tft.textWrite(strBuff);


  tft.textWrite(" ms   range = ");
  minTTF *= (sampleTime/numReads);
  output = minTTF;
  output /= 1000;
  dtostrf(output, 5, 3, strBuff);
  tft.textWrite(strBuff);
  tft.textWrite(" - ");
  maxTTF *= (sampleTime/numReads);
  output = maxTTF;
  output /= 1000;
  dtostrf(output, 5, 3, strBuff);
  tft.textWrite(strBuff);
  tft.textWrite(" ms");
}

// ===================================================================

// singleFireData only records raw analogReads
// this function converts the raw current measurements into pixel positions and stores it back in the same buffer
void processData(int testNum) {
  multiFireData[testNum][1] = 0; // reset max current reading to zero

  for(int i=0; i<numReads; i++) {
    //convert values to current
    //find ttf/max

    // convert the raw read into a pixel height on the chart; ACS712 has +-20,000 mA scale for 0-5 V output (2.5 volts = 0 current flow)
    int calcVar = analogVals[i];
    calcVar -= adcZeroOffset; // use zeroing data; this could be set up in ADC config but it's beyond the needs of this program
    calcVar -= ADCscale / 2; // fold the readings in half as we only care about absolute val of current readings
    calcVar = abs(calcVar); // flip negatives
    calcVar *= 625; 
    calcVar /= 64;
    calcVar *= chartHeight; //chart scaling; 4 amps to 200 pixels 4000ma / 200 pixels 
    calcVar /= maxmA;
    calcVar = (calcVar)<(0)?(0):((calcVar)>(chartHeight)?(0):(calcVar)); //constrain but >HIGH goes to LOW

    analogVals[i] = calcVar; //store that pixel height back in the analog reading buffer

    // find the maximum current which corresponds to the point when the coil fires
    // store these in the statistics buffer (used for mult and single)
    if ((analogVals[i] > multiFireData[testNum][1]) && (i>5)) {
      multiFireData[testNum][1] = analogVals[i]; //maxCurrent
      multiFireData[testNum][0] = i; //ttf
    }

  }
}

// ===================================================================

// Top level function which calls sub-routines for single fire tests
// fires the coil a single time and collects the coil current data
void singleFireTest() {
  int sampleTime;
  //fire the coil, dumping raw current readings into the analogVals buffer; returns the total time to collect all samples to sampleTime
  sampleTime = singleFireData(true);

  delay(10); // this delay was added because time isn't restrictive and I want to avoid any potential interference/noise from the coil

  //process all the raw data
  processData(0); //only one test -> use first position in multiFireData buffer

  //draw the chart axis etc and plot the coil current measurements over time
  tft.fillScreen(0xFFFF); // clear screen
  drawChart(sampleTime); // draw the chart axis etc and scale according to the ADC sampling time
  plotData(multiFireData[0][0], multiFireData[0][1], sampleTime); // plot the data
}

// ===================================================================

// Top level function which calls sub-routines for multi-fire tests
// fires the coil 50 times at a rate that approximates that of the engine running (~7.5 hz)
void multiFireTest() {

  // the first sample is used to scale the chart
  tft.fillScreen(0xFFFF);
  int sampleTime;
  unsigned int startTime = millis();
  sampleTime = singleFireData(true); // fire the coil, gathering raw current data
  delay(20); // short delay to avoid noise; possibly not needed
  processData(0); // process the data to usable form; store it in the first position of the data buffer
  drawChart(sampleTime); // draw the chart

  while(millis() - startTime < firePeriod); //wait until the total elapsed time since the coil fired is long enough to get the right timing (~133 mS)

  //continue firing the coil, collecting data and plotting it at ~7.5 hz until all the samples are collected
  for(int i=1; i<multiFireCount; i++) {
    startTime = millis();
    singleFireData(true);
    delay(20); // short delay to avoid noise; possibly not needed
    processData(i);
    quickPlot();
    while(millis() - startTime < firePeriod); //delays by firePeriod (133 ms) minus the charting/processing time
  }

  multiFireStats(sampleTime); // write the statistics
}

// ===================================================================

//displays data upon start of the program; called by setup();
void startupScreen() {
  tft.fillScreen(0xFFFF);
  const byte lineSpacing = 18; // pixel height of text line
  int textCursorY = 70; // Y position of text cursor
  const int variableColor = 0xB000;
  
  char strBuff[50]; // buffer for string manipulation

  //figure out how fast the ADC sampling is by running the coil firing routine
  int sampleTime = singleFireData(false); //perform a single fire test without activating the coil ("false")
  processData(0); //convert the raw readings to the data we want

  //display the zeroed current sensor position
  int averageRead = 0;
  for(int i=0; i<numReads; i++) {
    averageRead += analogVals[i];
  }
  averageRead /= numReads;
  averageRead *= maxmA;
  averageRead /= chartHeight;



  //text output telling the user about the system
  //provides some feedback on the status of the device

  tft.textMode();
  tft.setTextColor(0);
  tft.textTransparent(0);

  // Title
  tft.textEnlarge(1);
  tft.textSetCursor(2, 5);
  tft.textWrite("FACT:");
  tft.textSetCursor(90, 5);
  tft.textWrite("Ford/Arduino Coil Tester");

  // Title Variant
  tft.textEnlarge(0);
  tft.textSetCursor(10, 38);
  tft.textWrite("Trout variant v1.3");

  // Data points per spark (450)
  tft.textSetCursor(10, textCursorY);
  tft.textWrite("- ");
  sprintf(strBuff, "%d", numReads);
  tft.setTextColor(variableColor);
  tft.textWrite(strBuff);
  tft.setTextColor(0);
  tft.textWrite(" samples per fire");
  textCursorY += lineSpacing;

  // Data point stats
  tft.textSetCursor(10, textCursorY);
  tft.textWrite("- ");
  tft.textWrite("2x averaging per sample @ 12 bit resolution");
  textCursorY += lineSpacing;

  // Sample rate (usually 100 ksps; 10 µS per sample for 2x averaging at 12 bits)
  tft.textSetCursor(10, textCursorY);
  tft.textWrite("- ");
  sprintf(strBuff, "%d", sampleTime / numReads);
  tft.setTextColor(variableColor);
  tft.textWrite(strBuff);
  tft.setTextColor(0);
  tft.textWrite(" uS per sample (");
  sprintf(strBuff, "%d", 1000 / (sampleTime / numReads));
  tft.setTextColor(variableColor);
  tft.textWrite(strBuff);
  tft.setTextColor(0);
  tft.textWrite(" ksps)");
  textCursorY += lineSpacing;

  // ADC reading at rest
  tft.textSetCursor(10, textCursorY);
  tft.textWrite("- ");
  tft.textWrite("ADC reading average: ");
  sprintf(strBuff, "%d", averageRead);
  tft.setTextColor(variableColor);
  tft.textWrite(strBuff);
  tft.setTextColor(0);
  tft.textWrite(" mA");
  textCursorY += lineSpacing;

  // Number of 'single tests' in a multi-fire test (50)
  tft.textSetCursor(10, textCursorY);
  tft.textWrite("- ");
  sprintf(strBuff, "%d", multiFireCount);
  tft.setTextColor(variableColor);
  tft.textWrite(strBuff);
  tft.setTextColor(0);
  tft.textWrite(" tests per multi-fire");
  textCursorY += lineSpacing * 3;

  // Reports the reason the device reset (see Datasheet 15.8.14)
  tft.textSetCursor(10, textCursorY);
  tft.textWrite("- Reset code: ");
  byte resetCause = PM->RCAUSE.reg;
  sprintf(strBuff, "%d", resetCause);
  tft.textWrite(strBuff);

  // Describes function of buttons below display
  tft.textSetCursor(10, 250);
  tft.textWrite("Controls: |Single|  |Multi|");
}

// ===================================================================
// ===================================================================

void setup()
{
  //Serial.begin(9600);

  pinMode(relayPin, OUTPUT); // pin to switch SSR for coil power
  //pinMode(resetPin, INPUT_PULLUP); // unused reset button

  // Initalize the display
  if (!tft.begin(RA8875_480x272)) {
    //Serial.println("RA8875 Not Found!");
    while (1);
  }


  //turn on the display
  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX on RA8875 board
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft.PWM1out(255); // full brightness


  //set ADC reference (divider to 5v supply) and resolution (12bit)
  analogReference(AR_EXTERNAL);
  analogReadResolution(readResolutionSetting);

  zeroADC(); //zeros ADC to ACS712's resting output
  startupScreen(); //displays an initial screen to the user
}

// ===================================================================

void loop()
{
  int UIvolt = analogRead(userInput); // reads the userInput pin to see if any buttons were pressed

  if(UIvolt < ((ADCscale * 8) / 9)) { // did the pin get pulled down by button? (47k external pullup)
    
    delay(2); // wait 2 mS to debounce and settle
    UIvolt = analogRead(userInput); // read again

    if(UIvolt < (ADCscale / 3) ) { // if it was pulled near to ground then key #2 was pressed
      multiFireTest(); // run multiple tests in rapid succession
    }

    else if(UIvolt < ((ADCscale * 8) / 9)) { // if it was pulled to Vcc/2then key #1 was pressed
      singleFireTest(); // run a single test
    }

    // if the pin went back high during delay then ignore (debounce / EMI)
  }


}