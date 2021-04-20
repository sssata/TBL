#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <AbsMouse.h>

Adafruit_ADS1015 ads1015;

int POTA_PIN = 1;
int POTB_PIN = 2;
int ACTIVATE_PIN = 9;

float valueA = 0;
float valueB = 0;

float prevValueA = 0;
float prevValueB = 0;

float expFilterWeight = 0.4;
float expFilterWeightXY = 0.2;

float angleA = 0;
float angleB = 0;

float l1 = 60;
float l2 = 60;
float x_raw = 0;
float y_raw = 0;

float x_rawPrev = 0;
float y_rawPrev = 0;

int A_0 = 3197;
int A_90 = 1254;
int B_90 = 1982;
int B_180 = 3887;

int pollRate_hz = 600;
int period_uS = (1000*1000)/pollRate_hz;
unsigned long lastRunTime_uS = 0;
unsigned long currentTime_uS = 0;

int screenWidth = 1920;
int screenHeight = 1080;

float x_origin = 40;
float x_width = 67.7;
float x_max = x_origin + x_width;
float y_height = screenHeight*1.0/screenWidth*x_width;
float y_origin = y_height/2;
float y_max = y_origin - y_height;

int ADCSamples = 9;

volatile int activateMouse = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(POTA_PIN,INPUT);
  pinMode(POTB_PIN,INPUT);
  pinMode(ACTIVATE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ACTIVATE_PIN), toggleMouseActivate, FALLING);
  Serial.begin(115200);
  
  ads1015.setGain(GAIN_ONE);
  ads1015.setDataRate(RATE_ADS1015_3300SPS);
  ads1015.begin();
  analogReadResolution(12);
  analogReference(AR_DEFAULT);

  AbsMouse.init(screenWidth, screenHeight);
}

void loop() {
  
  currentTime_uS = micros();

  if (currentTime_uS - lastRunTime_uS > period_uS){
    // put your main code here, to run repeatedly:

    lastRunTime_uS = micros();
    
    valueA = analogSample(POTA_PIN,ADCSamples);
    valueB = analogSample(POTB_PIN,ADCSamples);

    //valueA = digitalSample(0,1);
    //valueB = digitalSample(2,1);

    //Serial.print(valueA); Serial.print(" "); Serial.print(valueB); Serial.print(" ");

    //valueA = ads1015.readADC_SingleEnded(0);
    //valueB = ads1015.readADC_SingleEnded(2);

    //valueA = ads1015.readADC_Differential_0_1();
    //valueB = ads1015.readADC_Differential_2_3();

    //Serial.print(valueA); Serial.print(" "); Serial.println(valueB);

    
    
    valueA = expFilter(valueA, prevValueA, expFilterWeight);
    valueB = expFilter(valueB, prevValueB, expFilterWeight);

    //Serial.print(valueA); Serial.print(" "); Serial.println(valueB);

    prevValueA = valueA;
    prevValueB = valueB;

    angleA = mapf(valueA, A_0, A_90, 0,90);
    angleB = mapf(valueB, B_90, B_180, 90,180);
    
    float angleB2 = -(180 - angleA - angleB);

    x_raw = l1*cos(angleA*PI/180) + l2*cos(angleB2*PI/180);
    y_raw = l1*sin(angleA*PI/180) + l2*sin(angleB2*PI/180);

    x_raw = expFilter(x_raw, x_rawPrev, expFilterWeightXY);
    y_raw = expFilter(y_raw, y_rawPrev, expFilterWeightXY);

    x_rawPrev = x_raw;
    y_rawPrev = y_raw;

    float x_mapped = constrain(mapf(x_raw,x_origin,x_max,0.0,screenWidth),0,screenWidth);
    float y_mapped = constrain(mapf(y_raw,y_origin,y_max,0.0,screenHeight),0,screenHeight);

    //Serial.print(" A raw ADC: ");Serial.print(valueA);
    //Serial.print(" B raw ADC: ");Serial.print(valueB);

    //Serial.print(valueA); Serial.print(" "); Serial.println(valueB);
    
    //Serial.print(" A angle: ");Serial.print(angleA);
    //Serial.print(" B angle: ");Serial.print(angleB);

    //Serial.print(" x: ");Serial.print(x_raw);
    //Serial.print(" y: ");Serial.print(y_raw);
    
    //Serial.print(" x mapped: ");Serial.print(x_mapped);
    //Serial.print(" y mapped: ");Serial.println(y_mapped);

    //Serial.println("");

    if (activateMouse == 1){
      AbsMouse.move((int)x_mapped, (int)y_mapped);
    }
    
  }
}

void toggleMouseActivate(){
  noInterrupts();
  if (activateMouse == 0){
    activateMouse = 1;
  }else{
    activateMouse = 0;
  }
  interrupts();
}

float digitalSample(int channel, int samples){

  int sum = 0;
  
  for(int i = 0; i < samples; i++){
    sum = sum + ads1015.readADC_SingleEnded(channel);
  }
  
  return ((sum*1.0)/samples);
}

float analogSample(int pin, int samples){

  int sum = 0;

  int sample[samples];
  
  for(int i = 0; i < samples; i++){
    sample[i] = analogRead(pin);
  }

  qsort(sample,samples,sizeof(sample[0]), sort_desc);

  for(int i = 0; i < samples; i++){
    //Serial.print(sample[i]); Serial.print(" ");
  }
  //Serial.println("");

  int startIndex = 1;
  int endIndex = samples-1;
  
  for (int i = startIndex; i < endIndex; i++){
    sum += sample[i];
  }
  
  return ((sum*1.0)/(endIndex - startIndex));
}

float expFilter(float current, float prev, float weight){
  return prev*(1-weight) + current*(weight);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}
