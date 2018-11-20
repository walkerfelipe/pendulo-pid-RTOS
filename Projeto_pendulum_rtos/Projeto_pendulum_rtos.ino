#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal.h>
#include <semphr.h>
#include <PID_v1.h>
SemaphoreHandle_t xTuningsSemaphore;
double Setpoint, Input, Output;
int contador=0; // contador que auxilia calculo de setpoint variavel
float contf=0; // contador float para Seno
int i=0,ajust=0;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
// Filtragem de entrada, Quantidade de amostras
int amostragem=100;
double kp=0.7,ki=1.5,kd=0.2;
//PID myPID(&Input, &Output, &Setpoint,0.7,1.5,0.2 , DIRECT);
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd , DIRECT);
void TaskPID( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskDisplay( void *pvParameters );
// the setup function runs once when you press reset or power the board

int buttonState = 0; // salva estado do botão
const int buttonPin1 = 8; // botão de ajuste 
const int buttonPin2 = 10; // botão de ajuste 
const int buttonPin3 = 7; // botão de ajuste 
void setup() {
   pinMode(buttonPin1, INPUT_PULLUP);
   pinMode(buttonPin2, INPUT_PULLUP);
   pinMode(buttonPin3, INPUT_PULLUP);

  if ( xTuningsSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xTuningsSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xTuningsSemaphore ) != NULL )
      xSemaphoreGive( ( xTuningsSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  
  Input = analogRead(0);
   Setpoint=400;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  lcd.begin(16, 2);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
   display();
  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskPID
    ,  (const portCHAR *)"PID"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

    
  xTaskCreate(
    TaskDisplay
    ,  (const portCHAR *) "Display"
    ,  128  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}
void display(){
  if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
      lcd.clear();
    lcd.setCursor(0,0);
  switch (ajust) {
    case 0:{
    lcd.print("Kp< Ki  Kd  Sp  ");
    lcd.setCursor(0, 2);
    lcd.print(kp);
    break;
      }
    case 1:{
    lcd.print("Kp  Ki< Kd  Sp  ");
    lcd.setCursor(0, 2);
    lcd.print(ki);
      break;
     }
    case 2:{
    lcd.print("Kp  Ki  Kd< Sp  ");
    lcd.setCursor(0, 2);
    lcd.print(kd);
        break;
      }
    case 3:{
    lcd.print("Kp  Ki  Kd  Sp<  ");
    lcd.setCursor(0, 2);
    lcd.print(Setpoint);
        break;
      }
     
  }
  xSemaphoreGive( xTuningsSemaphore ); // Now free or "Give" the Serial Port for others.
}
}
void processa(int x){
  if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
  switch (ajust) {
  case 0:{
    // statements
    kp=kp+(x*0.01);
    myPID.SetTunings(kp, ki, kd);
    break;}
  case 1:{
    ki=ki+(x*0.01);
    // statements
    myPID.SetTunings(kp, ki, kd);
    break;}
  case 2:{
    kd=kd+(x*0.01);
    // statements
    myPID.SetTunings(kp, ki, kd);
    break;}
  case 3:{
    Setpoint=Setpoint+(x*5);
    // statements
    myPID.SetTunings(kp, ki, kd);
     }
   }  
  xSemaphoreGive( xTuningsSemaphore ); // Now free or "Give" the Serial Port for others.
  }
}
void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskPID(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  PID
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care 
  of use the correct LED pin whatever is the board used.
  
  The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
  the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
  e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.
  
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
  
  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
  
  modified 2 Sep 2016
  by Arturo Guadalupi
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
    myPID.Compute();
    analogWrite(9,Output);
    xSemaphoreGive( xTuningsSemaphore ); // Now free or "Give" the Serial Port for others.
  }
  vTaskDelay(10);
    //vTaskDelay( 1 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;)
  {
    // read the input on analog pin 0:
    double temp=0;
    for(i=0;i<amostragem;i++)
    temp += analogRead(A0);
    Input=temp/100;
    // print out the value you read:
    if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print("\n");
     xSemaphoreGive( xTuningsSemaphore ); // Now free or "Give" the Serial Port for others.
  }
  
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskDisplay(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for(;;){
       buttonState = digitalRead(buttonPin1);
    if (buttonState == LOW) {
     processa(1);
     display();
     
     vTaskDelay(10);
    }
    buttonState = digitalRead(buttonPin2);
    if (buttonState == LOW) {
     processa(-1);
     display();
     vTaskDelay(10);
    }
    buttonState = digitalRead(buttonPin3);
    if (buttonState == LOW) {
     if(ajust>=3)ajust=0;
     else 
      ajust++;
     display();
     vTaskDelay(10);
    }
   //  SetTunings(kp, ki, kd);
     
  
     // vTaskDelay( 1 / portTICK_PERIOD_MS ); 
    }
}
  

