#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal.h>
#include <semphr.h>
#include <PID_v1.h>
SemaphoreHandle_t xTuningsSemaphore;
double Setpoint, Input, Output;
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

int buttonState = 0; // salva estado do botão
const int buttonPin1 = 8; // botão de seleção
const int buttonPin2 = 10; // botão de ajuste - 
const int buttonPin3 = 7; // botão de ajuste +
void setup() {
   pinMode(buttonPin1, INPUT_PULLUP);
   pinMode(buttonPin2, INPUT_PULLUP);
   pinMode(buttonPin3, INPUT_PULLUP);

  if ( xTuningsSemaphore == NULL )  // Verificando se o Semaforo foi criado
  {  
    xTuningsSemaphore = xSemaphoreCreateMutex();  // Criando um semaforo mutex para controle do recurso
    if ( ( xTuningsSemaphore ) != NULL )
      xSemaphoreGive( ( xTuningsSemaphore ) ); 
  }

  
  Input = analogRead(0);
   Setpoint=400;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  lcd.begin(16, 2);
  // Inicializando comunicação serial 9600 bits por segundo:
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

  
}
void display(){
  if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 200 ) == pdTRUE ){
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
 if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 200 ) == pdTRUE ){
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

// Agora, o agendador de tarefas, que assume o controle do agendamento de tarefas individuais,
  //é iniciado automaticamente.
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

  for (;;) // Uma Tarefa nunca tera um return ou sairá.
  {
    // verifica o estado do semaforo
    if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 200 ) == pdTRUE ){
    myPID.Compute(); //calculo pid
    analogWrite(9,Output);
    xSemaphoreGive( xTuningsSemaphore ); // Libera o semafro 
  }
  vTaskDelay(10);
  
  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    // Leitura analogica pin 0:
    double temp=0;
    for(i=0;i<amostragem;i++)
    temp += analogRead(A0);
    Input=temp/100;
    // Imprimindo saidas
    //verifica semaforo
    if ( xSemaphoreTake( xTuningsSemaphore, ( TickType_t ) 200 ) == pdTRUE ){
    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print("\n");
     xSemaphoreGive( xTuningsSemaphore ); // libera semaforo
  }
  
    vTaskDelay(1);  // delay (15ms) entre as leituras para estabilidade
  }
}

void TaskDisplay(void *pvParameters)  // display
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
  
     

     // vTaskDelay( 1 / portTICK_PERIOD_MS ); 
    }
}
  

