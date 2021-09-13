/*
Project: TESTPWM
Date:    September 2019

Copyright (c) 2019- AJ vd Werken

Permission is hereby granted, free of charge, to any person obtaining a copyfvalor
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <Wire.h> //INCLUSÃO DA BIBLIOTECA
#include "RTClib.h" //INCLUSÃO DA BIBLIOTECA
#include <LiquidCrystal.h>
#include "ade7753.h"// biblioteca ade7753_ufersa
#include <SPI.h> 
#include <PinChangeInt.h>
#include <ArduinoJson.h>
#include <TimerFour.h>

RTC_DS3231 rtc; //OBJETO DO TIPO RTC_DS3231
LiquidCrystal lcd(23, 25, 27, 29, 31, 33);
//RS, EN, D4, D5, D6, D7 

static const byte analogPin = A0;
static const byte analogPin7 = A7;// debug
const byte averageCount = 200;// Let's compare an average of 100
int pinpulsein = 2; // Pino de leitua do PWM. Utiliza IOC. 
float Vout;
float Vout1;
int amps = 6;
int STATE;

float Vlinha = 0;
float Vlinha_serial = 0;
float Ilinha = 0;
float Ilinha_serial = 0;
float Wlinha = 0;
float Wlinha2 = 0;
float Wlinha3 = 0;
float phi=0;
int modemet = 0;
int rele1 = 43;
int rele2 = 45;

// Variables for the Modified Moving Average
float movingAverage;
float movingAverageSum;
float frequency;
float durationH;
float durationL;
float Period;
float Duty;
float InverseDuty; 
String str;

// Timer variables
unsigned long previousMillis = 0;
unsigned int printInterval = 500;
unsigned long startTime = 0;
unsigned long stopTime = 0;

unsigned long tempo_inicio;
 
unsigned long tempo_fim;
 
unsigned long valor = 0;
unsigned long valor1 = 0;
unsigned long valor_soma = 0;
unsigned long valor_final = 0;

float valor_ajustado;

#define FASTADC 1 // daqui pra frente configura o AD
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile int pwm_value = 0;
volatile int prev_time = 0;
uint8_t latest_interrupted_pin;
 
void RisingEdge()
{
  if (digitalRead(pinpulsein)==HIGH)
    {
      pwm_value = micros()-prev_time;
      valor1= analogRead (7);  
    }
  else
    {
      prev_time = micros();
    }
}
 


//DECLARAÇÃO DOS DIAS DA SEMANA
char daysOfTheWeek[7][12] = {"Domingo", "Segunda", "Terça", "Quarta", "Quinta", "Sexta", "Sábado"};
int T_inicio;
int Delta;
 


void pwm(){
  uint8_t oldSREG = SREG;
  cli();
  

  pinMode(12,OUTPUT);
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); //| _BV(COM1A0) 
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  //OCR1A = 249;
 OCR1A = 252;
  // 10% = 28 , 96% = 239
  OCR1B = 130; // pwm on port 12

  SREG = oldSREG;

  //pinMode(2, OUTPUT);
  //pinMode(3, OUTPUT);
   
  //digitalWrite( 2, LOW );
  //digitalWrite( 3, HIGH );
}

void pilot_measurement(){
   

   if (valor1 > 900) { // Estado A CP=12V - desconectado
        STATE = 1;
      
    }
       
    if ((valor1 >= 760) && (valor1 <= 830)) { // Estado B PWM ON
    // if (valor <= 850) { // Estado B PWM ON - Conectado
        STATE = 2; 
      
        
    }
    if ((valor1 >= 700) && (valor1 <= 740)) { // Estado C charging
        STATE = 3;
      }

     if ((valor1 >= 600) && (valor1 <= 650)) { // Estado D charging vent
        STATE = 4; 
    }
       if (valor1 < 550) { // Estado E e F Error
        STATE = 5; 
    }
    
   
}

void callback(){ // envia dados pela serial 1

  StaticJsonDocument<500> doc;
  doc["Status"] = STATE;
  doc["Mains Voltage"] = Vlinha_serial;
  doc["Current"] = Ilinha_serial;
  doc["Power Consumption"] = Wlinha3;

  
  //doc["Status"] = 1;
  //doc["Mains Voltage"] = Vlinha_serial;
  //doc["Current"] = Ilinha_serial;
  //doc["Power Consumption"] = Wlinha3;

  serializeJson(doc, Serial1);
  //Serial.println ("rotina de 2 segundo");
  
}



void hora (){
  DateTime now = rtc.now();
         
              if(now.hour()<10){
              //Serial.print("0");//IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print("0");
              //Serial.print(now.hour(), DEC); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(now.hour(), DEC); //
              }
         
              else{
              //Serial.print(now.hour(), DEC); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(now.hour(), DEC); //
              }
         
              //Serial.print(':'); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(':'); //
    
              if(now.minute()<10){
              //Serial.print('0'); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print('0');
              //Serial.print(now.minute(), DEC); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(now.minute(), DEC); //
              }
         
              else{
              //Serial.print(now.minute(), DEC); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(now.minute(), DEC); //
              }
         
              //Serial.print(':'); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(':'); //
         
              if(now.second()<10){
              //Serial.print('0'); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print('0');
              //Serial.print(now.second(), DEC); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(now.second(), DEC); //     
              }
              else{
              //Serial.print(now.second(), DEC); //IMPRIME O CARACTERE NO MONITOR SERIAL
              lcd.print(now.second(), DEC); //
              }
}


void setup(){
  //lcd.begin(L,C)
  attachInterrupt(digitalPinToInterrupt(pinpulsein),RisingEdge , CHANGE);
  Timer4.initialize(2000000); // Inicializa o Timer1 e configura para um período de 0,1 segundos
  Timer4.attachInterrupt(callback); // Configura a função callback() como a função para ser chamada a cada interrupção do Timer1

  
  lcd.begin(2, 16);
  Serial.begin(9600); //INICIALIZA A SERIAL
  Serial1.begin(9600);//INICIALIZA A SERIAL 1
  if(! rtc.begin()) { // SE O RTC NÃO FOR INICIALIZADO, FAZ
    //Serial.println("DS3231 não encontrado"); //IMPRIME O TEXTO NO MONITOR SERIAL
    Serial.println("RTC Inoperante"); //IMPRIME O TEXTO NO MONITOR SERIAL
    lcd.print("RTC Inoperante");
    while(1); //SEMPRE ENTRE NO LOOP
  }
  if(rtc.lostPower()){ //SE RTC FOI LIGADO PELA PRIMEIRA VEZ / FICOU SEM ENERGIA / ESGOTOU A BATERIA, FAZ
    Serial.println("DS3231 OK!"); //IMPRIME O TEXTO NO MONITOR SERIAL
    lcd.print("DS3231 OK!");
    //REMOVA O COMENTÁRIO DE UMA DAS LINHAS ABAIXO PARA INSERIR AS INFORMAÇÕES ATUALIZADAS EM SEU RTC
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //CAPTURA A DATA E HORA EM QUE O SKETCH É COMPILADO
    //rtc.adjust(DateTime(2018, 9, 29, 15, 00, 45)); //(ANO), (MÊS), (DIA), (HORA), (MINUTOS), (SEGUNDOS)
  }
  delay(100); //INTERVALO DE 100 MILISSEGUNDOS

// SPI Init
  SPI.setDataMode(SPI_MODE2);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  pinMode(53,OUTPUT);
  digitalWrite(53, HIGH);
  DateTime now = rtc.now();
  T_inicio = now.unixtime();

  
ADE7753 meter;      // Instantiate class ADE7753 to "meter"
meter.getActiveEnergyReset();
meter.getApparentEnergyReset();

// configura o preescaler do ADC
 //ADCSRA &= ~PS_128;  //limpa configuração da biblioteca do arduino
 
 // valores possiveis de prescaler só deixar a linha com prescaler desejado
 // PS_16, PS_32, PS_64 or PS_128
 //ADCSRA |= PS_128; // 64 prescaler
 //   ADCSRA |= PS_64; // 64 prescaler
//ADCSRA |= PS_32; // 32 prescaler
// ADCSRA |= PS_16; // 16 prescaler

#if FASTADC  // daqui pra frente configura o AD
 // set prescale to 16
 sbi(ADCSRA,ADPS2) ;
 cbi(ADCSRA,ADPS1) ;
 cbi(ADCSRA,ADPS0) ;
#endif

  pinMode(rele1,OUTPUT);
  pinMode(rele2,OUTPUT);

  
}

void loop(){
    /*do{
    digitalWrite( 12, HIGH );// liga o sinal piloto
    valor = analogRead (0);
    //valor1 = analogRead (7);
    //Serial.print("Valor do AD0:");
    //Serial.print(valor);
    //Serial.print(" | ");
    //Serial.println(" dentro laço while");  
    //Serial.print("Valor do AD7:");
    //Serial.println(valor1);
    //lcd.clear(); 
    lcd.setCursor(0, 0);
    lcd.print("Desconectado");
    lcd.setCursor(0, 1);
    //lcd.print (valor);
    hora();
        }while (valor >=900);*/
  ADE7753 meter;
  meter.setMode(0x88);

  //Configuracion
   meter.analogSetup(GAIN_1,0,   0,0,   0,0);
  //meter.analogSetup(0, 0,   0, 0,   0, 0);
  //meter.rmsSetup(0,1000);
  //meter.rmsSetup(-2048, -1450);
  meter.rmsSetup(0, 0);
  //meter.getActiveEnergyReset();
  //meter.frecuencySetup(0, 1231);
  //  meter.energySetup(417,11,17,300,11,0);  //Energia en Joules
  //meter.energySetup(417, 40, 17, 300, 11, 0); //Energia en Wh
 meter.energySetup(0, 0, 0, 0, 0, 0);


        
    lcd.clear();    
    valor = analogRead (0);
    while (valor >=900){
    digitalWrite( 12, HIGH );// liga o sinal piloto
    //valor1 = analogRead (7);
    //Serial.print("Valor do AD0:");
    //_06022021Serial.println(valor);
    //Serial.print(" | ");
    //Serial.println(" dentro laço while");  
    //Serial.print("Valor do AD7:");
    //Serial.println(valor1);
    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Desconectado");
    lcd.setCursor(0, 1);
    STATE=1;
    
    //Serial1.println((STATE));//Envia State para a serial 1
    //lcd.print (valor);
    hora();
    Vlinha = meter.vrms();
    Ilinha = meter.irms();
    Wlinha = meter.getApparentEnergy();
    Wlinha2 = meter.getActiveEnergy();
        phi = (Wlinha2/Wlinha);
    lcd.setCursor(9, 1);
    lcd.print(Vlinha/3944);
    lcd.print(" V ");
    Vlinha_serial = (Vlinha/3944);
    Ilinha_serial = (Ilinha/9618);//96189);
    Wlinha3 = (Vlinha_serial*Ilinha_serial*phi);
    Serial.print(Vlinha_serial);//Envia State para a serial 1
    Serial.print(" | ");
    //Serial.print(Ilinha_serial);//Envia State para a serial 1
    Serial.print(" | ");
    Serial.print(Wlinha);//Envia State para a serial 1
    Serial.print(" | ");
    Serial.print(Wlinha2);//Envia State para a serial 1
    Serial.print(" | ");
    Serial.print(phi);//Envia State para a serial 1
    Serial.print(" | ");
    Serial.print(Vlinha_serial*Ilinha_serial);
    Serial.print(" | ");
    Serial.print(Wlinha3);//Envia State para a serial 1
    Serial.print(" | ");
    modemet = meter.getMode();
    Serial.println (modemet);
    valor = analogRead (0);
    }
  
    //Serial.println ("Saída do laço");
    pwm();
    //Serial.print ("valor PWM: ");
    //Serial.print (pwm_value);
    //Serial.print (" | ");
    pilot_measurement();
    //Serial.print ("Valor Ad7:");
    //Serial.println (valor1);
    Serial.println (STATE);
    str=String(STATE);
    //Serial1.println(str);//Envia State para a serial 1
    //Serial1.println(STATE);//Envia State para a serial 1
    
   switch (STATE) {
            case 1:
            //lcd.clear(); 
            lcd.setCursor(0, 0);
            lcd.print("Desconectado");
            lcd.setCursor(0, 1);
            hora();
            digitalWrite(rele1,LOW);   // desliga rele1
            digitalWrite(rele2,LOW);   // desliga rele2                           
            
            break;
           
            case 2:
            //lcd.clear(); 
            lcd.setCursor(0, 0);
            lcd.print("Conectado");
            digitalWrite(rele1,LOW);   // desliga rele1
            digitalWrite(rele2,LOW);   // desliga rele2
            lcd.setCursor(0, 1);
            lcd.print("Valor PWM:");
            lcd.print(pwm_value);
               
            break;
             
            case 3:
            //lcd.clear(); 
            lcd.setCursor(0, 0);
            lcd.print("Carregando");
            lcd.print(" ");
            lcd.print("P:");
            lcd.print(pwm_value);
            digitalWrite(rele1,HIGH);   // liga rele1
            digitalWrite(rele2,HIGH);   // liga rele2
            
            lcd.setCursor(0, 1);
                        
            Vlinha = meter.vrms();
            Ilinha = meter.irms();
            //Wlinha = meter.getActivePower();
            Wlinha = ((Vlinha/3944)*(Ilinha/96189));
            lcd.print(Ilinha/9618); //(Ilinha/96189);
            lcd.print("A");
            lcd.setCursor(6, 1);
            lcd.print(Wlinha);
            lcd.print("W");

    //consumo_actual=meter.getActiveEnergy();
    //aparente_nueva=meter.getApparentEnergy();
    //Serial.print("Consumo [W]: ");
    //Serial.println(consumo_actual-consumo_previo,DEC);
    //Serial.print("Consumo aparente [VA]: ");
    //Serial.println(aparente_nueva-aparente_vieja,DEC);

    //Serial.print("Energia act: ");
    //Serial.print(consumo_actual,DEC);
    //Serial.print(" | ");
    //Serial.println(meter.getActiveEnergy(),DEC);
    //consumo_previo=consumo_actual;
    //aparente_vieja=aparente_nueva;
    //Serial.print("Energia apa: ");
    //  Serial.print(meter.getApparentEnergy(),DEC);
    //Serial.print(" | ");
    //Serial.println(meter.getApparentEnergy(),DEC);

    // Serial.print("Temperatura: ");
    //Serial.print(meter.getTemp(),DEC);
    //Serial.print(" | ");
    //Serial.print(meter.getTemp(),DEC);
    //Serial.print(" | ");
    //Serial.print(meter.getTemp(),DEC);
    //Serial.print(" | ");
    //Serial.println(meter.getTemp(),DEC);
    //Serial.print(" | ");
    //Serial.print (meter.getCurrentOffset(), DEC);
    //Serial.print(" | ");
    //Serial.print("mode: ");
    //Serial.println(  meter.getMode(), DEC);



            
            break;
 
            case 4: 
            //lcd.clear(); 
            lcd.setCursor(0, 0);
            lcd.print("Carregando - Vent");
            digitalWrite(rele1,HIGH);   // liga rele1
            digitalWrite(rele2,HIGH);   // liga rele2
            
            break;

            case 5: 
            //lcd.setCursor(0, 0);
            lcd.print("Erro Sinal Piloto");
            digitalWrite(rele1,LOW);   // desliga rele1
            digitalWrite(rele2,LOW);   // desliga rele2
            //Serial.println(valor_final);
            
            break;
    }






  
    
    //pwm();
     //pilot_();
     //Serial.print("Valor do PWM:");
     //Serial.println(pwm_value);
     
}

/*
 void pwm_measurement (){
    /*durationH = pulseIn(pinpulsein, HIGH);
    //Serial.println(durationH);
     durationL = pulseIn(pinpulsein, LOW);
    //Serial.println(durationL);
    Period=(durationH+durationL);
    //Serial.println(Period);
    //Serial.print(" us");
    frequency = (1000000/Period);
    //Serial.println(frequency);
    //InverseDuty = ((Period)/(durationH));
    Duty=(durationH)/(Period);
    //Duty = (durationH);
    InverseDuty = 1/Duty;
    }
 */

 /*
  startTime = micros();
  // Remove previous movingAverage from the sum
  movingAverageSum = movingAverageSum - movingAverage;
   
  movingAverageSum = movingAverageSum + currentValue;
  // Recalculate movingAverage

  movingAverage = movingAverageSum / averageCount;
 
  stopTime = micros();
 
  if (millis() - previousMillis >= printInterval) {
    Serial.print(F("currentValue: "));
    Serial.println(currentValue);
    //Serial.print(F("PWM Value: "));
    //Serial.println(PWMvalue);
    Serial.print(F("Moving Average: "));
    Serial.println(movingAverage);
    Serial.print("Calculation time: ");
    Serial.print(stopTime - startTime);
    Serial.println(" us");
    Serial.println();
    Vout1 = ((movingAverage*4.84)/1000);
    Serial.println(Vout1);
    Serial.print(" V");  
    Serial.flush();
 
    // reset the millis clock
    previousMillis = millis();
      }*/

   /*void PulseChange(){

}*/
/*
void FallingEdge() 
{
 
}*/
