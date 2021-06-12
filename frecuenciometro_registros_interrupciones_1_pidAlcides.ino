/********************************************************
* NOMBRE AOUTOR:  MANUEL ALFONSO MANJARRES RIVERA       *
*                                                       *
*                                                       *
*                                                       *
*********************************************************/


#include <EEPROM.h>
#include <avr/wdt.h>

/*DEFINICIONES DE LOS PINES*/
#define pinInterrupt 2
#define button1  4 
#define button2  5 
#define ledGreen 6
#define ledRed   7 
#define salidapwm 3

/*CONSTANTES SOMBOLICAS GENERALES*/
#define _MIL_ 1000.0
#define _VELOCIDAD_CRISTAL 16000000.0
#define _PREESCALER_CONSTAN  1024.0
#define time_ms 500 
#define memorilocation1 0 
#define memorilocation2 100 
#define FactorReduccion 0.1
#define set_memorilimMax 1
#define set_memorilimMin 2  


/*-----VARIABLES PARA EL CONTROL PID---------*/

 /*CAMBIAR HASTA AJUSTAR EL MODELO*/
#define kp 0.0 //constante proporcional
#define ki 0.0  //constante integral 
#define kd 0.0 // constante derivativa

#define set_point 60.0 //set point en 60hz
#define sample_time 1
#define max_ 255.0
#define min_ 0.0

float pid_int=0.0;   
float salida=0.0;
float error_=0.0;
float error=0.0; 
float ui_=0.0;
float ui=0.0;
float up=0.0; 
float ud=0.0; 
float ut=0.0;
/*----------------------------------------------------------*/
/*---------------------------------------------------------*/

/*---VARIABLES GLOBALES---------------------------------- */

 volatile unsigned long conteo=0;
 volatile unsigned long deltaT=0; 
 volatile unsigned long t0=0; 

 double FrecuenciaLimiteInferior=59.0; 
 double FrecuenciaLimiteSuperior=61.0;
 volatile double frecuencia=0.0; 
 float Data_1=0.0; 
 
 unsigned char counterInterrupts=0;  
 unsigned char flag=0;
 unsigned char flag_memori=0; /*pendiente por definir su funcion*/
/*---------------------------------*/

/*---PROTOTIPOS DE FUNCION------*/
  void PID();
  void Led_funtion(void);
  void set_memori(void);   
/*------------------------------*/

/*INICIO DE CONFIGURACIONES DEL MICROCONTROLADOR*/
void setup() {
    
    
    wdt_disable();
    wdt_enable( WDTO_2S );
    noInterrupts(); 
    Serial.begin(9600); 
  
/*----PIN MODE PARA LOS LED----------------------*/
    pinMode(ledRed,OUTPUT);
    digitalWrite(ledRed,LOW);
    pinMode(ledGreen,OUTPUT);
    digitalWrite(ledGreen,LOW);

/*----SALIDA DEL PWM---------------------------*/
    pinMode(salidapwm,OUTPUT);

/*----INTERRUPCION EXTERNA POR EL PIN 2------*/
    pinMode(pinInterrupt,INPUT); 
    attachInterrupt(digitalPinToInterrupt(pinInterrupt),ISR_EXTERNALPIN,RISING);  


/*******************************************************************************/
/*----CONFIGURACION DE LOS REGISTROS------------------------------------------*/
/*****************************************************************************/

/*----INTERRUPCIONES EXTERNAS POR CAMBIO DE NIVEL---------------------------*/      
    
    //DDRD |=(1<<2); 
    pinMode(button1,INPUT_PULLUP); 
    pinMode(button2,INPUT_PULLUP);
    PCMSK2 |=(1<<2)|(1<<3); //HABILITAMOS EL PCINT18 Y PCINT19
    PCICR |=(1<<2); 

/*----INTERRUPCION POR TIMER1------------------------------------------*/
     
    TCCR1A=0; //
    TCCR1B=0; 
    TCCR1B=(1<<CS10)|(1<<CS12);//PREESCALADOR A X1024
    TCNT1=0;    
    TIMSK1 |= (1 << TOIE1);  

   
   set_memori(); 
  interrupts(); 
   wdt_reset();//tiempo de reinicio 2 segundos 

    /*
    EEPROM.put(memorilocation1,0.0); 
    EEPROM.put(memorilocation2,0.0); 
    while(1);
   */
   Serial.print("iniciando\n ultimo valor en memoria-->");
   Serial.print(Data_1); 
   Serial.print("\n"); 
   Serial.print(FrecuenciaLimiteSuperior);
   Serial.print("\n"); 
   Serial.print(FrecuenciaLimiteInferior);
  
}/*FIN DE LA CONFIGURACION DEL MICROCONTROLADOR*/
/**************************************************************************************************************************************/
/*************************************************************************************************************************************/

void loop() {

    Serial.print("\nfrecuecia: ");
    Serial.println(frecuencia,3);
    Serial.print("\n conteo: ");
    Serial.print(conteo);
 
    if(flag==false){
        frecuencia=0.0;
          TCNT1=0;  
          conteo=0.0; 
        delay(100); 
      }
        flag=false; 
    
    led_funtion();
    PID();
     
    wdt_reset(); //reseteamosel timer del wd para que no reinicie el micro en caso de que todo vaya bien
    
    delay(time_ms);
    
}//----------


/*FUNCION DE INTERRUPCION EXTERNA*/
void ISR_EXTERNALPIN(){
    noInterrupts();
       conteo=TCNT1;  
       frecuencia=_VELOCIDAD_CRISTAL/(conteo*_PREESCALER_CONSTAN);    
       TCNT1=0; 
       flag=1; 
    interrupts(); 
  }/*FIN DE LA FUNCION DE INTERRUPCION EXTERNA*/


/*RUTINA DE INTERRUPCION PARA AUMENTAR O DISMINUIR LOS LIMITES DE FRECUENCIA PERMITIDOS*/
ISR (PCINT2_vect){

    if((PIND & (1<<button1))==0){
        
       
        if(FrecuenciaLimiteSuperior<70.0 && FrecuenciaLimiteInferior>50.0){
           
             Data_1+=FactorReduccion;   
             FrecuenciaLimiteSuperior+=FactorReduccion;
             FrecuenciaLimiteInferior-=FactorReduccion; 
              
            EEPROM.put(memorilocation1,Data_1);    
            if(flag_memori!=set_memorilimMax){
                flag_memori=set_memorilimMax;
                EEPROM.put(memorilocation2,flag_memori);   
                } 
                 while((PIND & (1<<button1))==0);
          }
         
          Serial.print("\n--AUMENTANDO LOS LIMITES PERMITIDOS--\n frecuencia superior: ");
          Serial.print(FrecuenciaLimiteSuperior);
          Serial.print("\nfrecuencia inferior: ");
          Serial.print(FrecuenciaLimiteInferior); 
          Serial.print("\n acumulado: ");
          Serial.print(Data_1); 
          Serial.print("\n");
      }//fin de interrupcion boton 1 

    
    if((PIND & (1<<button2))==0){
             
        if(FrecuenciaLimiteSuperior>60.0 && FrecuenciaLimiteInferior<60.0){
             Data_1-=FactorReduccion ;
             FrecuenciaLimiteSuperior-=FactorReduccion ;
             FrecuenciaLimiteInferior+=FactorReduccion ; 
  
            EEPROM.put(memorilocation1,Data_1);  
          
            if(flag_memori!=set_memorilimMin){
                flag_memori=set_memorilimMin;
                EEPROM.put(memorilocation2,flag_memori);   
                }
                 while((PIND & (1<<button2))==0);
          }
                
          Serial.print("\n--DISMINUYENDO LOS LIMITES PERMITIDOS--\n frecuencia superior: ");
          Serial.print(FrecuenciaLimiteSuperior);
          Serial.print("\nfrecuencia inferior: ");
          Serial.print(FrecuenciaLimiteInferior); 
          Serial.print("\n acumulado: ");
          Serial.print(Data_1); 
          Serial.print("\n");
    }//fin de interrupcion por el boton 2 
  
  }
/*******************************************************************************************************/


/*ENCIENDE UN LED INDICADOR, PARA SABER VISUALMENTE SI ESTAMOS DENTRO DE LOS LIMITES PERMITIDOS*/
void led_funtion(void){
    
    if(frecuencia<FrecuenciaLimiteSuperior && frecuencia>FrecuenciaLimiteInferior){
        digitalWrite(ledGreen,HIGH); 
        digitalWrite(ledRed,LOW); 
      }
    else {
        digitalWrite(ledGreen,LOW); 
        digitalWrite(ledRed,HIGH); 
        } 

  }
/********************************************************************************************/

/*INICIA EL SISTEMA CON LAS VARIABLES CORRESPONDIENTES A LAS CONFIGURACIONES DE LOS LIMITES*/
void set_memori(void){

    EEPROM.get(memorilocation2,flag_memori); 

    if(flag_memori==set_memorilimMax ){
        EEPROM.get(memorilocation1,Data_1);
           FrecuenciaLimiteSuperior+=Data_1;
           FrecuenciaLimiteInferior-=Data_1; 
      return; 
    }
    
    if(flag_memori==set_memorilimMin){
        EEPROM.get(memorilocation1,Data_1);
         FrecuenciaLimiteSuperior-=Data_1;
         FrecuenciaLimiteInferior+=Data_1; 
      return; 
    }
}
/******************************************************************************/



void PID(){
      noInterrupts();
      pid_int=frecuencia; 
      //error=set_point-(frecuencia); 
      error=frecuencia-set_point ;
      up=kp*error;
      ui=ui_+ (ki*sample_time*error); 
      ud=(float)kd*(error-error_)/ (float)sample_time;
      ut=up+ui+ud; 

    if(ut>max_)ut=max_;
      else if (ut<min_)ut=min_;
    
      ui_=ui;
      error_=error; 
      analogWrite(salidapwm,ut); 
      Serial.print("\n salida pwm->");
      Serial.println(ut) ;
      Serial.print("\n");
      interrupts();
  }


  
