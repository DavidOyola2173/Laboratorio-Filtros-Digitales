/*Codigo Motor-Generador Dc Filtros:
 * Autores: David Santiago Oyola Lozano
 * Diego Fernando Blanco Rueda
 * Heiner Camilo Varela
*/

//DEFINICION DE PINES DE LOS COMPONENTES:-----------------------------------------------------------------------------------------------------

//Encoder:
#define encoder 2

//Puente H:
int ENA = 11;
int IN1 = 8;
int IN2 = 9;

//DEFINICION DE VARIABLES:--------------------------------------------------------------------------------------------------------------------
//Lectura Analogica del Encoder:
volatile unsigned muestreoactualinterrupcion=0; //El tiempo actual del muestreo de la interrupcion.
volatile unsigned muestreoanteriorinterrupcion=0; //El tiempo anterior al tiempo actual del muestreo de interrupcion.
volatile unsigned deltamuestreointerrupcion=0;  //El diferencial de tiempo entre el tiempo anterior y actual de muestreo de interrupcion.

//Variables del calculo de velocidad angular:
int ciclo=0;  //Contador
double frecuencia=0;  //La frecuencia de interrupcion calculada por el promedio de deltas de tiempo.
double w=0; //Velocidad angular calculada del encoder.

//Variables para tiempo de muestro:
float  tiempo1=0;
float  tiempo2=0;
float  tiempo=0;
double n=1; //Numero de interrupciones (o ranuras) del medidor del encoder.

//Vector de suma promnedio:
float vector[]={0,0,0,0,0,0,0,0,0,0}; //Vector donde se guarda cada delta de tiempo para hacer un promedio.
int tam=10; //Tamaño de evaluacion de los ciclos.

//Parametros de control de PWM:                        
int  PWM=0;  //PWM que se controla.
float lecturaAnalogica =0; //Lectura analogica motor.
float voltajeSalida = 0; //Voltaje Generador.

//Parametros Filtros:
double yn_volt2, yn_volt4;

double Voyn4, Voyn3, Voyn2, Voyn1;
double Voxn4, Voxn3, Voxn2, Voxn1;

double yn_vel2, yn_vel4;

double Veyn4, Veyn3, Veyn2, Veyn1;
double Vexn4, Vexn3, Vexn2, Vexn1;

//INICIO DEL CODIGO:-------------------------------------------------------------------------------------------------------------------------
//Funcion del encoder:
void encoderVoid(){ //Esta parte del codigo entra a funcionar cuando sucede una interrupcion en el encoder
ciclo++;  //Inicia el tick.

//En esta parte se mide el diferencial de tiempo entre tick y tick medido por el encoder para sacar un promedio de tiempo:
if(ciclo==1){
      float media=0;  //Se inicializa la variable para obtener el promedio de los delta de tiempos
      deltamuestreointerrupcion=muestreoactualinterrupcion-muestreoanteriorinterrupcion;//Se hace el diferencial de tiempo de interrupcion de ticks del encoder.
      for(int i=0;i<tam-1;i++){ //Ciclo for para crear un vector de n posiciones.
        vector[i]=vector[i+1];  //Este vector se utiliza para almacenar los delta de tiempos.
      }
      vector[tam-1]=deltamuestreointerrupcion;
      for(int i=0;i<tam;i++){ //Ciclo for para comenzar a hacer el promedio de tiempos de interrupcion.
        media=vector[i]+media;
      }
      media=media/tam;  //Se divide entre la cantidad de datos para asi obtener el promedio.
      deltamuestreointerrupcion=media;
      muestreoanteriorinterrupcion=muestreoactualinterrupcion;//Se actualiza el tiempo de interrupcion anterior
    }
ciclo=0;  //Deja el tick en 0
muestreoactualinterrupcion=millis();  //Actualiza el muestreo actual.
    
    //if(voltajeSalida==0.00){n=0;}
    
//Calculo de la frecuencia angular:
    if(deltamuestreointerrupcion!=0){ //Según el promedio de tiempo sacado anteriormente se pasa a frecuencia en radianes por segundo:
      frecuencia=(1000*n)/(double)deltamuestreointerrupcion;
    }else{
      frecuencia=0;
    }
    //Se calcula la velocidad angular siendo 2*pi*frecuencia:
    w=(2*3.141516)*frecuencia;
}

void setup() {
  //Activacion de la interrupcion en flanco de bajada del encoder:
  attachInterrupt(digitalPinToInterrupt(encoder),encoderVoid,FALLING);
  Serial.begin(9600); //Monitor Serial en 9600 baudios.
  tiempo1=millis();
  //Se definen las entradas y salidas de los pines:
  pinMode(encoder, INPUT);  //Se necesita el pin del encoder como entrada para la interrupcion.
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); //Se define los pines del puente H como Salidas.
  pinMode(IN2, OUTPUT);
  
  //Señal Puente H L298N:
  digitalWrite(IN1,HIGH); //Se ajustan IN1 e IN2 para que gire en sentido horario el motor.
  digitalWrite(IN2,LOW);  //Si se quiere girar antihorario definir IN1=LOW e IN2=HIGH.
}

void loop() {
  //Manda el setpoint de PWM Manualmente:
  if(Serial.available()>0){
     String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el PWM a tomar.
     PWM=entradaSerial.toFloat();
  }

  if(Serial.available()=='h'){
    w=0;
    frecuencia=0;
    deltamuestreointerrupcion=0;
    PWM=0;
  }
  analogWrite(ENA,map(PWM,0,100,0,255)); //Enviamos el PWM al puente H.

  //Manda el setpoint de PWM de 10 en 10 automaticamente:
  /*if(millis() > 1000){
    PWM = int(millis()/1000);
  }
  if(millis() >= 101000){
    PWM = 0;
  }*/
  
  lecturaAnalogica=analogRead(A5); //Se lee el voltaje del pin analogico A5 para mensurar el voltaje de salida del generador.
  voltajeSalida=lecturaAnalogica/1023*5.0;
  
  tiempo2=millis();

  //CONFIGURACION FILTROS DIGITALES:
  /*
   * Instruccion de uso de los filtros:
   * - Para usar el filtro de segundo orden (sea de voltaje o velocidad) se debe comentar la linea de la ecuacion de diferencia del filtro de cuarto orden y sus parámetros extras.
   * - Para usar el filtro de cuarto orden (sea de voltaje o velocidad) se debe comentar la linea de la ecuacion de diferencia del filtro de segundo orden.
   */
   
  //Filtros Voltaje:
  //Filtro de Segundo Orden:
  yn_volt2 = 0.039507142397173*Voxn1 + 0.039507142397173*Voxn2 + 1.562310912858839*Voyn1 - 0.641325197653186*Voyn2;
  //Todos positivos excepto el ultimo
  
  //Filtro de Cuarto Orden:
  //yn_volt4 = 0.038200132186319*Voxn1 + 0.114600396558958*Voxn2 + 0.114600396558958*Voxn3 + 0.038200132186319*Voxn4 + 1.461344708802750*Voyn1 - 1.126469430817319*Voyn2 - 0.424438734294352*Voyn3 - 0.064915069770336*Voyn4;

  //Voyn4=Voyn3;
  //Voyn3=Voyn2;
  Voyn2=Voyn1;
  Voyn1=yn_volt2;
 
  //Voxn4=Voxn3;
  //Voxn3=Voxn2;
  Voxn2=Voxn1;
  Voxn1=voltajeSalida;
  
  /*
  //Filtros Velocidad:
  //Filtro de Segundo Orden:
  yn_vel2 = 0.039397178951906*Vexn1 + 0.039397178951906*Vexn2 + 1.562977224656995*Veyn1 - 0.641771582560807*Veyn2;
  //Filtro de Cuarto Orden:
  //yn_vel4 = 0.0008027063658306815*Vexn1 + 0.002408119097492*Vexn2 + 0.002408119097492*Vexn3 + 0.0008027063658306815*Vexn4 + 3.186014082683437*Veyn1 - 3.873036758083691*Veyn2 - 2.121242670533515*Veyn3 - 0.440641646059906*Veyn4;
  
  //Veyn4=Veyn3;
  //Veyn3=Veyn2;
  Veyn2=Veyn1;
  Veyn1=yn_vel2;
  
  //Vexn4=Vexn3;
  //Vexn3=Vexn2;
  Vexn2=Vexn1;
  Vexn1=w;
  */
  
  if(tiempo2>=tiempo1+2){  //Se obtendrá las muestras de velocidad, voltaje y PWM para los instantes de tiempo correspondientes al tiempo de muestreo.
      tiempo1 = millis();   //En este caso se ingresa el tiempo de muestreo en el if.
      tiempo = tiempo1/1000;
      
      //Salida de monitor Serial para imprimir los datos: 
      
      //Velocidad:
      //Serial.print(" "); Serial.print(yn_vel2); Serial.print(" ");  //Voltaje de salida en voltios.
      //Serial.print(" "); Serial.print(yn_vel4); Serial.print(" ");  //Voltaje de salida en voltios.
      //Serial.print(" "); Serial.print(w); Serial.print(" ");  //Velocidad en RPM.

      //Voltaje:
      //Serial.print(" "); Serial.print(yn_volt2); Serial.print(" ");  //Voltaje de salida en voltios.
      Serial.print(" "); Serial.print(yn_volt4); Serial.print(" ");  //Voltaje de salida en voltios.
      Serial.print(" "); Serial.print(voltajeSalida); Serial.print(" ");  //Porcentaje de PWM aplicado.
      
      //Serial.print(" "); Serial.print(tiempo); Serial.print(" ");  //Porcentaje de PWM aplicado.
      //Serial.print(" "); Serial.print(PWM); Serial.print(" ");  //Porcentaje de PWM aplicado.
      Serial.println();
    }
}
