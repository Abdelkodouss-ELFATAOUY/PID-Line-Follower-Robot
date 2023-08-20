 /*********************************************************************************************
                                        INIAILISATION
**********************************************************************************************/
#include <Servo.h>
#include <SoftwareSerial.h>

// DECLARATION DES VARIABLE EN LIEN AVEC LE CHOIX DU MODE........................................
# define ARRETE 0
# define SUIVEE_LA_LIGNE 1
# define PAS_DE_LIGNE 2
# define ULTRASONS 3
  int mode = 0;

//DECLARATION EN LIEN AVEC LES MOTEURS...........................................................
   int motor_left_pwmPin  = 6;
   int motor_right_pwmPin = 5;

   int motor_left_pin_1 = 12;
   int motor_left_pin_2 = 13;
   int motor_right_pin_1 = 8;
   int motor_right_pin_2 = 7;
   
   const int power = 180;
   int power_right=200;
   int power_left=140;
/*POUR LE SENSE DES MOTEURS*/
 #define RIGHT 1
 #define LEFT -1

//DECLARATION EN LIEN AVEC LE CAPTEUR DE DISTANCE ET LE SERVOMOTEUR...............................
/*servo moteur*/
  Servo servo;
/* capteur Ulrasons*/
   const int triggerPin=2;//fil orange
   const int echoPin=3;//fil jaune
   const int delay_time = 250;//Temps accordé au servo pour la mesure de la distance de chaque côté
   long duree;
   int distance;
   float intervalle;
   int distanceAvant;
   int distanceDroite;
   int distanceGauche;

//DECLARATION EN LIEN AVEC LES CAPTEURS DE LUMINOSITE..............................................
// LFSensor Le Capteur plus à droit est "0"
   const int lineFollowSensor0 = A0; 
   const int lineFollowSensor1 = A1; 
   const int lineFollowSensor2 = A2; 
   const int lineFollowSensor3 = A3;
   const int lineFollowSensor4 = A4;

   int LFSensor[5]={0, 0, 0, 0, 0};

// Module bluetooth................................................................................
SoftwareSerial BT1(0,1); //le pin 0 à Rx  et  le pin 1 à Tx
String command;
String device;

// PID controller..................................................................................
float Kp=20;
float Ki=12;
float Kd=4;
float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void setup() 
{
  //Inialistion de la communication série
  Serial.begin(9600);
  BT1.begin(9600);

  //Inialistion des broches des moteurs
  pinMode(motor_left_pin_1, OUTPUT);
  pinMode(motor_left_pin_2, OUTPUT);
  pinMode(motor_right_pin_1, OUTPUT);
  pinMode(motor_right_pin_2, OUTPUT);
  digitalWrite(motor_left_pin_1,LOW);
  digitalWrite(motor_left_pin_2,LOW);
  digitalWrite(motor_right_pin_1,LOW);
  digitalWrite(motor_right_pin_2,LOW);

 //Inialistion du capteur de distance et du servomoteur 
  pinMode(triggerPin,OUTPUT);
  pinMode( echoPin,INPUT);
  servo.attach(9);//Associe l'objet servo à la pin 9 (PWM)
  servo.write(90); 

  //Inialistion des capteurs de coleur
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);
  
  BT1.print("Vérifier les constantes PID à envoyer au Robot");
  BT1.println('\n');
    while (!mode)
  {  
    commande_BT_Serie();
  }
  checkPIDvalues();
}
/***********************************************************************************
                               FONCTION POUR LES MOTEURS
************************************************************************************/
void farword_Right(){
  digitalWrite(motor_right_pin_1, LOW);
  digitalWrite(motor_right_pin_2, HIGH);
 }
 void Backword_Right(){
  digitalWrite(motor_right_pin_1, HIGH);
  digitalWrite(motor_right_pin_2, LOW);
 }
 void farword_Left(){
  digitalWrite(motor_left_pin_1, LOW);
  digitalWrite(motor_left_pin_2, HIGH);
  }
  void Backword_Left(){
  digitalWrite(motor_left_pin_1, HIGH);
  digitalWrite(motor_left_pin_2, LOW);
 }
 void motorStop()
{
  analogWrite(motor_right_pwmPin,0);
  analogWrite(motor_left_pwmPin, 0);
  digitalWrite(motor_right_pin_1, LOW);
  digitalWrite(motor_right_pin_2, LOW);
  digitalWrite(motor_left_pin_1, LOW);
  digitalWrite(motor_left_pin_2, LOW);
  delay(200);
}
//--------------------------------------------- 
void motorForward()
{analogWrite(motor_right_pwmPin,power );
  analogWrite(motor_left_pwmPin, power );
  farword_Right();
  farword_Left();
}
//---------------------------------------------
void motorBackward()
{
   analogWrite(motor_right_pwmPin,power );
  analogWrite(motor_left_pwmPin, power );
  Backword_Right();
  Backword_Left();
}
//---------------------------------------------
void motorTurn(int direction, int degrees)
{
  analogWrite(motor_right_pwmPin,200 );
  analogWrite(motor_left_pwmPin,140 );
if(direction==RIGHT){
  farword_Left();
  Backword_Right();
  delay (round(8*degrees+1));
  motorStop();
}
else if(direction==LEFT){
  farword_Right();
  Backword_Left();
  delay (round(8*degrees+1));
  motorStop();
  }
  }
/************************************************************************************************
                                 FONCTIONS POUR LE MODE ULTRATION
*************************************************************************************************/
void mesurerDistanceAvant()
{
  servo.write(90);
  delay(delay_time);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH); //Envoie d'une onde sonore
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  intervalle = pulseIn(echoPin, HIGH); //Réception de l'écho
  intervalle = intervalle/5.8/10; //Conversion de la différence de temps entre l'envoie de l'onde sonore et la réception de l'écho en distance (cm)
  Serial.println("Distance avant:");
  Serial.println(intervalle);
  distanceAvant = intervalle; //Arrondissement de la distance
}
//------------------------------------------------------------------
void mesurerDistanceGauche()
{
  servo.write(180);
  delay(delay_time);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds( 2);
  digitalWrite(triggerPin, HIGH); //Envoie d'une onde sonore
  delayMicroseconds( 10);
  digitalWrite(triggerPin, LOW);
  intervalle = pulseIn(echoPin, HIGH); //Réception de l'écho
  intervalle = intervalle/5.8/10; //Conversion de la différence de temps entre l'envoie de l'onde sonore et la réception de l'écho en distance (cm)
  Serial.println("Distance gauche:");
  Serial.println(intervalle);
  distanceGauche = intervalle;
}
//-------------------------------------------------------------------
void mesurerDistanceDroite()
{
  servo.write(0);
  delay(delay_time);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds( 2);
  digitalWrite(triggerPin, HIGH); //Envoie d'une onde sonore
  delayMicroseconds( 10);
  digitalWrite(triggerPin, LOW);
  intervalle = pulseIn(echoPin, HIGH); //Réception de l'écho
  intervalle = intervalle/5.8/10; //Conversion de la différence de temps entre l'envoie de l'onde sonore et la réception de l'écho en distance (cm)
  Serial.println("Distance droite:");
  Serial.println(intervalle);
  distanceDroite = intervalle;
}
//-------------------------------------------------------------------
void modeUltrason()
{
  mesurerDistanceAvant();  
  if(distanceAvant < 25) //Si la distance avant est de moins de 25cm
  {
    motorStop();
    mesurerDistanceGauche();
    delay(delay_time);
    mesurerDistanceDroite();
    delay(delay_time);
    
    if(distanceGauche < 15 && distanceDroite < 15) //Si la distance à gauche et la distance à droite sont de moins de 15cm
    {
      motorBackward();
      delay(500);
      motorTurn(RIGHT,20);
    }
    else if(distanceGauche > distanceDroite) //Si la distance gauche est plus grande que la distance droite
    {
      motorTurn(LEFT,60);
    }
    else if(distanceGauche <= distanceDroite) //Si la distance gauche est plus petite ou égale à la distance droite
    {
      motorTurn(RIGHT,70);
    }
  }
  else //Si la distance avant est de plus de 25cm
  {
    motorForward();
  }
}
//------------------------------------------------------------------
void ultrasonDistance()
 {
digitalWrite(triggerPin,HIGH);
delayMicroseconds(10);
digitalWrite(triggerPin,LOW);
duree = pulseIn(echoPin,HIGH);
duree = duree/2;
distance = (duree*340)/10000.0;
 }
/***********************************************************************************************
                               FONCTIONS POUR les CAPTEURS INFRAROUGE
************************************************************************************************/
      /* Lire les valeur des capteur de ligne 

                TableCapteur  ValeurError
                 0 0 0 0 1       4              
                 0 0 0 1 1       3              
                 0 0 0 1 0       2              
                 0 0 1 1 0       1              
                 0 0 1 0 0       0              
                 0 1 1 0 0      -1              
                 0 1 0 0 0      -2              
                 1 1 0 0 0      -3              
                 1 0 0 0 0      -4              

                 1 1 1 1 1        0 Robot trouvé ligne continue : ARRETE
                 0 0 0 0 0        0 Robot n'a trouvé aucune ligne : Tourner 180o

      */
void readLFSsensors()
{
  LFSensor[0] = digitalRead(lineFollowSensor0);
  LFSensor[1] = digitalRead(lineFollowSensor1);
  LFSensor[2] = digitalRead(lineFollowSensor2);
  LFSensor[3] = digitalRead(lineFollowSensor3);
  LFSensor[4] = digitalRead(lineFollowSensor4);
  
  if((     LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = SUIVEE_LA_LIGNE; error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = SUIVEE_LA_LIGNE; error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error =- 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error = -2;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error = -3;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = SUIVEE_LA_LIGNE; error = -4;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = ARRETE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = PAS_DE_LIGNE; error = 0;}
}

/************************************************************************************************
                                     CONNEXION PAR BLUETOOTH
*************************************************************************************************/
 void checkBTcmd()  
 { 
   while (BT1.available())   //Vérifier s'il y a un octet disponible à lire
   {
     delay(10); //Retard ajouté pour rendre la chose stable
     char c = BT1.read(); //Effectuer une lecture en série
     device += c; //construire la chaine
   }  
   if (device.length() > 0) 
   {
     Serial.print("Commande reçue BT ==> ");
     Serial.println(device); 
     command = device;
     device ="";  //Réinitialiser la variable
     BT1.flush();
    } 
}
//-----------------------------------------------------
void manualCmd()
{
  switch (command[0])
  {
    case 'g':
      mode = SUIVEE_LA_LIGNE;
      break;
    
    case 's': 
      motorStop(); //Eteindre les deux moteurs
      break;

    case 'f':  
      motorForward();  
      break;

    case 'r':     
      motorTurn(RIGHT, 30);
      motorStop();
      
      break;

   case 'l': 
      motorTurn(LEFT, 30);
      motorStop();
      break;

    case 'b':  
      motorBackward();
      break;

    case 'U':
      mode = ULTRASONS;
      break;
      
    case 'p':
      Kp = command[2];
      break;
    
    case 'i':
      Ki = command[2];
      break; 
    
    case 'd':
      Kd = command[2];
      break;
  }
}
//-------------------------------------------------------
void commande_BT_Serie(){
  checkBTcmd();
  manualCmd();
  command = "";
}

/************************************************************************************************
                                    FONCTIONS POUR PID 
*************************************************************************************************/
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}
//-----------------------------------------------
void checkPIDvalues()
{
  
  BT1.print("PID: ");
  BT1.print(Kp);
  BT1.print(" - ");
  BT1.print(Ki);
  BT1.print(" - ");
  BT1.println(Kd);  
  
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);  
  
}
//-----------------------------------------------
void test_Capteurs_SuiviLigne()
{
     int LFS0 = digitalRead(lineFollowSensor0);
     int LFS1 = digitalRead(lineFollowSensor1);
     int LFS2 = digitalRead(lineFollowSensor2);
     int LFS3 = digitalRead(lineFollowSensor3);
     int LFS4 = digitalRead(lineFollowSensor4);
     
     Serial.print ("LFS: L  0 1 2 3 4  R ==> "); 
     Serial.print (LFS0); 
     Serial.print (" ");
     Serial.print (LFS1); 
     Serial.print (" ");
     Serial.print (LFS2); 
     Serial.print (" ");
     Serial.print (LFS3); 
     Serial.print (" ");
     Serial.print (LFS4); 
     Serial.print ("  ==> ");
    
     Serial.print (" P: ");
     Serial.print (P);
     Serial.print (" I: ");
     Serial.print (I);
     Serial.print (" D: ");
     Serial.print (D);
     Serial.print (" PID: ");
     Serial.println (PIDvalue);
}
//--------------------------------------------
void motorPIDcontrol()
{
  int leftMotorSpeed = power - PIDvalue;
  int rightMotorSpeed = power+ PIDvalue;
  
  // La vitesse du moteur ne doit pas dépasser la valeur max PWM
   constrain(leftMotorSpeed, 0, 255);
   constrain(rightMotorSpeed, 0, 255);
  
  analogWrite(motor_right_pwmPin, rightMotorSpeed);
  analogWrite(motor_left_pwmPin, leftMotorSpeed);
   farword_Right();
   farword_Left();
}
//-------------------------------------------------------------------
void Suiveurligne_Ultrasons()
{
 readLFSsensors();
 ultrasonDistance();    
 if(distance <= 20){
  motorStop();
  servo.write(180);
  delay(400);
 }
else{ 
      calculatePID();
      motorPIDcontrol(); 
    servo.write(90);
   }
}
/********************************************************************************************************
                                   Intelligent_Robot_Suiveur_ligne
********************************************************************************************************/

void loop() 
{
    commande_BT_Serie();
    readLFSsensors();    
    switch (mode)
  {
    case ARRETE: 
      motorStop();
      BT1.print("The End");
      previousError = error;
      break;

    case PAS_DE_LIGNE:  
      motorStop();
      motorTurn(LEFT, 180);
      previousError = 0;
      break;

    case SUIVEE_LA_LIGNE:     
      Suiveurligne_Ultrasons();   
      break;  
      
    case ULTRASONS:
      modeUltrason();
      break;
  }
}
