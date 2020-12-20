//*************************
//Robotic Pollinator: Control, Encoder Reading and Serial Communication
//*************************
//Authors: Daniel Medina, Daniel Acuña
//Email: dlmedina@uc.cl, doacuna@uc.cl
//Date: December 2020
//*************************

// puertos redefinidos Serial 1 (puente H 1)
#define RX1 18   // D32
#define TX1 19  // D25

//#define SW1 32
//#define SW2 27
//#define SW3 21
//#define SW4 4 //Led Azul

// canales encoders
// M1
#define A1 34  // D22
#define B1 35  // D23
// M2
#define A2 26  // D5
#define B2 25  // D18
// M3
#define A3 22  // D19
#define B3 23  // D21
// M4 (maxon)
#define A4 15 // D2
#define B4 2 // D4
// interrupciones
volatile int A;
volatile int B;
// sincronización entre main e interrupciones
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;

// variables encoders
String data;
char select = 'y'; // y: puente H de Motores 1 y 2. z: puente H de Motores 3 y 4.
// posición angular en "cuentas"
volatile int POS1 = 0;       // para que cuando el brazo esté vertical
volatile int POS2 = 17353;   // todos los ángulos sean 0, es decir, se tiene
volatile int POS3 = -17800;//-19230;  // la Zero Configuration, necesaria para hacer
volatile int POS4 = -270;    // la cinemática inversa-directa
int PPR = 44000;
int PPRmaxon = 16;

// -------- CONTROLADOR --------
// variables de tiempo
//unsigned long time;
//unsigned long time_ant = 0;
float dt = 0.02;

// variables de theta
float theta1Goal = -0.3;
float theta2Goal = 0.1;
float theta3Goal = -1.63;
float theta4Goal = 0.0;

// PID 1
float kP_1 = 500.0; // ajustar
float kI_1 = 20.0; // ajustar
float kD_1 = 0.0; // ajustar
float e_acum1 = 0.0;
float theta1_ant1 = 0.0;
float theta1_ant2 = 0.0;
float theta1_ant3 = 0.0;
float sat_e_acum1 = 1000.0; // ajustar
float sat_u1 = 2047.0;

// PID 2
float kP_2 = 1000.0; // ajustar
float kI_2 = 500.0; // ajustar
float kD_2 = 100.0; // ajustar
float e_acum2 = 0.0;
float theta2_ant1 = 0.0;
float theta2_ant2 = 0.0;
float theta2_ant3 = 0.0;
float sat_e_acum2 = 1000.0; // ajustar
float sat_u2 = 2047.0;

// PID 3
float kP_3 = 500.0; // ajustar
float kI_3 = 100.0; // ajustar
float kD_3 = 0.0; // ajustar
float e_acum3 = 0.0;
float theta3_ant1 = 0.0;
float theta3_ant2 = 0.0;
float theta3_ant3 = 0.0;
float sat_e_acum3 = 1000.0; // ajustar
float sat_u3 = 2047.0;

// PID 4
float kP_4 = 300.0; // ajustar
float kI_4 = 100.0; // ajustar
float kD_4 = 5.0; // ajustar
float e_acum4 = 0.0;
float theta4_ant1 = 0.0;
float theta4_ant2 = 0.0;
float theta4_ant3 = 0.0;
float sat_e_acum4 = 1000.0; // ajustar
float sat_u4 = 2047.0;

// servomotor
#include <Servo_ESP32.h>
Servo_ESP32 servo;
static const int servoPin = 12; // D12
String servo_state;

// funciones interrupciones encoders
void IRAM_ATTR encoder1() {
  portENTER_CRITICAL(&synch);
  A = digitalRead(A1);
  B = digitalRead(B1);
  if (A and !B)
  {
    POS1++;
  }
  else{
    if(A and B)
    {
      POS1--;
    }
  }
 portEXIT_CRITICAL(&synch);
}
void IRAM_ATTR encoder2() {
  portENTER_CRITICAL(&synch);
  A = digitalRead(A2);
  B = digitalRead(B2);
  if (A and !B)
  {
    POS2++;
  }
  else{
    if(A and B)
    {
      POS2--;
    }
  }
 portEXIT_CRITICAL(&synch);
}
void IRAM_ATTR encoder3() {
  portENTER_CRITICAL(&synch);
  A = digitalRead(A3);
  B = digitalRead(B3);
  if (A and !B)
  {
    POS3--; 
    
  }
  else{
    if(A and B)
    {
      POS3++;
    }
  }
 portEXIT_CRITICAL(&synch);
}
void IRAM_ATTR encoder4() {
  portENTER_CRITICAL(&synch);
  A = digitalRead(A4);
  B = digitalRead(B4);
  if (A and !B)
  {
    POS4--;
  }
  else{
    if(A and B)
    {
      POS4++;
    }
  }
 portEXIT_CRITICAL(&synch);
}


//--------------------------- SETUP ---------------------------
void setup() {
  // inicializado de puertos UARTs:
  Serial.begin(9600);                        // python-ESP32
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1); // puenteH1 (M2, M3)
  Serial2.begin(9600);                       // puenteH2 (M1, M4)

  //Switches Fin de carrera
  //pinMode(SW1, INPUT);
  //pinMode(SW2, INPUT);
  //pinMode(SW3, INPUT);
  //pinMode(SW4, INPUT);
  
  // encoders
  pinMode(A1, INPUT);
  pinMode(B1, INPUT);
  pinMode(A2, INPUT);
  pinMode(B2, INPUT);
  pinMode(A3, INPUT);
  pinMode(B3, INPUT);
  pinMode(A4, INPUT);
  pinMode(B4, INPUT);

  attachInterrupt(digitalPinToInterrupt(A1), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(A2), encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(A3), encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(A4), encoder4, RISING);

  // servomotor
  servo.attach(servoPin);

}


//--------------------------- LOOP ---------------------------
void loop() {

  if (Serial.available())
  {
    String theta1Goal_str = Serial.readStringUntil(';');
    String theta2Goal_str = Serial.readStringUntil(';');
    String theta3Goal_str = Serial.readStringUntil(';');
    String theta4Goal_str = Serial.readStringUntil(';');
    servo_state = Serial.readStringUntil('\0');
    theta1Goal = theta1Goal_str.toFloat();
    theta2Goal = theta2Goal_str.toFloat();
    theta3Goal = theta3Goal_str.toFloat();
    theta4Goal = theta4Goal_str.toFloat();
  }
   
  // overflow de posiciones angulares de motores
  POS1 = POS1%PPR;
  POS2 = POS2%PPR;
  POS3 = POS3%PPR;
  //POS4 = POS4%PPRmaxon;
  // posiciones angulares de cuentas a radianes
  float theta1Measu = POS1*6.2832/0.667/PPR;
  float theta2Measu = POS2*6.2832*0.667/PPR;
  float theta3Measu = POS3*6.2832*0.667/PPR;
  float theta4Measu = POS4*6.2832*0.01481/PPRmaxon;

  String dataTheta1 = String(theta1Measu, 4);
  String dataTheta2 = String(theta2Measu, 4);
  String dataTheta3 = String(theta3Measu, 4);
  String dataTheta4 = String(theta4Measu, 4);
  String dataSer = dataTheta1 + ";" + dataTheta2 + ";" + dataTheta3 + ";" + dataTheta4;

  if (Serial.availableForWrite())
  {
    Serial.println(dataSer);
  }

  // control servo
  if (servo_state == "on\n")
    {
      servo.write(90); // en grados
    }
  else if (servo_state == "off\n")
    {
      servo.write(60); // en grados
    }

  // --------- PID 1 ---------
  float filtMeasu1 = (theta1Measu + theta1_ant1 + theta1_ant2 + theta1_ant3) / 4;
  float e_k1 = theta1Goal - filtMeasu1;
  // float e_k1 = theta1Goal - theta1Measu;
  float uP_1 = kP_1 * e_k1;
  float uI_1 = kI_1 * (e_acum1 + e_k1) * dt;
  float uD_1 = kD_1 * (theta1_ant1 - theta1Measu) / dt;
  float u1 = - uP_1 - uI_1 - uD_1;
  //float u1 = 0;
  theta1_ant3 = theta1_ant2;
  theta1_ant2 = theta1_ant1;
  theta1_ant1 = theta1Measu;
  e_acum1 = e_acum1 + e_k1;
  // saturación e_acum
  if (e_acum1 > sat_e_acum1)
  {
    e_acum1 = sat_e_acum1;
  }
  else if (e_acum1 < -sat_e_acum1)
  {
    e_acum1 = -sat_e_acum1;
  }
  // saturación u
  if (u1 > sat_u1)
  {
    u1 = sat_u1;
  }
  else if (u1 < -sat_u1)
  {
    u1 = -sat_u1;
  }

  // --------- PID 2 ---------
  float filtMeasu2 = (theta2Measu + theta2_ant1 + theta2_ant2 + theta2_ant3) / 4;
  float e_k2 = theta2Goal - filtMeasu2;
  float uP_2 = kP_2 * e_k2;
  float uI_2 = kI_2 * (e_acum2 + e_k2) * dt;
  float uD_2 = kD_2 * (theta2_ant1 - theta2Measu) / dt;
  float u2 = - uP_2 - uI_2 - uD_2 + 400*sin(theta2Measu) + 300*sin(theta2Measu + theta3Measu);
  //float u2 = + 400*sin(theta2Measu) + 300*sin(theta2Measu + theta3Measu);
  theta2_ant3 = theta2_ant2;
  theta2_ant2 = theta2_ant1;
  theta2_ant1 = theta2Measu;
  e_acum2 = e_acum2 + e_k2;
  // saturación e_acum
  if (e_acum2 > sat_e_acum2)
  {
    e_acum2 = sat_e_acum2;
  }
  else if (e_acum2 < -sat_e_acum2)
  {
    e_acum2 = -sat_e_acum2;
  }
  // saturación u
  if (u2 > sat_u2)
  {
    u2 = sat_u2;
  }
  else if (u2 < -sat_u2)
  {
    u2 = -sat_u2;
  }
  // --------- PID 3 ---------
  float filtMeasu3 = (theta3Measu + theta3_ant1 + theta3_ant2 + theta3_ant3) / 4;
  float e_k3 = theta3Goal - filtMeasu3;
  float uP_3 = kP_3 * e_k3;
  float uI_3 = kI_3 * (e_acum3 + e_k3) * dt;
  float uD_3 = kD_3 * (theta3_ant1 - theta3Measu) / dt;
  float u3 = uP_3 + uI_3 + uD_3 - 250*sin(theta2Measu + theta2Measu); // check sign
  //float u3 = -250*sin(theta2Measu + theta3Measu);
  //float u3 = 0;
  theta3_ant3 = theta3_ant2;
  theta3_ant2 = theta3_ant1;
  theta3_ant1 = theta3Measu;
  e_acum3 = e_acum3 + e_k3;
  // saturación e_acum
  if (e_acum3 > sat_e_acum3)
  {
    e_acum3 = sat_e_acum3;
  }
  else if (e_acum3 < -sat_e_acum3)
  {
    e_acum3 = -sat_e_acum3;
  }
  // saturación u
  if (u3 > sat_u3)
  {
    u3 = sat_u3;
  }
  else if (u3 < -sat_u3)
  {
    u3 = -sat_u3;
  }
  // --------- PID 4 ---------
  float filtMeasu4 = (theta4Measu + theta4_ant1 + theta4_ant2 + theta4_ant3) / 4;
  float e_k4 = theta4Goal - filtMeasu4;
  float uP_4 = kP_4 * e_k4;
  float uI_4 = kI_4 * (e_acum4 + e_k4) * dt;
  float uD_4 = kD_4 * (theta4_ant1 - theta4Measu) / dt;
  float u4 = uP_4 + uI_4 + uD_4;
  //float u4 = 0;
  theta4_ant3 = theta4_ant2;
  theta4_ant2 = theta4_ant1;
  theta4_ant1 = theta4Measu;
  e_acum4 = e_acum4 + e_k4;
  // saturación e_acum
  if (e_acum4 > sat_e_acum4)
  {
    e_acum4 = sat_e_acum4;
  }
  else if (e_acum4 < -sat_e_acum4)
  {
    e_acum4 = -sat_e_acum4;
  }
  // saturación u
  if (u4 > sat_u4)
  {
    u4 = sat_u4;
  }
  else if (u4 < -sat_u4)
  {
    u4 = -sat_u4;
  }
  

  // ganancias a los motores
  //Serial1.println("M1: " + String(int(round(u1))));
  //Serial1.flush();
  //Serial1.println("M2: " + String(int(round(u2))));
  //Serial1.flush();
  //Serial2.println("M1: " + String(int(round(u3))));
  //Serial2.flush();
  //Serial2.println("M2: " + String(int(round(u4))));
  //Serial2.flush();
  Serial1.print("M1: " + String(int(round(u1))) + "\r\n");
  Serial1.print("M2: " + String(int(round(u4))) + "\r\n");
  Serial2.print("M1: " + String(int(round(u2))) + "\r\n");
  Serial2.print("M2: " + String(int(round(u3))) + "\r\n");

  delay(20);
}
