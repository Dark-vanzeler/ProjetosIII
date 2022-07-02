// Atribuição dos pinos
#define ECHO_PIN1 2
#define TRIG_PIN1 3
#define ECHO_PIN2 4
#define TRIG_PIN2 5
#define ECHO_PIN3 6
#define TRIG_PIN3 7
#define m1 11 //motor m1 LadoEsquerdo
#define m2 12 //motor m2 LadoDireito

// Filtros de Kalman
double kalman(double U){
  static const double R = 10;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
}

double kalman1(double U1){
  static const double R1 = 10;
  static const double H1 = 1.00;
  static double Q1 = 10;
  static double P1 = 0;
  static double U_hat1 = 0;
  static double K1 = 0;
  K1 = P1*H1/(H1*P1*H1+1);
  U_hat1 += + K1*(U1-H1*U_hat1);
  P1 = (1-K1*H1)*P1+Q1;
  return U_hat1;
}

double kalman2(double U2){
  static const double R2 = 10;
  static const double H2 = 1.00;
  static double Q2 = 10;
  static double P2 = 0;
  static double U_hat2 = 0;
  static double K2 = 0;
  K2 = P2*H2/(H2*P2*H2+1);
  U_hat2 += + K2*(U2-H2*U_hat2);
  P2 = (1-K2*H2)*P2+Q2;
  return U_hat2;
}

void setup() {
  Serial.begin(9600);
  // Configuração dos pinos
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT); // definição E/S pinos do Sensor 1
  
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT); // definição E/S pinos do Sensor 2
  
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);// definição E/S pinos do Sensor 3
  
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
}

// Estimação da distância
float readDistanceCM(int x, int y) {
  digitalWrite(x, LOW);
  delayMicroseconds(2);
  digitalWrite(x, HIGH);
  delayMicroseconds(10);
  digitalWrite(x, LOW); 
  int duration = pulseIn(y, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  
  boolean close = true;

  float distance1 = readDistanceCM(TRIG_PIN3, ECHO_PIN3);
  float kaldist = kalman(distance1);
  float distance2 = readDistanceCM(TRIG_PIN2, ECHO_PIN2);
  float kaldist1 = kalman1(distance2);
  float distance3 = readDistanceCM(TRIG_PIN1, ECHO_PIN1);
  float kaldist2 = kalman2(distance3);
  
  // Avalia qual/quais motores devem ser ativados  
  if (150 > kaldist && kaldist < kaldist1 && kaldist < kaldist2){
    digitalWrite(m2, close);  // Coloca m2 em nível logico alto
    delay(500);               // durante 500 ms
    digitalWrite(m2, !close); // Coloca m1 em nível logico baixo
  }else if (150 > kaldist1 && kaldist1 < kaldist && kaldist1 < kaldist2){
    digitalWrite(m2, close);  // Coloca m2 e m1 
    digitalWrite(m1, close);  // em nível logico alto
    delay(500);               // durante 500 ms
    digitalWrite(m2, !close); // Coloca m2 em nível logico baixo
    digitalWrite(m1, !close); // Coloca m1 em nível logico baixo
  }else if (150 > kaldist2 && kaldist2 < kaldist && kaldist2 < kaldist1){
    digitalWrite(m1, close);  // Coloca m1 em nível logico alto
    delay(500);               // durante 500 ms 
    digitalWrite(m1, !close); // Coloca m1 em nível logico baixo
  }
  
  Serial.print(kaldist);
  Serial.print(" , ");
  Serial.print(kaldist1);
  Serial.print(" , ");
  Serial.println(kaldist2);
  delay(100);
}
