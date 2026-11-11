/*
 * Copia del sketch validado de tu companero (hexapodo-secundario-firmware,
 * pruebas/motores_encoders/sketch_test_motor2_final). Sirve para aislar
 * hardware vs firmware: si con esto el motor gira, el cableado esta bien.
 * Monitor serie a 115200, enviar: F (100% IN1), R (100% IN2), 1/2 (PWM 130),
 * S (freno), T (ticks,CS crudo), Z (cero), P (ping).
 * IN1 = D18, IN2 = D17, CS = 39 (VN), C1 = D25, C2 = D13
 */
const int M2_IN1 = 18;
const int M2_IN2 = 17;
const int M2_CS  = 39;
const int M2_C1  = 25;
const int M2_C2  = 13;

volatile long ticksM2 = 0;

void IRAM_ATTR isrM2() {
  int c1 = digitalRead(M2_C1);
  int c2 = digitalRead(M2_C2);
  if (c1 != c2) ticksM2++; else ticksM2--;
}

void freno() {
  analogWrite(M2_IN1, 0); digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 0); digitalWrite(M2_IN2, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  freno();
  pinMode(M2_C1, INPUT_PULLUP);
  pinMode(M2_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M2_C1), isrM2, CHANGE);
  Serial.println("READY_M2_FINAL");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') { digitalWrite(M2_IN2, LOW); analogWrite(M2_IN1, 130); Serial.println("OK:M2_IN1_PWM"); }
    else if (cmd == '2') { digitalWrite(M2_IN1, LOW); analogWrite(M2_IN2, 130); Serial.println("OK:M2_IN2_PWM"); }
    else if (cmd == 'F') { digitalWrite(M2_IN2, LOW); digitalWrite(M2_IN1, HIGH); Serial.println("OK:M2_IN1_FULL"); }
    else if (cmd == 'R') { digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH); Serial.println("OK:M2_IN2_FULL"); }
    else if (cmd == 'S') { freno(); Serial.println("OK:STOP"); }
    else if (cmd == 'Z') { ticksM2 = 0; Serial.println("OK:ZERO"); }
    else if (cmd == 'T') { Serial.printf("DATA:%ld,%d\n", ticksM2, analogRead(M2_CS)); }
    else if (cmd == 'P') { Serial.println("PONG"); }
  }
}
