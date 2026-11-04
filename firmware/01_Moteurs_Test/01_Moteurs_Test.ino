// ============================================================================
//  ÉTAPE 1 : arrêter les moteurs, puis identifier chaque fil (quelle roue, quel sens)
//
//  Pourquoi les moteurs tournaient tout seuls :
//   - les 8 fils des moteurs (m1a … m4b) arrivent directement sur des broches de l'ESP32 ;
//   - une broche jamais réglée « flotte », et certaines (D15, D5) sont à l'état HAUT au démarrage
//     de l'ESP32 : le pont en H du moteur reçoit alors un ordre de tourner ;
//   - le firmware RobotControl du prof utilise D19 (= fil m2b ici) pour sa liaison I2C : le moteur 2
//     reçoit donc des impulsions dès qu'il « parle ».
//   Remède : la TOUTE PREMIÈRE chose du programme met les 8 broches en sortie, à l'état BAS.
//
//  Utilisation : ROBOT SURÉLEVÉ (roues dans le vide), moniteur série à 115200 bauds, puis taper :
//     0           arrêt de tout
//     1 … 8       une seule broche à l'état HAUT pendant 0,8 s  -> noter quelle roue tourne et dans quel sens
//     a b c d     moteur M1 M2 M3 M4 en marche « A » (A haut, B bas) pendant 1 s
//     A B C D     moteur M1 M2 M3 M4 en marche « B » (A bas, B haut) pendant 1 s
//  Aucune bibliothèque à installer.
// ============================================================================

// Câblage relevé sur le robot : fil -> broche de l'ESP32
struct Fil { const char* name; uint8_t pin; };
const Fil WIRES[8] = {
  {"m3a", 15}, {"m3b", 2}, {"m4a", 4}, {"m4b", 16},
  {"m1a", 17}, {"m1b", 5}, {"m2a", 18}, {"m2b", 19},
};
// Moteurs : broche A, broche B
const uint8_t MOTOR_A[4] = {17, 18, 15, 4};   // M1 M2 M3 M4
const uint8_t MOTOR_B[4] = { 5, 19,  2, 16};

void allLow() {
  for (const Fil& w : WIRES) {
    pinMode(w.pin, OUTPUT);
    digitalWrite(w.pin, LOW);
  }
}

void setup() {
  allLow();                       // EN PREMIER : plus aucun moteur ne tourne
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== Test des moteurs : tous ARRETES ===");
  Serial.println("0 = stop | 1..8 = une broche | a..d = moteur sens A | A..D = moteur sens B");
  for (int k = 0; k < 8; k++) {
    Serial.printf("  %d -> fil %s (D%d)\n", k + 1, WIRES[k].name, WIRES[k].pin);
  }
}

void pulsePin(int k) {
  allLow();
  Serial.printf("Fil %s (D%d) HAUT pendant 0,8 s : quelle roue ? quel sens ?\n", WIRES[k].name, WIRES[k].pin);
  digitalWrite(WIRES[k].pin, HIGH);
  delay(800);
  allLow();
}

void runMotor(int m, bool directionA) {
  allLow();
  Serial.printf("Moteur M%d sens %s (D%d=%d, D%d=%d) pendant 1 s\n", m + 1, directionA ? "A" : "B",
                MOTOR_A[m], directionA, MOTOR_B[m], !directionA);
  digitalWrite(MOTOR_A[m], directionA ? HIGH : LOW);
  digitalWrite(MOTOR_B[m], directionA ? LOW : HIGH);
  delay(1000);
  allLow();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c >= '1' && c <= '8') pulsePin(c - '1');
  else if (c >= 'a' && c <= 'd') runMotor(c - 'a', true);
  else if (c >= 'A' && c <= 'D') runMotor(c - 'A', false);
  else if (c == '0' || c == ' ') { allLow(); Serial.println("STOP"); }
}
