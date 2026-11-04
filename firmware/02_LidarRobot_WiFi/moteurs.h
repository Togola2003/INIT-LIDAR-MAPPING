// ============================================================================
//  Moteurs du robot Mecanum : 4 roues, chacune pilotée par 2 broches (A, B) d'un pont en H.
//    A en PWM, B à 0  -> la roue tourne dans un sens ; A à 0, B en PWM -> l'autre sens ; tout à 0 -> arrêt.
//
//  Rempli avec le résultat de l'étape 1 (01_Moteurs_Test, relevé du 04/11) :
//  pour chaque POSITION de roue, broche A = celle qui la fait AVANCER, broche B = celle qui la fait reculer.
//  (inverser = true si, avec « avancer », une roue tourne quand même en arrière.)
// ============================================================================
#pragma once
#include <Arduino.h>

struct Wheel { uint8_t pinA; uint8_t pinB; bool invert; const char* name; };

Wheel WHEELS[4] = {
  {18, 19, false, "avant gauche"},     // M2 : m2a (D18) avance, m2b (D19) recule   (touches 7 / 8)
  { 5, 17, false, "avant droite"},     // M1 : m1b (D5)  avance, m1a (D17) recule   (touches 6 / 5)
  {15,  2, false, "arriere gauche"},   // M3 : m3a (D15) avance, m3b (D2)  recule   (touches 1 / 2)
  {16,  4, false, "arriere droite"},   // M4 : m4b (D16) avance, m4a (D4)  recule   (touches 4 / 3)
};
enum { AVG = 0, AVD = 1, ARG = 2, ARD = 3 };

const uint32_t PWM_FREQ = 5000;   // Hz
const uint8_t PWM_BITS = 8;       // vitesse 0..255

// Mouvements d'un robot Mecanum (signe de chaque roue : avant gauche, avant droite, arrière gauche, arrière droite)
struct Move { const char* name; int8_t s[4]; };
const Move MOVES[] = {
  {"forward",     { 1,  1,  1,  1}},
  {"backward",    {-1, -1, -1, -1}},
  {"left",        {-1,  1,  1, -1}},   // translation vers la gauche
  {"right",       { 1, -1, -1,  1}},
  {"rotateleft",  {-1,  1, -1,  1}},   // rotation sur place, sens trigonométrique
  {"rotateright", { 1, -1,  1, -1}},
};

inline void pwmAttach(uint8_t pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  static uint8_t channel = 0;
  ledcSetup(channel, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, channel++);
#endif
}

inline void pwmWrite(uint8_t pin, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  // en version 2, ledcWrite prend le canal : on retrouve le canal à partir de l'ordre d'attache
  for (uint8_t k = 0; k < 4; k++) {
    if (WHEELS[k].pinA == pin) { ledcWrite(2 * k, duty); return; }
    if (WHEELS[k].pinB == pin) { ledcWrite(2 * k + 1, duty); return; }
  }
#endif
}

// À appeler EN PREMIER dans setup() : broches à 0 avant tout le reste
inline void motorsSafeBoot() {
  for (const Wheel& w : WHEELS) {
    pinMode(w.pinA, OUTPUT); digitalWrite(w.pinA, LOW);
    pinMode(w.pinB, OUTPUT); digitalWrite(w.pinB, LOW);
  }
}

inline void motorsInit() {
  for (const Wheel& w : WHEELS) {
    pwmAttach(w.pinA);
    pwmAttach(w.pinB);
    pwmWrite(w.pinA, 0);
    pwmWrite(w.pinB, 0);
  }
}

// speed : -255 (plein arrière) … 255 (plein avant)
inline void setWheel(uint8_t k, int speed) {
  const Wheel& w = WHEELS[k];
  if (w.invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  pwmWrite(w.pinA, speed > 0 ? speed : 0);
  pwmWrite(w.pinB, speed < 0 ? -speed : 0);
}

inline void stopAll() {
  for (uint8_t k = 0; k < 4; k++) setWheel(k, 0);
}

inline const Move* findMove(const String& name) {
  for (const Move& m : MOVES) if (name == m.name) return &m;
  return nullptr;
}
