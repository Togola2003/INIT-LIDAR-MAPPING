// ============================================================================
//  ESP32 + LD06 : envoi des scans complets en CSV sur le port série (115200 bauds)
//  Dérivé de l'exemple ESP32_LD06_Test.ino fourni (bibliothèque Robot-Maker-SAS/LD06).
//  Seul changement : printScanCSV au lieu de printScanTeleplot, pour que le script
//  Python  `python -m lidar.acquire`  puisse enregistrer les scans dans des fichiers.
//
//  Format par point : N,Angle(°),Distance(mm),Intensity[,x(mm),y(mm)]
//  Une ligne vide sépare deux scans 360°.
//  Câblage : TX du LD06 -> GPIO33 (RX Serial2), PWM du LD06 -> GPIO32, masse commune.
// ============================================================================

#include "ld06.h"

#define RX_LIDAR     33
#define TX_LIDAR     -1
#define LD06_PWM_PIN 32
#define LIDAR_BAUD   230400

LD06 ld06(Serial2, LD06_PWM_PIN);

void setup() {
  Serial.begin(115200);

  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(10);
  ld06.init();

  // Filtrage optionnel côté microcontrôleur (on préfère filtrer côté PC pour garder les données brutes)
  // ld06.enableFiltering();
  // ld06.setIntensityThreshold(100);
}

void loop() {
  if (ld06.readScan()) {
    ld06.printScanCSV(Serial);  // un scan complet + ligne vide
  }
}
