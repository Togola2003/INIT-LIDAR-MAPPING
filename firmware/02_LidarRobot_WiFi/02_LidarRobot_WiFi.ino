// ============================================================================
//  ÉTAPE 2 : robot complet en Wi-Fi : lidar LD06 + 4 moteurs Mecanum + télécommande téléphone.
//
//  - Moteurs ARRÊTÉS dès la mise sous tension (broches mises à 0 en tout premier).
//  - Le robot crée son Wi-Fi (SSID / mot de passe ci-dessous) ; adresse : http://192.168.4.1
//  - Même interface HTTP que RobotControl du prof : LidarLab, RobotControl.html et le téléphone
//    fonctionnent tous avec ce firmware :
//      /scan                                  dernier tour du lidar, en binaire (voir lidar/robot.py)
//      /forward /backward /left /right
//      /rotateleft /rotateright ?time=ms      un mouvement de time ms (2 s maximum), puis arrêt automatique
//      /stop                                  arrêt immédiat
//      /speed?v=60..255                       vitesse des mouvements (défaut 150)
//      /wheel?w=0..3&v=-255..255&time=ms      une seule roue (réglages : 0 AVG, 1 AVD, 2 ARG, 3 ARD)
//      /pose?x=mm&y=mm&z=cdeg                 pose mémorisée, renvoyée dans l'en-tête de /scan
//      /status (JSON)  /ping  /  (télécommande pour téléphone)
//  - Uniquement des bibliothèques incluses avec la carte ESP32 + la bibliothèque LD06 (déjà installée).
//  Carte : « ESP32 Dev Module ». Câblage lidar : TX du LD06 -> GPIO33, PWM -> GPIO32.
// ============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include "ld06.h"
#include "moteurs.h"
#include "page.h"

// ---------------------------------------------------------------- Wi-Fi : donne un nom propre à TON robot
const char* SSID = "LD06-Togola";       // plusieurs robots dans la salle ? mets un nom unique, ex. "LD06-Togola"
const char* PASSWORD = "12345678";     // 8 caractères minimum

// ---------------------------------------------------------------- lidar
#define RX_LIDAR 33
#define TX_LIDAR -1
#define LD06_PWM_PIN 32
LD06 lidar(Serial2, LD06_PWM_PIN);

const uint16_t MAX_SCAN_POINTS = 460;
uint8_t scanBuf[8 + 5 * MAX_SCAN_POINTS];
size_t scanSize = 0;
uint32_t scanCount = 0;
int16_t poseX = 0, poseY = 0;
uint16_t poseZ = 0;

// ---------------------------------------------------------------- mouvement en cours
const uint32_t MAX_MOVE_MS = 2000;
int moveSpeed = 150;
bool moving = false;
uint32_t moveEnd = 0;

WebServer server(80);

void sendText(int code, const String& text, const char* type = "text/plain") {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, type, text);
}

void startMove(const Move& m, uint32_t ms) {
  for (uint8_t k = 0; k < 4; k++) setWheel(k, m.s[k] * moveSpeed);
  moving = true;
  moveEnd = millis() + min(ms, MAX_MOVE_MS);
}

void stopRobot() {
  stopAll();
  moving = false;
}

void handleMove() {
  const Move* m = findMove(server.uri().substring(1));
  if (!m) { sendText(404, "MOUVEMENT INCONNU"); return; }
  uint32_t t = server.hasArg("time") ? server.arg("time").toInt() : 300;
  if (t == 0) stopRobot(); else startMove(*m, t);
  sendText(200, "OK");
}

void handleWheel() {
  int w = server.arg("w").toInt();
  if (w < 0 || w > 3) { sendText(400, "w = 0..3"); return; }
  stopAll();
  setWheel(w, server.arg("v").toInt());
  moving = true;
  moveEnd = millis() + min((uint32_t)(server.hasArg("time") ? server.arg("time").toInt() : 800), MAX_MOVE_MS);
  sendText(200, String("roue ") + WHEELS[w].name);
}

void handleScan() {
  if (!scanSize) { sendText(503, "NO_VALID_SCAN"); return; }
  scanBuf[0] = poseX & 0xff; scanBuf[1] = (uint16_t)poseX >> 8;
  scanBuf[2] = poseY & 0xff; scanBuf[3] = (uint16_t)poseY >> 8;
  scanBuf[4] = poseZ & 0xff; scanBuf[5] = poseZ >> 8;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(scanSize);
  server.send(200, "application/octet-stream", "");
  server.client().write(scanBuf, scanSize);
}

void handleStatus() {
  String j = String("{\"points\":") + (scanSize ? (scanSize - 8) / 5 : 0) + ",\"scans\":" + scanCount +
             ",\"rpm\":" + (lidar.getSpeed() / 6) + ",\"speed\":" + moveSpeed + ",\"moving\":" +
             (moving ? "true" : "false") + ",\"ssid\":\"" + SSID + "\"}";
  sendText(200, j, "application/json");
}

void publishScan() {
  uint16_t n = lidar.getNbPointsInScan();
  if (n == 0 || n > MAX_SCAN_POINTS) return;
  scanBuf[6] = n & 0xff;
  scanBuf[7] = n >> 8;
  size_t off = 8;
  for (uint16_t i = 0; i < n; ++i) {
    const DataPoint* p = lidar.getPoints(i);
    uint16_t a = (uint16_t)(p->angle * 100);
    scanBuf[off++] = a & 0xff;  scanBuf[off++] = a >> 8;
    scanBuf[off++] = p->distance & 0xff;  scanBuf[off++] = p->distance >> 8;
    scanBuf[off++] = p->intensity;
  }
  scanSize = off;
  scanCount++;
}

void setup() {
  motorsSafeBoot();                    // EN PREMIER : aucune roue ne doit tourner
  Serial.begin(115200);
  motorsInit();
  stopAll();

  Serial2.setRxBufferSize(4096);       // le lidar continue d'arriver pendant qu'on répond au Wi-Fi
  Serial2.begin(230400, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(2);
  lidar.init();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  delay(200);
  Serial.printf("Wi-Fi \"%s\" pret : http://%s\n", SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", [] { server.send_P(200, "text/html", PAGE_HTML); });
  for (const Move& m : MOVES) server.on(String("/") + m.name, handleMove);
  server.on("/stop", [] { stopRobot(); sendText(200, "STOPPED"); });
  server.on("/speed", [] {
    if (server.hasArg("v")) moveSpeed = constrain(server.arg("v").toInt(), 0, 255);
    sendText(200, String(moveSpeed));
  });
  server.on("/wheel", handleWheel);
  server.on("/scan", handleScan);
  server.on("/status", handleStatus);
  server.on("/pose", [] {
    poseX = constrain(server.arg("x").toInt(), -32768, 32767);
    poseY = constrain(server.arg("y").toInt(), -32768, 32767);
    poseZ = ((server.arg("z").toInt() % 36000) + 36000) % 36000;
    sendText(200, "OK");
  });
  server.on("/ping", [] { sendText(200, "OK"); });
  server.onNotFound([] { sendText(404, "?"); });
  server.begin();
}

void loop() {
  if (lidar.readScan()) publishScan();
  server.handleClient();
  if (moving && (int32_t)(millis() - moveEnd) >= 0) stopRobot();   // arrêt automatique en fin de mouvement
}
