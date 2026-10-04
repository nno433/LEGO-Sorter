#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TJpg_Decoder.h>
#include <Preferences.h>
#include <AccelStepper.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==================== CONFIGURATION & NETWORK ====================
static const char* WIFI_SSID   = "----------------------";
static const char* WIFI_PASS   = "----------------------";
static const char* CAM_IP     = "-----------------------";
static const uint16_t CAM_PORT = 80;
static const char* CAM_PATH   = "/capture";
static const uint16_t UDP_PORT = 8888;

// ==================== 4x IR SENSOR CONFIG ====================
#define IR1_PIN                26
#define IR2_PIN                27
#define IR3_PIN                14
#define IR4_PIN                13

// ==================== STEPPER MOTORS (A4988) ====================
#define STEP1_DIR_PIN          5
#define STEP1_STEP_PIN         17
#define CONVEYOR_SPEED_HZ      220    // Speed in Hz / steps per second
#define CONVEYOR_PWM_CHANNEL   0      // ESP32 LEDC PWM channel
#define REVERSE_CONVEYOR_DIR   true   // Set true/false to swap belt direction

#define STEP2_DIR_PIN          4
#define STEP2_STEP_PIN         16

// ==================== VISION GRID & PARAMETERS ====================
#define GRID_W              160
#define GRID_H              120
#define TOTAL_PIXELS        (GRID_W * GRID_H)
#define JPEG_SCALE          4

#define BLACK_THRESHOLD     60      // Ignores dark belt noise (R+G+B <= 60)
#define MIN_SATURATION      28.0f   // Rejects weak color drift (0-100%)
#define MIN_VALUE_PERCENT   35.0f   // Minimum required brightness percentage
#define MIN_OBJECT_PIXELS   450     // Filters out noise pixel clusters on 160x120 grid
#define MAX_JPG_BYTES       200000
#define HTTP_TIMEOUT_MS     3000

#define MAX_CONTOUR_POINTS    1000
#define CORNER_WINDOW         6
#define CORNER_ANGLE_DEG      50.0f
#define CORNER_RELEASE_DEG    30.0f
#define CORNER_MIN_SEPARATION 8

// ==================== STRUCTS & TYPES ====================
struct Pt { 
  int x; 
  int y; 
};

enum SortMode { MODE_COLOR = 0, MODE_SHAPE = 1, MODE_BOTH = 2 };

enum UIObjectColor { UI_COLOR_RED = 0, UI_COLOR_GREEN = 1, UI_COLOR_BLUE = 2, UI_COLOR_YELLOW = 3, UI_COLOR_UNKNOWN = -1 };
enum UIObjectShape { UI_SHAPE_CIRCLE = 0, UI_SHAPE_TRIANGLE = 1, UI_SHAPE_RECTANGLE = 2, UI_SHAPE_UNKNOWN = -1 };

struct BucketConfig {
  UIObjectColor color = UI_COLOR_RED;
  UIObjectShape shape = UI_SHAPE_CIRCLE;
};

struct ShapeDebug {
  float areaR = 0, areaC = 0, areaT = 0, actual = 0;
  int   corners = 0;
  bool  agree = false;
  char  areaVote = '?';
  char  cornerVote = '?';
};

// ==================== GLOBALS & STATE ====================
WiFiUDP udp;
Preferences preferences;

AccelStepper stepper2(AccelStepper::DRIVER, STEP2_STEP_PIN, STEP2_DIR_PIN);
const int STEPS_FOR_72_DEG = 40; 
static int currentBucket = 0;

SortMode activeSortMode = MODE_COLOR;
BucketConfig activeBuckets[4];

static uint16_t* rgbGrid  = nullptr;
static uint8_t*  mask     = nullptr;   
static uint16_t* ccQueue  = nullptr;   
static int16_t*  contourX = nullptr;
static int16_t*  contourY = nullptr;

static uint32_t objCount = 0;
static int       objMinX = 0, objMaxX = 0;
static int       objMinY = 0, objMaxY = 0;
static double   objCx = 0.0, objCy = 0.0;

static Pt pN, pS, pW, pE;

// ==================== GEOMETRY HELPERS ====================
static inline float distPt(const Pt& a, const Pt& b) {
  const float dx = (float)(a.x - b.x);
  const float dy = (float)(a.y - b.y);
  return sqrtf(dx * dx + dy * dy);
}

// ==================== IR SENSORS ====================
bool isObjectDetected() {
  return (digitalRead(IR1_PIN) == LOW || 
          digitalRead(IR2_PIN) == LOW || 
          digitalRead(IR3_PIN) == LOW ||
          digitalRead(IR4_PIN) == LOW);
}

// ==================== HARDWARE CONVEYOR CONTROL ====================
void startConveyor() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(STEP1_STEP_PIN, CONVEYOR_SPEED_HZ, 8);
  ledcWrite(STEP1_STEP_PIN, 128);
#else
  ledcSetup(CONVEYOR_PWM_CHANNEL, CONVEYOR_SPEED_HZ, 8);
  ledcAttachPin(STEP1_STEP_PIN, CONVEYOR_PWM_CHANNEL);
  ledcWrite(CONVEYOR_PWM_CHANNEL, 128);
#endif
  Serial.println("[CONVEYOR] Belt STARTED");
}

void stopConveyor() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(STEP1_STEP_PIN);
#else
  ledcDetachPin(STEP1_STEP_PIN);
#endif
  pinMode(STEP1_STEP_PIN, OUTPUT);
  digitalWrite(STEP1_STEP_PIN, LOW);
  Serial.println("[CONVEYOR] Belt STOPPED");
}

void initHardwareConveyor() {
  pinMode(STEP1_DIR_PIN, OUTPUT);
  digitalWrite(STEP1_DIR_PIN, REVERSE_CONVEYOR_DIR ? LOW : HIGH);

  startConveyor();
  Serial.printf("[CONVEYOR] Hardware PWM started at %d Hz\n", CONVEYOR_SPEED_HZ);
}

// ==================== COLOR ENGINE ====================
static inline void unpackRGB(uint16_t px, uint8_t& r, uint8_t& g, uint8_t& b) {
  r = ((px >> 11) & 0x1F) << 3;
  g = ((px >>  5) & 0x3F) << 2;
  b = ( px        & 0x1F) << 3;
}

static inline bool isColoured(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t sum = (uint32_t)r + g + b;
  if (sum <= BLACK_THRESHOLD) return false;

  const float rf = r / 255.0f, gf = g / 255.0f, bf = b / 255.0f;
  const float maxV = fmaxf(rf, fmaxf(gf, bf));
  const float minV = fminf(rf, fminf(gf, bf));
  const float delta = maxV - minV;

  const float v = maxV * 100.0f;
  const float s = (maxV == 0.0f) ? 0.0f : (delta / maxV) * 100.0f;

  if (v < MIN_VALUE_PERCENT) return false;
  if (s < MIN_SATURATION)    return false;
  
  return true;
}

static float hueOf(uint8_t r, uint8_t g, uint8_t b) {
  const float rf = r / 255.0f, gf = g / 255.0f, bf = b / 255.0f;
  const float maxV = fmaxf(rf, fmaxf(gf, bf));
  const float minV = fminf(rf, fminf(gf, bf));
  const float delta = maxV - minV;
  if (delta < 0.0001f) return -1.0f;

  float h;
  if      (maxV == rf) h = 60.0f * fmodf(((gf - bf) / delta), 6.0f);
  else if (maxV == gf) h = 60.0f * (((bf - rf) / delta) + 2.0f);
  else                 h = 60.0f * (((rf - gf) / delta) + 4.0f);
  if (h < 0.0f) h += 360.0f;
  return h;
}

static const char* classifyColor(uint8_t r, uint8_t g, uint8_t b) {
  const float h = hueOf(r, g, b);
  if (h < 0.0f) return "UNKNOWN";

  if (h < 15.0f || h >= 345.0f)  return "RED";
  if (h < 40.0f)                  return "ORANGE";
  if (h < 70.0f)                  return "YELLOW";
  if (h < 170.0f)                 return "GREEN";
  if (h < 200.0f)                 return "CYAN";
  if (h < 255.0f)                 return "BLUE";
  if (h < 290.0f)                 return "PURPLE";
  return "PINK";
}

UIObjectColor mapStrToUIColor(const char* colorStr) {
  if (strcmp(colorStr, "RED") == 0 || strcmp(colorStr, "ORANGE") == 0 || 
      strcmp(colorStr, "PINK") == 0 || strcmp(colorStr, "PURPLE") == 0) {
    return UI_COLOR_RED;
  }
  if (strcmp(colorStr, "GREEN") == 0) return UI_COLOR_GREEN;
  if (strcmp(colorStr, "BLUE") == 0 || strcmp(colorStr, "CYAN") == 0) return UI_COLOR_BLUE;
  if (strcmp(colorStr, "YELLOW") == 0) return UI_COLOR_YELLOW;
  return UI_COLOR_UNKNOWN;
}

UIObjectShape mapStrToUIShape(const char* shapeStr) {
  if (strcmp(shapeStr, "RECTANGLE") == 0) return UI_SHAPE_RECTANGLE;
  if (strcmp(shapeStr, "TRIANGLE") == 0)  return UI_SHAPE_TRIANGLE;
  if (strcmp(shapeStr, "CIRCLE") == 0)    return UI_SHAPE_CIRCLE;
  return UI_SHAPE_UNKNOWN;
}

const char* getShapeAbbr(const char* shapeStr) {
  if (strcmp(shapeStr, "RECTANGLE") == 0) return "RECT";
  if (strcmp(shapeStr, "TRIANGLE") == 0)  return "TRI";
  if (strcmp(shapeStr, "CIRCLE") == 0)    return "CIRC";
  return "N/A";
}

// ==================== FLASH STORAGE (NVS) ====================
void saveConfigToFlash() {
  preferences.begin("sort_cfg", false);
  preferences.putInt("mode", (int)activeSortMode);
  for (int i = 0; i < 4; i++) {
    String keyC = "c" + String(i);
    String keyS = "s" + String(i);
    preferences.putInt(keyC.c_str(), (int)activeBuckets[i].color);
    preferences.putInt(keyS.c_str(), (int)activeBuckets[i].shape);
  }
  preferences.end();
}

void loadConfigFromFlash() {
  preferences.begin("sort_cfg", true);
  activeSortMode = (SortMode)preferences.getInt("mode", 0);
  for (int i = 0; i < 4; i++) {
    String keyC = "c" + String(i);
    String keyS = "s" + String(i);
    activeBuckets[i].color = (UIObjectColor)preferences.getInt(keyC.c_str(), 0);
    activeBuckets[i].shape = (UIObjectShape)preferences.getInt(keyS.c_str(), 0);
  }
  preferences.end();
}

// ==================== ABSOLUTE DEGREE STEPPER DRIVER ====================
void setStepperToBucket(int bucket) {
  int targetDegrees = bucket * 72;
  long targetStepPosition = bucket * STEPS_FOR_72_DEG;

  if (stepper2.currentPosition() == targetStepPosition) {
    Serial.printf("[STEPPER] Stepper already at Bucket %d (%d deg)\n", bucket, targetDegrees);
    return;
  }

  Serial.printf("[STEPPER] Rotating Stepper 2 to Bucket %d (%d deg / Target Step: %ld)...\n", 
                bucket, targetDegrees, targetStepPosition);

  // Synchronous movement ensures all steps finish before conveyor resumes
  stepper2.runToNewPosition(targetStepPosition);
  currentBucket = bucket;
}

// ==================== UDP CONFIG & ACKNOWLEDGEMENT ====================
void checkIncomingConfig() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  char buf[255];
  int len = udp.read(buf, 255);
  if (len <= 0) return;
  buf[len] = 0;

  String data = String(buf);
  data.trim();

  if (data.startsWith("CFG,")) {
    data.remove(0, 4);
    int values[9];
    int idx = 0;

    while (data.length() > 0 && idx < 9) {
      int commaPos = data.indexOf(',');
      if (commaPos == -1) {
        values[idx++] = data.toInt();
        break;
      }
      values[idx++] = data.substring(0, commaPos).toInt();
      data.remove(0, commaPos + 1);
    }

    if (idx == 9) {
      activeSortMode = (SortMode)values[0];
      for (int i = 0; i < 4; i++) {
        activeBuckets[i].color = (UIObjectColor)values[1 + (i * 2)];
        activeBuckets[i].shape = (UIObjectShape)values[2 + (i * 2)];
      }

      saveConfigToFlash();

      udp.beginPacket(udp.remoteIP(), UDP_PORT);
      udp.print("ACK_CFG");
      udp.endPacket();

      Serial.println("[UDP CONFIG] Sorting rules updated and saved to NVS!");
    }
  }
}

int calculateTargetBucket(UIObjectColor detectedColor, UIObjectShape detectedShape) {
  for (int i = 0; i < 4; i++) {
    switch (activeSortMode) {
      case MODE_COLOR:
        if (detectedColor != UI_COLOR_UNKNOWN && detectedColor == activeBuckets[i].color) return i + 1;
        break;
      case MODE_SHAPE:
        if (detectedShape != UI_SHAPE_UNKNOWN && detectedShape == activeBuckets[i].shape) return i + 1;
        break;
      case MODE_BOTH:
        if (detectedColor != UI_COLOR_UNKNOWN && detectedShape != UI_SHAPE_UNKNOWN) {
          if (detectedColor == activeBuckets[i].color && detectedShape == activeBuckets[i].shape) return i + 1;
        }
        break;
    }
  }
  return 0; // Default / Unmatched Bucket
}

void sendResultToMaster(String colorStr, String shapeStr, int bucket) {
  String payload = "RES," + colorStr + "," + shapeStr + "," + String(bucket);

  udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
  udp.print(payload);
  udp.endPacket();
}

// ==================== TJPG DECODER CALLBACK ====================
static bool tjpgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  for (int j = 0; j < h; j++) {
    const int py = y + j;
    if (py < 0 || py >= GRID_H) continue;
    const int rowSrc = j * w;
    const int rowDst = py * GRID_W;
    for (int i = 0; i < w; i++) {
      const int px = x + i;
      if (px < 0 || px >= GRID_W) continue;
      rgbGrid[rowDst + px] = bitmap[rowSrc + i];
    }
  }
  return true;
}

// ==================== CONNECTED COMPONENTS ====================
static bool keepLargestComponent() {
  static const int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
  static const int dy[8] = { 0, 1, 1,  1,  0, -1, -1, -1 };

  int      bestSeed = -1;
  uint32_t bestSize = 0;

  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (mask[i] != 1) continue;

    int head = 0, tail = 0;
    ccQueue[tail++] = (uint16_t)i;
    mask[i] = 2;

    uint32_t size = 0;
    while (head < tail) {
      const int idx = ccQueue[head++];
      size++;
      const int x = idx % GRID_W;
      const int y = idx / GRID_W;

      for (int k = 0; k < 8; k++) {
        const int nx = x + dx[k];
        const int ny = y + dy[k];
        if (nx < 0 || ny < 0 || nx >= GRID_W || ny >= GRID_H) continue;
        const int ni = ny * GRID_W + nx;
        if (mask[ni] == 1) {
          mask[ni] = 2;
          ccQueue[tail++] = (uint16_t)ni;
        }
      }
    }

    if (size > bestSize) { bestSize = size; bestSeed = i; }
  }

  if (bestSeed < 0 || bestSize < MIN_OBJECT_PIXELS) return false;

  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (mask[i] == 2) mask[i] = 1;
  }

  {
    int head = 0, tail = 0;
    ccQueue[tail++] = (uint16_t)bestSeed;
    mask[bestSeed] = 2;

    while (head < tail) {
      const int idx = ccQueue[head++];
      const int x = idx % GRID_W;
      const int y = idx / GRID_W;

      for (int k = 0; k < 8; k++) {
        const int nx = x + dx[k];
        const int ny = y + dy[k];
        if (nx < 0 || ny < 0 || nx >= GRID_W || ny >= GRID_H) continue;
        const int ni = ny * GRID_W + nx;
        if (mask[ni] == 1) {
          mask[ni] = 2;
          ccQueue[tail++] = (uint16_t)ni;
        }
      }
    }
  }

  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (mask[i] == 1) mask[i] = 0;
    else if (mask[i] == 2) mask[i] = 1;
  }

  return true;
}

// ==================== MASK BUILDER & BOUNDS ====================
static bool buildMask() {
  uint32_t rawCount = 0;
  for (int y = 0; y < GRID_H; y++) {
    const int rowBase = y * GRID_W;
    for (int x = 0; x < GRID_W; x++) {
      const int idx = rowBase + x;
      uint8_t r, g, b;
      unpackRGB(rgbGrid[idx], r, g, b);
      mask[idx] = isColoured(r, g, b) ? 1 : 0;
      if (mask[idx]) rawCount++;
    }
  }

  if (rawCount < MIN_OBJECT_PIXELS) return false;
  if (!keepLargestComponent()) return false;

  objCount = 0;
  objMinX = GRID_W; objMaxX = -1;
  objMinY = GRID_H; objMaxY = -1;
  double sumX = 0, sumY = 0;

  pN = {0, GRID_H};
  pS = {0, -1};
  pW = {GRID_W, 0};
  pE = {-1, 0};

  for (int y = 0; y < GRID_H; y++) {
    const int rowBase = y * GRID_W;
    for (int x = 0; x < GRID_W; x++) {
      const int idx = rowBase + x;
      if (!mask[idx]) continue;

      objCount++;
      sumX += x; sumY += y;
      if (x < objMinX) objMinX = x;
      if (x > objMaxX) objMaxX = x;
      if (y < objMinY) objMinY = y;
      if (y > objMaxY) objMaxY = y;

      if (y < pN.y) { pN.x = x; pN.y = y; }
      if (y > pS.y) { pS.x = x; pS.y = y; }
      if (x < pW.x) { pW.x = x; pW.y = y; }
      if (x > pE.x) { pE.x = x; pE.y = y; }
    }
  }

  if (objCount < MIN_OBJECT_PIXELS) return false;
  objCx = sumX / objCount;
  objCy = sumY / objCount;
  return true;
}

// ==================== AREA CALCULATIONS ====================
static float areaRect() {
  const float wAABB = (float)(objMaxX - objMinX + 1);
  const float hAABB = (float)(objMaxY - objMinY + 1);
  const float areaAABB = wAABB * hAABB;

  const float dNW = distPt(pN, pW);
  if (dNW < 6.0f || (pN.x == pW.x && pN.y == pW.y)) return areaAABB;

  const float a = distPt(pN, pW);
  const float b = distPt(pW, pS);
  if (a < 1.0f || b < 1.0f) return areaAABB;

  const float areaRotated = a * b;
  return (areaRotated > areaAABB) ? areaAABB : areaRotated;
}

static float areaCircle() {
  const float dV = distPt(pN, pS);
  const float dH = distPt(pW, pE);
  if (dV < 1.0f || dH < 1.0f) return -1.0f;
  return (float)M_PI * dV * dH * 0.25f;
}

static float area3Pts(Pt p1, Pt p2, Pt p3) {
  const float a = distPt(p1, p2);
  const float b = distPt(p2, p3);
  const float c = distPt(p3, p1);
  const float s = (a + b + c) * 0.5f;
  const float v = s * (s - a) * (s - b) * (s - c);
  return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

static float areaTriangle() {
  const Pt raw[4] = { pN, pS, pW, pE };
  static const int combos[4][3] = { {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3} };

  float maxA = 0.0f;
  for (int i = 0; i < 4; i++) {
    const float a = area3Pts(raw[combos[i][0]], raw[combos[i][1]], raw[combos[i][2]]);
    if (a > maxA) maxA = a;
  }
  return (maxA > 0.0f) ? maxA : -1.0f;
}

// ==================== CONTOUR & CORNER TRACING ====================
static int traceContour() {
  int startX = -1, startY = -1;
  for (int y = 0; y < GRID_H && startX < 0; y++) {
    const int rowBase = y * GRID_W;
    for (int x = 0; x < GRID_W; x++) {
      if (mask[rowBase + x]) { startX = x; startY = y; break; }
    }
  }
  if (startX < 0) return 0;

  static const int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
  static const int dy[8] = { 0, 1, 1,  1,  0, -1, -1, -1 };

  int n = 0;
  int x = startX, y = startY;
  int dir = 7;

  do {
    if (n < MAX_CONTOUR_POINTS) {
      contourX[n] = (int16_t)x;
      contourY[n] = (int16_t)y;
    }
    n++;

    int found = -1;
    for (int k = 0; k < 8; k++) {
      const int d = (dir + 5 + k) & 7;
      const int nx = x + dx[d];
      const int ny = y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_W || ny >= GRID_H) continue;
      if (mask[ny * GRID_W + nx]) { found = d; x = nx; y = ny; break; }
    }
    if (found < 0) break;
    dir = found;
  } while (!(x == startX && y == startY) && n < MAX_CONTOUR_POINTS);

  return (n < MAX_CONTOUR_POINTS) ? n : 0;
}

static int countCorners(int n) {
  if (n < 2 * CORNER_WINDOW + 4) return 0;

  int   corners  = 0;
  bool inCorner = false;
  int   lastIdx  = -1000;

  for (int i = 0; i < n; i++) {
    const int prev = (i - CORNER_WINDOW + n) % n;
    const int next = (i + CORNER_WINDOW) % n;

    const float v1x = (float)(contourX[i]    - contourX[prev]);
    const float v1y = (float)(contourY[i]    - contourY[prev]);
    const float v2x = (float)(contourX[next] - contourX[i]);
    const float v2y = (float)(contourY[next] - contourY[i]);

    const float l1 = sqrtf(v1x*v1x + v1y*v1y);
    const float l2 = sqrtf(v2x*v2x + v2y*v2y);
    if (l1 < 0.5f || l2 < 0.5f) continue;

    float dot = (v1x*v2x + v1y*v2y) / (l1 * l2);
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;
    const float angle = acosf(dot) * 180.0f / (float)M_PI;

    if (angle > CORNER_ANGLE_DEG && !inCorner) {
      if (i - lastIdx >= CORNER_MIN_SEPARATION || lastIdx < 0) {
        corners++;
        lastIdx  = i;
        inCorner = true;
      }
    } else if (angle < CORNER_RELEASE_DEG) {
      inCorner = false;
    }
  }
  return corners;
}

// ==================== SHAPE CLASSIFIER ====================
static const char* classifyShape(ShapeDebug* dbg = nullptr) {
  const float actual = (float)objCount;

  const float aR = areaRect();
  const float aC = areaCircle();
  const float aT = areaTriangle();

  const float eR = (aR > 0.0f) ? fabsf(aR - actual) / actual : 999.0f;
  const float eC = (aC > 0.0f) ? fabsf(aC - actual) / actual : 999.0f;
  const float eT = (aT > 0.0f) ? fabsf(aT - actual) / actual : 999.0f;

  char  areaVote = 'C';
  float bestErr  = eC;
  if (eR < bestErr) { bestErr = eR; areaVote = 'R'; }
  if (eT < bestErr) { bestErr = eT; areaVote = 'T'; }

  const int n = traceContour();
  const int corners = countCorners(n);
  char cornerVote = 'C';
  if (corners >= 4)      cornerVote = 'R';
  else if (corners == 3) cornerVote = 'T';

  char finalVote;
  if (corners == 4 && areaVote == 'T') {
    finalVote = 'T';
  } else {
    finalVote = cornerVote;
  }

  if (dbg) {
    dbg->areaR = aR; dbg->areaC = aC; dbg->areaT = aT;
    dbg->actual = actual; dbg->corners = corners;
    dbg->agree = (areaVote == cornerVote);
    dbg->areaVote = areaVote; dbg->cornerVote = cornerVote;
  }

  switch (finalVote) {
    case 'R': return "RECTANGLE";
    case 'T': return "TRIANGLE";
    case 'C': return "CIRCLE";
    default:  return "UNKNOWN";
  }
}

// ==================== ANALYSIS & PROCESS EXECUTION ====================
void analyzeAndProcess() {
  double sumR = 0, sumG = 0, sumB = 0;
  for (int i = 0; i < TOTAL_PIXELS; i++) {
    if (!mask[i]) continue;
    uint8_t r, g, b;
    unpackRGB(rgbGrid[i], r, g, b);
    sumR += r; sumG += g; sumB += b;
  }
  const uint8_t avgR = (uint8_t)(sumR / objCount);
  const uint8_t avgG = (uint8_t)(sumG / objCount);
  const uint8_t avgB = (uint8_t)(sumB / objCount);

  const char* rawColorStr = classifyColor(avgR, avgG, avgB);

  ShapeDebug dbg;
  const char* rawShapeStr = classifyShape(&dbg);

  UIObjectColor mappedUiColor = mapStrToUIColor(rawColorStr);
  UIObjectShape mappedUiShape = mapStrToUIShape(rawShapeStr);
  const char* shapeAbbr = getShapeAbbr(rawShapeStr);

  int targetBucket = calculateTargetBucket(mappedUiColor, mappedUiShape);

  // Print full classification details to Serial Monitor
  Serial.println("\n==========================================");
  Serial.println("         VISION SCAN ANALYSIS             ");
  Serial.println("==========================================");
  Serial.printf("  DETECTED COLOR : %s\n", rawColorStr);
  Serial.printf("  DETECTED SHAPE : %s (%s)\n", rawShapeStr, shapeAbbr);
  Serial.printf("  TARGET BUCKET  : BUCKET %d (%d deg)\n", targetBucket, targetBucket * 72);
  Serial.println("==========================================\n");

  // Rotate stepper motor directly
  setStepperToBucket(targetBucket);

  // Broadcast payload to Master ESP32 via UDP
  sendResultToMaster(String(rawColorStr), String(shapeAbbr), targetBucket);
}

// ==================== RAW SOCKET IMAGE FETCHING ====================
bool fetchAndProcessFrame() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP ERROR] WiFi is disconnected!");
    return false;
  }

  WiFiClient client;
  if (!client.connect(CAM_IP, CAM_PORT)) {
    Serial.println("[HTTP ERROR] Connection to camera failed! Check IP or close browser streams.");
    return false;
  }

  client.printf("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", CAM_PATH, CAM_IP);

  unsigned long startWait = millis();
  while (!client.available()) {
    if (millis() - startWait > HTTP_TIMEOUT_MS) {
      Serial.println("[HTTP ERROR] Camera response timeout!");
      client.stop();
      return false;
    }
    delay(1);
  }

  int contentLength = -1;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
    
    line.toLowerCase();
    if (line.startsWith("content-length:")) {
      contentLength = line.substring(15).toInt();
    }
  }

  if (contentLength <= 0 || contentLength > MAX_JPG_BYTES) {
    Serial.println("[HTTP ERROR] Invalid image Content-Length.");
    client.stop();
    return false;
  }

  uint8_t* jpgBuf = (uint8_t*)malloc(contentLength);
  if (!jpgBuf) {
    Serial.println("[MEMORY ERROR] Buffer allocation failed.");
    client.stop();
    return false;
  }

  size_t totalRead = 0;
  startWait = millis();
  while (totalRead < (size_t)contentLength && (client.connected() || client.available())) {
    int availableBytes = client.available();
    if (availableBytes > 0) {
      int readNow = client.read(jpgBuf + totalRead, contentLength - totalRead);
      if (readNow > 0) {
        totalRead += readNow;
        startWait = millis();
      }
    } else {
      if (millis() - startWait > HTTP_TIMEOUT_MS) break;
      delay(1);
    }
  }

  client.stop();

  if (totalRead != (size_t)contentLength) {
    Serial.println("[HTTP ERROR] Incomplete image download.");
    free(jpgBuf);
    return false;
  }

  // Decode JPEG buffer into RGB grid
  TJpgDec.drawJpg(0, 0, jpgBuf, contentLength);
  free(jpgBuf);

  // Process pixel mask
  if (!buildMask()) {
    Serial.println("[VISION] Frame captured, but no valid target object detected.");
    if (currentBucket != 0) {
      Serial.println("[VISION] Resetting stepper to Bucket 0 (0 deg)");
      setStepperToBucket(0);
      sendResultToMaster("NONE", "NONE", 0);
    }
    return false;
  }

  // Run full vision analysis and target calculation
  analyzeAndProcess();
  return true;
}

// ==================== DYNAMIC BUFFER ALLOCATION ====================
static bool allocateBuffers() {
  rgbGrid  = (uint16_t*)malloc(TOTAL_PIXELS * sizeof(uint16_t));
  mask     = (uint8_t*) malloc(TOTAL_PIXELS);
  ccQueue  = (uint16_t*)malloc(TOTAL_PIXELS * sizeof(uint16_t));
  contourX = (int16_t*) malloc(MAX_CONTOUR_POINTS * sizeof(int16_t));
  contourY = (int16_t*) malloc(MAX_CONTOUR_POINTS * sizeof(int16_t));

  return (rgbGrid && mask && ccQueue && contourX && contourY);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- [BOOT] Vision Sorter Starting ---");

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);

  if (!allocateBuffers()) {
    Serial.println("[FATAL ERROR] Memory allocation failed!");
    while (true) delay(1000);
  }

  initHardwareConveyor();

  stepper2.setMaxSpeed(800.0);
  stepper2.setAcceleration(400.0);
  stepper2.setCurrentPosition(0); 
  currentBucket = 0;

  TJpgDec.setJpgScale(JPEG_SCALE);
  TJpgDec.setCallback(tjpgOutput);

  loadConfigFromFlash();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to %s...", WIFI_SSID);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.println("--- [READY] IR Sensors Active & Scanning Loop Started ---\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  checkIncomingConfig();

  if (isObjectDetected()) {
    Serial.println("\n------------------------------------------");
    Serial.println("[IR SENSOR] Object Detected!");

    // 1. Always stop conveyor immediately
    stopConveyor();

    // 2. Pause 150ms to settle mechanical vibrations before photo
    delay(150);

    // 3. Scan frame: camera photo -> image decode -> vision analysis -> stepper command
    Serial.println("[VISION] Fetching frame & scanning object...");
    fetchAndProcessFrame();

    // 4. Immediately restart conveyor so object travels to drop zone
    startConveyor();

    // 5. Keep belt moving while object exits IR beam area
    while (isObjectDetected()) {
      delay(1);
    }
    Serial.println("------------------------------------------\n");
  }

  delay(10);
}
