/*
 * ESP32 MLX90640 → Composite Video (Monochrome)
 * Using: Adafruit_MLX90640 + bitluni ESP32CompositeVideo
 *
 * Libraries:
 *   - "Adafruit MLX90640" by Adafruit
 *   - "Adafruit BusIO" by Adafruit (dependency)
 *   - ESP32CompositeVideo by bitluni (manual install)
 *
 * Wiring:
 *   GPIO25  → RCA centre (composite video out)
 *   GND     → RCA shield
 *   GPIO15  → MLX90640 SDA
 *   GPIO13  → MLX90640 SCL
 *   3.3V    → MLX90640 VCC
 *   GND     → MLX90640 GND
 */

#include "esp_pm.h"
#include "CompositeGraphics.h"
#include "CompositeOutput.h"
#include <Wire.h>
#include <Adafruit_MLX90640.h>

// ── Video resolution ──────────────────────────────────────────────────────────
// sendFrameHalfResolution() displays this at double size on screen,
// so 320x240 fills a ~640x480 PAL output
const int XRES = 320;
const int YRES = 240;

// ── Objects ───────────────────────────────────────────────────────────────────
CompositeGraphics graphics(XRES, YRES);
CompositeOutput composite(CompositeOutput::PAL, XRES * 2, YRES * 2);
Adafruit_MLX90640 mlx;

// ── MLX data ──────────────────────────────────────────────────────────────────
#define MLX_W 32
#define MLX_H 24

float mlxTemp[MLX_W * MLX_H];      // written by sensor read
float displayTemp[MLX_W * MLX_H];  // read by draw, copied under mutex
volatile float sceneMin = 20.0f;
volatile float sceneMax = 35.0f;
volatile bool newFrameReady = false;

SemaphoreHandle_t frameMutex;

// ── Core 0: video output task ─────────────────────────────────────────────────
// Just hammers out frames as fast as possible — this is exactly what
// bitluni's example does
void compositeCore(void* data) {
  while (true) {
    composite.sendFrameHalfResolution(&graphics.frame);
  }
}

// ── Temp → greyscale 0..54 ────────────────────────────────────────────────────
inline uint8_t tempToGray(float t, float tMin, float tMax) {
  if (tMax <= tMin) return 0;
  float norm = (t - tMin) / (tMax - tMin);
  norm = constrain(norm, 0.0f, 1.0f);
  return (uint8_t)(norm * 54.0f);
}

// ── Draw the thermal frame into the graphics buffer ───────────────────────────
void draw() {
    graphics.begin(0);

    if (newFrameReady) {
        if (xSemaphoreTake(frameMutex, 0) == pdTRUE) {
            memcpy(displayTemp, (const void*)mlxTemp, sizeof(displayTemp));
            float tMin = sceneMin;
            float tMax = sceneMax;
            newFrameReady = false;
            xSemaphoreGive(frameMutex);

            for (int py = 0; py < YRES; py++) {
                for (int px = 0; px < XRES; px++) {

                    // Map screen pixel back to a fractional position in the
                    // 32x24 thermal grid. Offset by 0.5 so screen edges align
                    // with the centres of the outermost thermal pixels.
                    float tx = ((float)px / (XRES - 1)) * (MLX_W - 1);
                    float ty = ((float)py / (YRES - 1)) * (MLX_H - 1);

                    // Integer cell coordinates and fractional parts
                    int x0 = (int)tx;
                    int y0 = (int)ty;
                    int x1 = x0 + 1;
                    int y1 = y0 + 1;

                    // Clamp to grid bounds
                    if (x1 >= MLX_W) x1 = MLX_W - 1;
                    if (y1 >= MLX_H) y1 = MLX_H - 1;

                    float fx = tx - x0;  // 0.0 .. 1.0 horizontal blend
                    float fy = ty - y0;  // 0.0 .. 1.0 vertical blend

                    // Horizontal mirror correction (same as before)
                    int mx0 = (MLX_W - 1 - x0);
                    int mx1 = (MLX_W - 1 - x1);

                    // Sample the four surrounding thermal pixels
                    float t00 = displayTemp[y0 * MLX_W + mx0];
                    float t10 = displayTemp[y0 * MLX_W + mx1];
                    float t01 = displayTemp[y1 * MLX_W + mx0];
                    float t11 = displayTemp[y1 * MLX_W + mx1];

                    // Bilinear blend: first interpolate along X, then Y
                    float top    = t00 + (t10 - t00) * fx;
                    float bottom = t01 + (t11 - t01) * fx;
                    float temp   = top + (bottom - top) * fy;

                    graphics.dot(px, py, tempToGray(temp, tMin, tMax));
                }
            }
        }
    }

    graphics.end();
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  delay(2000);

  // Lock CPU to max speed — composite video needs it
  esp_pm_lock_handle_t pmLock;
  esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "pmLock", &pmLock);
  esp_pm_lock_acquire(pmLock);

  // Init composite video
  composite.init();
  graphics.init();

  // Pin the video output loop to Core 0
  xTaskCreatePinnedToCore(compositeCore, "compositeCore", 1024, NULL, 1, NULL, 0);

  // Init sensor on Core 1 (where setup() and loop() run)
  Wire.begin(15, 13);
  Wire.setClock(800000);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found! Check wiring.");
    while (1) delay(100);
  }

  mlx.setMode(MLX90640_CHESS);
  //mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_16BIT);  //16-19
  //mlx.setRefreshRate(MLX90640_0_5_HZ);  // slowest — increase once working
  mlx.setRefreshRate(MLX90640_8_HZ);

  frameMutex = xSemaphoreCreateMutex();

  Serial.println("Ready.");
}

void loop() {
  // Read sensor
  float localTemp[MLX_W * MLX_H];
  if (mlx.getFrame(localTemp) == 0) {
    float tMin = localTemp[0], tMax = localTemp[0];
    for (int i = 1; i < MLX_W * MLX_H; i++) {
      if (localTemp[i] < tMin) tMin = localTemp[i];
      if (localTemp[i] > tMax) tMax = localTemp[i];
    }
    float margin = (tMax - tMin) * 0.05f;

    if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      memcpy((void*)mlxTemp, localTemp, sizeof(localTemp));
      sceneMin = tMin - margin;
      sceneMax = tMax + margin;
      newFrameReady = true;
      xSemaphoreGive(frameMutex);
    }
    /*Serial.print(tMin,2);
        Serial.print(" ");
        Serial.print(tMax,2);
        Serial.println();*/
  } else {
    Serial.println("MLX90640 frame error");
    delay(500);
  }

  // Draw whatever is in the buffer (video keeps outputting between sensor reads)
  draw();
}