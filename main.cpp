#include "mbed.h"
#include "stm32f4xx_hal.h"
#include "LCD_DISCO_F429ZI.h"
#include "TS_DISCO_F429ZI.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <numeric>


// // Gyroscope and LCD setup
SPI spi(PF_9, PF_8, PF_7);   // MOSI, MISO, SCLK
DigitalOut cs(PC_1);          // Chip Select
LCD_DISCO_F429ZI lcd;         // LCD object

// Constants
const float MOTION_THRESHOLD = 0.5f;         // Threshold in rad/s for motion detection
const int STABILIZATION_DELAY_MS = 1000;     // Time below threshold to consider stabilization
const int FILTER_SIZE = 5;                   // Number of samples for the moving average
const float SIMILARITY_THRESHOLD = 0.7f;     // Similarity threshold for comparison
const size_t RESAMPLE_POINTS = 20;           // Fixed number of points for resampling

// Timer for motion stabilization and data capture
Timer stabilizationTimer;
Timer captureTimer;

// Buffer for moving average
std::vector<float> magnitudeBuffer(FILTER_SIZE, 0.0f);

// Data structure for motion capture
struct MotionData {
    float gx, gy, gz;     // Angular velocities (rad/s)
    uint32_t timestamp;   // Timestamp in milliseconds

    MotionData(float x, float y, float z, uint32_t t)
        : gx(x), gy(y), gz(z), timestamp(t) {}
};

std::vector<MotionData> capturedData; // Captured data for the current motion


std::vector<MotionData> password = {
    {1.166, 0.22175, 0.2305, 4450},
    {0.66875, 0.069, 0.137, 103998},
    {0.3545, 0.201, 0.0585, 203998},
    {-0.16575, 0.0885, 0.15575, 303998},
    {-0.25, 0.16, 0.25125, 403998},
    {-0.70075, 0.42225, 0.2435, 503998},
    {-0.38825, 0.23975, 0.18275, 603998},
    {0.493, 0.09325, 0.249, 703998},
    {0.50175, 0.1735, 0.6525, 803998},
    {0.3595, 0.23225, 0.7375, 903998},
    {0.16775, 0.0495, 0.5865, 1003998},
    {0.05225, 0.174, 0.66875, 1103998},
    {0.0995, 0.103, 0.5605, 1203998},
    {0.22325, 0.113, -0.13775, 1303998},
    {0.1005, 0.1, 0.62675, 1403998},
    {0.06525, 0.07375, 0.59125, 1503998},
    {0.1875, 0.0775, 0.37975, 1603998},
    {0.0635, 0.03525, 0.07375, 1703998},
    {0.06025, 0.061, 0.03, 1803998}
};


// Function Prototypes
void configureGyro();
void readGyroData(float &gx, float &gy, float &gz);
float calculateMovingAverage(float newSample);
void startMotionCapture();
void stopMotionCapture();
std::vector<MotionData> resampleMotionData(const std::vector<MotionData> &data, size_t targetSize);
float calculateMAE(const std::vector<MotionData> &attempt, const std::vector<MotionData> &password);
void debugMotionData(const std::vector<MotionData> &data, const char *label);

// Function to configure gyroscope
void configureGyro() {
    cs = 0;
    spi.write(0x20); // CTRL_REG1
    spi.write(0x6F); // Enable X/Y/Z, power on, 100 Hz
    cs = 1;

    cs = 0;
    spi.write(0x23); // CTRL_REG4
    spi.write(0x30); // High-resolution, ±2000 dps
    cs = 1;
}

// Function to read gyroscope data
void readGyroData(float &gx, float &gy, float &gz) {
    uint8_t buffer[6];

    cs = 0;
    spi.write(0x28 | 0x80 | 0x40); // Read starting at OUT_X_L with auto-increment
    for (int i = 0; i < 6; i++) {
        buffer[i] = spi.write(0x00);
    }
    cs = 1;

    gx = ((int16_t)((buffer[1] << 8) | buffer[0])) * 0.07f * 0.0174533f; // Convert to rad/s
    gy = ((int16_t)((buffer[3] << 8) | buffer[2])) * 0.07f * 0.0174533f;
    gz = ((int16_t)((buffer[5] << 8) | buffer[4])) * 0.07f * 0.0174533f;
}

// Function to calculate moving average
float calculateMovingAverage(float newSample) {
    magnitudeBuffer.push_back(newSample);
    if (magnitudeBuffer.size() > FILTER_SIZE) {
        magnitudeBuffer.erase(magnitudeBuffer.begin());
    }
    float sum = std::accumulate(magnitudeBuffer.begin(), magnitudeBuffer.end(), 0.0f);
    return sum / magnitudeBuffer.size();
}

// Function to start motion capture
void startMotionCapture() {
    capturedData.clear(); // Clear previous data
    captureTimer.reset();
    captureTimer.start();
    lcd.Clear(LCD_COLOR_GREEN);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Capturing Motion", CENTER_MODE);
}

// Function to stop motion capture
void stopMotionCapture() {
    captureTimer.stop();
    lcd.Clear(LCD_COLOR_BLUE);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Motion Stabilized", CENTER_MODE);
}

// Function to resample motion data
std::vector<MotionData> resampleMotionData(const std::vector<MotionData> &data, size_t targetSize) {
    std::vector<MotionData> resampled;
    if (data.size() < 2 || targetSize < 2) return resampled;

    float step = static_cast<float>(data.back().timestamp - data.front().timestamp) / (targetSize - 1);

    for (size_t i = 0; i < targetSize; ++i) {
        float targetTime = data.front().timestamp + i * step;

        auto lower = std::lower_bound(data.begin(), data.end(), targetTime,
                                      [](const MotionData &point, float time) {
                                          return point.timestamp < time;
                                      });

        if (lower == data.begin()) {
            resampled.push_back(*lower);
        } else if (lower == data.end()) {
            resampled.push_back(*(lower - 1));
        } else {
            const MotionData &prev = *(lower - 1);
            const MotionData &next = *lower;

            float ratio = (targetTime - prev.timestamp) / (next.timestamp - prev.timestamp);
            resampled.emplace_back(
                prev.gx + ratio * (next.gx - prev.gx),
                prev.gy + ratio * (next.gy - prev.gy),
                prev.gz + ratio * (next.gz - prev.gz),
                static_cast<uint32_t>(targetTime)
            );
        }
    }
    return resampled;
}

// Function to calculate MAE with dynamic scaling
float calculateMAE(const std::vector<MotionData> &attempt, const std::vector<MotionData> &password) {
    float gxError = 0, gyError = 0, gzError = 0;
    float maxGx = 0, maxGy = 0, maxGz = 0;

    // Find the range of values in the password data for scaling
    for (const auto &point : password) {
        maxGx = std::max(maxGx, fabs(point.gx));
        maxGy = std::max(maxGy, fabs(point.gy));
        maxGz = std::max(maxGz, fabs(point.gz));
    }

    // Avoid division by zero
    maxGx = (maxGx == 0) ? 1 : maxGx;
    maxGy = (maxGy == 0) ? 1 : maxGy;
    maxGz = (maxGz == 0) ? 1 : maxGz;

    size_t size = attempt.size();
    for (size_t i = 0; i < size; ++i) {
        gxError += fabs(attempt[i].gx - password[i].gx) / maxGx;
        gyError += fabs(attempt[i].gy - password[i].gy) / maxGy;
        gzError += fabs(attempt[i].gz - password[i].gz) / maxGz;
    }

    return (gxError + gyError + gzError) / (3.0f * size);
}

// Function to debug motion data
void debugMotionData(const std::vector<MotionData> &data, const char *label) {
    printf("%s:\n", label);
    for (const auto &point : data) {
        int scaled_gx = static_cast<int>(point.gx * 1000); // Scale to preserve 3 decimal places
        int scaled_gy = static_cast<int>(point.gy * 1000);
        int scaled_gz = static_cast<int>(point.gz * 1000);

        printf("GX: %d.%03d, GY: %d.%03d, GZ: %d.%03d, Timestamp: %lu ms\n",
               scaled_gx / 1000, abs(scaled_gx % 1000),
               scaled_gy / 1000, abs(scaled_gy % 1000),
               scaled_gz / 1000, abs(scaled_gz % 1000),
               point.timestamp);
    }
}

// Main program
int main() {
    float gx, gy, gz;
    bool motionDetected = false;

    // Initialize SPI and LCD
    spi.format(8, 3);
    spi.frequency(1000000);
    cs = 1;

    lcd.Clear(LCD_COLOR_BLUE);
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"GYROSCOPE DATA", CENTER_MODE);

    // Configure gyroscope
    configureGyro();

    // Start the stabilization timer
    stabilizationTimer.start();

    // Main loop
    while (1) {
        readGyroData(gx, gy, gz);

        float magnitude = sqrt(gx * gx + gy * gy + gz * gz);
        float smoothedMagnitude = calculateMovingAverage(magnitude);

        if (!motionDetected && smoothedMagnitude > MOTION_THRESHOLD) {
            motionDetected = true;
            stabilizationTimer.reset();
            startMotionCapture();
        }

        if (motionDetected && smoothedMagnitude < MOTION_THRESHOLD) {
            if (stabilizationTimer.elapsed_time().count() >= STABILIZATION_DELAY_MS) {
                motionDetected = false;
                stopMotionCapture();

                // Resample data to fixed points
                auto resampledPassword = resampleMotionData(password, RESAMPLE_POINTS);
                auto resampledAttempt = resampleMotionData(capturedData, RESAMPLE_POINTS);

                // Debugging captured data and password
                debugMotionData(resampledPassword, "Password Data");
                debugMotionData(resampledAttempt, "Attempt Data");

                // Compare motion sequences
                float mae = calculateMAE(resampledAttempt, resampledPassword);
                lcd.Clear(mae < SIMILARITY_THRESHOLD ? LCD_COLOR_GREEN : LCD_COLOR_RED);
                lcd.DisplayStringAt(0, LINE(7), (uint8_t *)(mae < SIMILARITY_THRESHOLD ? "Access Granted" : "Access Denied"), CENTER_MODE);
                break; // End program after comparison
            }
        } else if (smoothedMagnitude >= MOTION_THRESHOLD) {
            stabilizationTimer.reset();
        }

        if (motionDetected) {
            capturedData.push_back(MotionData(gx, gy, gz, captureTimer.elapsed_time().count()));
        }

        ThisThread::sleep_for(100ms);
    }
}

