#include "Arduino.h"
#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"

// Buzzer
const int BUZZER_PIN = 23;
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
VN_100 vn(&SPI, 10); // Vector Nav
mmfs::DPS310 baro1; 
mmfs::MS5611 baro2;
mmfs::BMI088andLIS3MDL airbrake_imu;
mmfs::MAX_M10S gps;
mmfs::Sensor* airbrake_sensors[5] = {&baro1, &baro2, &airbrake_imu, &gps, &vn};

// Initialize Airbrake State
AirbrakeKF kf;
AirbrakeState AIRBRAKE(airbrake_sensors, 5, &kf, BUZZER_PIN);

// MMFS Stuff
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {
    // Initialize Serial and SPI
    Serial.begin(115200);
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

    if (CrashReport) Serial.println(CrashReport);

    // Immediately turn the motor off
    pinMode(brk_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(brk_pin, HIGH);
    digitalWrite(dir_pin, LOW);

    // MMFS Setup
    SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
    SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
    psram = new mmfs::PSRAM();
    logger.init(&AIRBRAKE);

    logger.recordLogData(mmfs::INFO_, "Entering Setup");

    // Check SD Card and PSRAM
    bool sdReady = logger.isSdCardReady();
    bool psramReady = logger.isPsramReady();
    bb.onoff(BUZZER_PIN, sdReady && psramReady ? 1000 : 200, 1);
    
    if (!sdReady) logger.recordLogData(mmfs::INFO_, "SD Card Failed to Initialize");
    if (!psramReady) logger.recordLogData(mmfs::INFO_, "PSRAM Failed to Initialize");

    // Initialize Sensors
    if (!AIRBRAKE.init()) {
        logger.recordLogData(mmfs::INFO_, "State Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else {
        bb.onoff(BUZZER_PIN, 1000, 1);
        baro1.setBiasCorrectionMode(true);
        baro2.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    }

    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

static double last = 0;
void loop() {
    bb.update();
    if (millis() - last < 100) return;
    last = millis();

    // Log Data
    Serial.println(airbrake_imu.getAccelerationGlobal().z());
    AIRBRAKE.updateState();
    logger.recordFlightData();

    // Manage bias correction based on stage
    if (AIRBRAKE.stage == BOOST) {
        baro1.setBiasCorrectionMode(false);
        baro2.setBiasCorrectionMode(false);
        gps.setBiasCorrectionMode(false);
    }
    
    // Uncomment to deploy airbrake
    // if (AIRBRAKE.stage == DEPLOY) AIRBRAKE.goToDegree(-10);
    // else AIRBRAKE.goToDegree(0);
}