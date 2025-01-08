#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"

// Constants
const int BUZZER_PIN = 23;
const int enc_chan_a = 36;
const int enc_chan_b = 37;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000 / UPDATE_RATE;

// Buzzer and sensors
BlinkBuzz bb({BUZZER_PIN}, 1, true);
VN_100 vn(&SPI, 10);
mmfs::DPS310 baro1;
mmfs::MS5611 baro2;
mmfs::BMI088andLIS3MDL airbrake_imu;
mmfs::MAX_M10S gps;
mmfs::Sensor* airbrake_sensors[] = {&baro1, &baro2, &airbrake_imu, &gps, &vn};

// Airbrake State
AirbrakeKF kf;
AirbrakeState AIRBRAKE(airbrake_sensors, 5, &kf, BUZZER_PIN);

// MMFS
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM* psram;

void initializeHardware() {
    pinMode(brk_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(brk_pin, HIGH);
    digitalWrite(dir_pin, LOW);

    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

    if (CrashReport) Serial.println(CrashReport);
}

void checkMemory() {
    if (!logger.isSdCardReady()) {
        logger.recordLogData(mmfs::INFO_, "SD Card Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else {
        bb.onoff(BUZZER_PIN, 1000, 1);
    }

    if (!logger.isPsramReady()) {
        logger.recordLogData(mmfs::INFO_, "PSRAM Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else {
        bb.onoff(BUZZER_PIN, 1000, 1);
    }
}

void initializeSensors() {
    if (!AIRBRAKE.init()) {
        logger.recordLogData(mmfs::INFO_, "State Failed to Completely Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else {
        bb.onoff(BUZZER_PIN, 1000, 1);
        baro1.setBiasCorrectionMode(true);
        baro2.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    }
}

void setup() {
    Serial.begin(115200);
    initializeHardware();

    psram = new mmfs::PSRAM();
    logger.init(&AIRBRAKE);
    logger.recordLogData(mmfs::INFO_, "Entering Setup");

    checkMemory();
    initializeSensors();

    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

void loop() {
    static double last = 0;
    bb.update();

    if (millis() - last < 100) return;
    last = millis();

    Serial.println(airbrake_imu.getAccelerationGlobal().z());
    AIRBRAKE.updateState();
    logger.recordFlightData();

    if (AIRBRAKE.stage == BOOST) {
        baro1.setBiasCorrectionMode(false);
        baro2.setBiasCorrectionMode(false);
        gps.setBiasCorrectionMode(false);
    }
}
