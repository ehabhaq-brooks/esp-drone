#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "param.h"
#include "log.h"

#include "zranger2.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"

#include "sensors_phone_imu.h"
#include "imu.h"
#include "static_mem.h"

#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"
#include "crtp_commander.h"

#include "estimator.h"


#define SENSORS_ENABLE_RANGE_VL53L1X
#define SENSORS_ENABLE_FLOW_PMW3901


typedef struct __attribute__((packed)) {
    uint64_t timestamp; // Timestamp in microseconds
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float depth; // Depth in centimeters
    float roll;  // Roll angle in degrees
    float pitch; // Pitch angle in degrees
    float yaw;   // Yaw angle in degrees
} PhoneIMUPacket;

typedef struct __attribute__((packed)) {
    uint64_t timestamp; // Timestamp in microseconds
    float roll;  // Roll angle in degrees
    float pitch; // Pitch angle in degrees
    float yaw;   // Yaw angle in degrees
    float depth;  // Depth in centimeters
} PhoneIMUAttitudePacket;

#define PHONEIMU_PORT 12345
#define PHONEIMU_STACKSIZE 4096
#define PHONEIMU_PRIORITY 5

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xQueueHandle phone_imu_attitude_queue;
STATIC_MEM_QUEUE_ALLOC(phone_imu_attitude_queue, 1, sizeof(attitude_t))
static xQueueHandle phone_imu_quaternion_queue;
STATIC_MEM_QUEUE_ALLOC(phone_imu_quaternion_queue, 1, sizeof(quaternion_t));

static xSemaphoreHandle dataReady;

static sensorData_t sensorData;
static attitude_t   phone_imu_attitude;
tofMeasurement_t tofData;
static quaternion_t phone_imu_quaternion;
static bool isInit = false;
static uint8_t udp_receive_buffer[sizeof(PhoneIMUPacket) + sizeof(PhoneIMUAttitudePacket)];


static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
#ifdef SENSORS_ENABLE_RANGE_VL53L1X
static bool isVl53l1xPresent = false;
#endif
#ifdef SENSORS_ENABLE_RANGE_VL53L0X
static bool isVl53l0xPresent = false;
#endif
#ifdef SENSORS_ENABLE_FLOW_PMW3901
static bool isPmw3901Present = false;
#endif
static bool isMpu6050TestPassed = false;

STATIC_MEM_TASK_ALLOC(phoneImuTask, PHONEIMU_STACKSIZE);

static void phoneImuTask(void* arg)
{
    int sock;
    struct sockaddr_in listenAddr, clientAddr;
    socklen_t addrLen = sizeof(clientAddr);

    int64_t lastTimestamp = 0;
    float freq = 0;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        DEBUG_PRINTE("Socket create failed");
        vTaskDelete(NULL);
        return;
    }

    memset(&listenAddr, 0, sizeof(listenAddr));
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_port = htons(PHONEIMU_PORT);
    listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr*)&listenAddr, sizeof(listenAddr)) < 0) {
        DEBUG_PRINTE("Socket bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    DEBUG_PRINTI("Phone IMU listening on UDP port %d", PHONEIMU_PORT);

    PhoneIMUPacket pkt;
    PhoneIMUAttitudePacket attitudePkt;
    while (1) {
        int len = recvfrom(sock, &udp_receive_buffer, sizeof(udp_receive_buffer), 0,
                           (struct sockaddr*)&clientAddr, &addrLen);
        
        //ESP_LOGI(DEBUG_MODULE, "Received %d bytes from UDP", len);
        // We received acclerometer and gyro data
        if (len == 60) {
            // //put packet in respective struct
             memcpy(&pkt, udp_receive_buffer, sizeof(PhoneIMUPacket));
            // Get current timestamp in microseconds
            int64_t now = pkt.timestamp;  
            
            sensorData.interruptTimestamp = xTaskGetTickCount();

            // Compute frequency if we have a previous timestamp
            if (lastTimestamp > 0) {
                int64_t delta_us = now - lastTimestamp;
                freq = 1000000.0f / delta_us;  // Hz
            }
            lastTimestamp = now;

            //Axis3f acc = { { pkt.ax * 9.8, pkt.ay * 9.8, pkt.az * 9.8 } };
            // to reverse the z-axis you have to reverse the corresponding opposite gyroscope axis as well
            Axis3f acc = { {  (pkt.ay ) ,(pkt.ax  ), (pkt.az  * -1.0) } };
            Axis3f gyro = { { pkt.gy * 57.296 , pkt.gx * 57.296 , pkt.gz * 57.296 } };

            sensorData.acc = acc;
            sensorData.gyro = gyro;
            tofData.timestamp = xTaskGetTickCount(); // should be in processor ticks
            tofData.distance = pkt.depth * 0.01f; // Convert cm to m
            tofData.stdDev = 1.0f; // Assume a fixed standard deviation for depth measurement

            // Update phone IMU attitude
            phone_imu_attitude.roll = pkt.roll; 
            phone_imu_attitude.pitch = pkt.pitch * -1.0; // Invert pitch to match Crazyflie convention
            phone_imu_attitude.yaw = pkt.yaw;


            // Push to queues
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
            xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            xQueueOverwrite(barometerDataQueue, &sensorData.baro);
            xQueueOverwrite(phone_imu_attitude_queue, &phone_imu_attitude);
            estimatorEnqueueTOF(&tofData);

            // DEBUG_PRINTI("freq=%.2f Hz | acc: %.3f, %.3f, %.3f | gyro: %.3f, %.3f, %.3f | tof : %.3f | roll: %.2f, pitch: %.2f, yaw: %.2f",
            //           freq, acc.x, acc.y, acc.z,
            //           gyro.x, gyro.y, gyro.z, tofData.distance,
            //           phone_imu_attitude.roll, phone_imu_attitude.pitch, phone_imu_attitude.yaw);
                      
            // Wake up stabilizer
            xSemaphoreGive(dataReady);
        }

        // We received attitude data
        if (len == 24)
        {
            //put packet in respective struct
            memcpy(&attitudePkt, udp_receive_buffer, sizeof(PhoneIMUAttitudePacket));

            // Get current timestamp in microseconds
            int64_t now = attitudePkt.timestamp;
            // Compute frequency if we have a previous timestamp
            if (lastTimestamp > 0) {
                int64_t delta_us = now - lastTimestamp;
                freq = 1000000.0f / delta_us;  // Hz
            }
            lastTimestamp = now;
            
            // Update phone IMU attitude
            phone_imu_attitude.roll = attitudePkt.roll; 
            phone_imu_attitude.pitch = attitudePkt.pitch;
            phone_imu_attitude.yaw = attitudePkt.yaw;

            tofData.timestamp = xTaskGetTickCount(); // should be in processor ticks
            tofData.distance = attitudePkt.depth * 0.01f; // Convert cm to m
            tofData.stdDev = 1.0f; // Assume a fixed standard deviation for depth measurement
            // Push to attitude queue
            xQueueOverwrite(phone_imu_attitude_queue, &phone_imu_attitude);
            // Update phone IMU quaternion
            xQueueOverwrite(phone_imu_quaternion_queue, &phone_imu_quaternion);
            // Update TOF data queue
            estimatorEnqueueTOF(&tofData);
            //  DEBUG_PRINTI("freq=%.2f Hz |Attitude: roll=%.2f, pitch=%.2f, yaw=%.2f, depth=%.2f cm",
            //           freq, phone_imu_attitude.roll, phone_imu_attitude.pitch, phone_imu_attitude.yaw, attitudePkt.depth); 
            
            // Wake up stabilizer
            xSemaphoreGive(dataReady);
        }

        vTaskDelay(1); // Yield to avoid starving other tasks
    }
}

void attitude_acquire_from_phone_imu(attitude_t *state_attitude, quaternion_t *state_attitudeQuaternion)
{
    if (xQueueReceive(phone_imu_attitude_queue, state_attitude, 0) == pdTRUE) {
        // Successfully received attitude
    } else {
        // Handle error or use default values
        state_attitude->roll = 0.0f;
        state_attitude->pitch = 0.0f;
        state_attitude->yaw = 0.0f;
    }

    if (xQueueReceive(phone_imu_quaternion_queue, state_attitudeQuaternion, 0) == pdTRUE) {
        // Successfully received quaternion
    } else {
        // Handle error or use default values
        state_attitudeQuaternion->x = 0.0f;
        state_attitudeQuaternion->y = 0.0f;
        state_attitudeQuaternion->z = 0.0f;
        state_attitudeQuaternion->w = 1.0f; // Default quaternion (no rotation)
    }
}
bool sensorsPhoneImuTest(void)
{
    if (!isInit) {
        DEBUG_PRINT("Phone IMU not initialized");
        return false;
    }

    isMpu6050TestPassed = true; // Assume test passed for phone IMU
    // Assume the phone IMU is always ready and calibrated
    return true;
}

bool sensorsPhoneImuAreCalibrated()
{
    // Assume the phone IMU is pre‑calibrated
    return true;
}
bool sensorsPhoneImuManufacturingTest(void)
{
    // Assume the phone IMU is pre‑calibrated
    return true;
}

bool sensorsPhoneImuReadGyro(Axis3f* gyro)
{
    return pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0);
}

bool sensorsPhoneImuReadAcc(Axis3f* acc)
{
    return pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0);
}

bool sensorsPhoneImuReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsPhoneImuReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsPhoneImuSetAccMode(accModes accMode)
{
    // Difficult to switch mode so do nothing.
    switch (accMode) {
        case ACC_MODE_PROPTEST:
            break;
        case ACC_MODE_FLIGHT:
        default:
            break;
    }
}
void sensorsPhoneImuAcquire(sensorData_t* sensors, const uint32_t tick)
{
    sensors->acc = sensorData.acc;
    sensors->gyro = sensorData.gyro;
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}


void sensorsPhoneImuInit(void)
{
    if (isInit) return;

    accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
    gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
    magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
    barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);
    phone_imu_attitude_queue = STATIC_MEM_QUEUE_CREATE(phone_imu_attitude_queue);
    phone_imu_quaternion_queue = STATIC_MEM_QUEUE_CREATE(phone_imu_quaternion_queue);
    
    dataReady = xSemaphoreCreateBinary();

    STATIC_MEM_TASK_CREATE(phoneImuTask, phoneImuTask, "PHONEIMU",
                           NULL, PHONEIMU_PRIORITY);

    isInit = true;
}

void sensorsPhoneImuWaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

static uint8_t disable = 0;
#define PARAM_CORE (1<<5)
#define PARAM_PERSISTENT (1 << 8)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)

PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcACS37800, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &disable)
PARAM_GROUP_STOP(deck)


static uint32_t effect = 0;
static uint32_t neffect = 0;
PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)

