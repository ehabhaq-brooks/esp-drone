#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "num.h"
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

typedef struct __attribute__((packed)) {
    uint64_t timestamp; // Timestamp in microseconds
    float x;  // X position in meters
    float y;  // Y position in meters
    float z;  // Z position in meters
    float vx; // Velocity in X direction in m/s
    float vy; // Velocity in Y direction in m/s
    float vz; // Velocity in Z direction in m/s
    float yaw; // Yaw angle in degrees
    float pitch; // Pitch angle in degrees
    float roll;  // Roll angle in degrees
} PhoneAbsolutePosition_velocity_Packet;

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
static positionMeasurement_t phone_absolute_position;
static velocityMeasurement_t phone_velocity;
tofMeasurement_t tofData;
static quaternion_t phone_imu_quaternion;
static bool isInit = false;
static uint8_t udp_receive_buffer[sizeof(PhoneIMUPacket) + sizeof(PhoneIMUAttitudePacket)];
struct sockaddr_in  clientAddr;

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
// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

// Turn on adaptive standard deviation for the kalman filter
static bool useAdaptiveStd = true;

// Set standard deviation flow 
// (will not work if useAdaptiveStd is on)
static float flowStdFixed = 2.0f;

#endif
static bool isMpu6050TestPassed = false;

STATIC_MEM_TASK_ALLOC(phoneImuTask, PHONEIMU_STACKSIZE);

int sock;
struct sockaddr_in listenAddr;
socklen_t addrLen = sizeof(clientAddr);

static void phoneImuTask(void* arg)
{
    
    static float previous_x = 0;
    static float previous_y = 0;


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
    PhoneAbsolutePosition_velocity_Packet positionPkt;

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
        else if (len == 24)
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

        // We received phone positioning x,y,z data
        else if (len == 44)
        {
            // //put packet in respective struct
             memcpy(&positionPkt, udp_receive_buffer, sizeof(positionPkt));
            // Get current timestamp in microseconds
            int64_t now = positionPkt.timestamp;  
            float delta_seconds = 0.1f; // Default to 0.1s if no previous timestamp available

            sensorData.interruptTimestamp = xTaskGetTickCount();

            // Compute frequency if we have a previous timestamp
            if (lastTimestamp > 0) {
                int64_t delta_us = now - lastTimestamp;
                delta_seconds = delta_us / 1000000.0f;
                freq = 1000000.0f / delta_us;  // Hz
            }
            lastTimestamp = now;

            phone_absolute_position.x = positionPkt.x;  //swapped to match crazyflie. crazyflie considers x as forward facing axis
            phone_absolute_position.y = positionPkt.y;  //swapped to match crazyflie.
            phone_absolute_position.z = positionPkt.z;
            phone_absolute_position.stdDev = 1.0f; // Assume a fixed standard deviation for position measurement

            phone_velocity.vx = deadband(positionPkt.vx, 0.00); //(phone_absolute_position.x - previous_x) / delta_seconds;
            phone_velocity.vy = deadband(positionPkt.vy, 0.00);//(phone_absolute_position.y - previous_y) / delta_seconds;
            phone_velocity.vz = deadband(positionPkt.vz, 0.00);

            // Update phone IMU attitude
            phone_imu_attitude.roll = positionPkt.roll; 
            phone_imu_attitude.pitch = positionPkt.pitch; // Invert pitch to match Crazyflie convention
            phone_imu_attitude.yaw = - positionPkt.yaw; //reverse sign to match with crazyflie coordinate system

            // Push to queues
            estimatorEnqueuePosition(&phone_absolute_position);
            estimatorEnqueueVelocity(&phone_velocity);
            xQueueOverwrite(phone_imu_attitude_queue, &phone_imu_attitude);
            // DEBUG_PRINTI("freq=%.2f Hz | Position: x=%.3f, y=%.3f, z=%.3f vx=%.3f, vy=%.3f yaw=%.3f",
            //           freq, phone_absolute_position.x, phone_absolute_position.y, phone_absolute_position.z
            //           , phone_velocity.vx, phone_velocity.vy,phone_imu_attitude.yaw );

            
            // Wake up stabilizer
            xSemaphoreGive(dataReady);

            previous_x = phone_absolute_position.x;
            previous_y = phone_absolute_position.y;

        }
        vTaskDelay(1); // Yield to avoid starving other tasks
    }
}

void send_reset_origin_to_phone()
{
    const char *reset_msg = "RESET";
        int err = sendto(sock, reset_msg, strlen(reset_msg), 0,
                            (struct sockaddr *)&clientAddr, addrLen);
        if (err < 0) {
            ESP_LOGE(DEBUG_MODULE, "Error occurred during sending: errno %d", errno);
        } else {
            ESP_LOGI(DEBUG_MODULE, "Sent reset command to phone");
        }
}
void attitude_acquire_from_phone_imu(attitude_t *state_attitude)
{

        state_attitude->yaw = phone_imu_attitude.yaw;

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
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &isInit)
PARAM_ADD(PARAM_UINT8, disable, &useFlowDisabled)
PARAM_ADD(PARAM_UINT8, adaptive, &useAdaptiveStd)
PARAM_ADD(PARAM_FLOAT, flowStdFixed, &flowStdFixed)
PARAM_GROUP_STOP(deck)


static uint32_t effect = 0;
static uint32_t neffect = 0;
PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)

