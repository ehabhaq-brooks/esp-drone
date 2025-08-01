#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"

#include "sensors_phone_imu.h"
#include "imu.h"
#include "static_mem.h"

#define DEBUG_MODULE "PHONEIMU"
#include "debug_cf.h"

typedef struct __attribute__((packed)) {
    uint64_t timestamp_ns;
    float ax, ay, az;
    float gx, gy, gz;
} PhoneIMUPacket;

#define PHONEIMU_PORT 12345
#define PHONEIMU_STACKSIZE 2048
#define PHONEIMU_PRIORITY 5

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle dataReady;

static sensorData_t sensorData;
static bool isInit = false;

STATIC_MEM_TASK_ALLOC(phoneImuTask, PHONEIMU_STACKSIZE);

static void phoneImuTask(void* arg)
{
    int sock;
    struct sockaddr_in listenAddr, clientAddr;
    socklen_t addrLen = sizeof(clientAddr);

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
    while (1) {
        int len = recvfrom(sock, &pkt, sizeof(pkt), 0,
                           (struct sockaddr*)&clientAddr, &addrLen);
        if (len == sizeof(pkt)) {
            Axis3f acc = { { pkt.ax, pkt.ay, pkt.az } };
            Axis3f gyro = { { pkt.gx, pkt.gy, pkt.gz } };

            sensorData.acc = acc;
            sensorData.gyro = gyro;
            sensorData.interruptTimestamp = pkt.timestamp_ns / 1000; // microseconds

            // Push to queues
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
            xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            xQueueOverwrite(barometerDataQueue, &sensorData.baro);

            // Wake up stabilizer
            xSemaphoreGive(dataReady);
        }

        vTaskDelay(1); // Yield to avoid starving other tasks
    }
}

bool sensorsPhoneImuTest(void)
{
    if (!isInit) {
        DEBUG_PRINT("Phone IMU not initialized");
        return false;
    }

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
    
    dataReady = xSemaphoreCreateBinary();

    STATIC_MEM_TASK_CREATE(phoneImuTask, phoneImuTask, "PHONEIMU",
                           NULL, PHONEIMU_PRIORITY);

    isInit = true;
}

void sensorsPhoneImuWaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}
