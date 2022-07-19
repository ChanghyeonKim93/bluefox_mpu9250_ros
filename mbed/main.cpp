#include "ThisThread.h"
#include "mbed.h"

#include "BufferedSerial.h"
#include "platform/mbed_thread.h"
#include "serial_comm_mbed.h"
#include "mpu9250_spi.h"
#include "union_struct.h"
#include <chrono>

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// Parameter settings
#define BAUD_RATE 921600

#define SPI2_MOSI   PB_15
#define SPI2_MISO   PB_14
#define SPI2_SCK    PB_13
#define SPI2_CS     PB_12

#define PIN_IMU_INT PC_9
#define PIN_TRIGGER PB_8

// IMU
MPU9250 imu(SPI2_CS, SPI2_MOSI, SPI2_MISO, SPI2_SCK, AFS_8G, GFS_1000DPS, MFS_16BITS);

InterruptIn imu_int(PIN_IMU_INT);

int16_t data[9];
SHORT_UNION acc_short[3];
SHORT_UNION gyro_short[3];
SHORT_UNION mag_short[3];

volatile bool flag_IMU_init  = false;
volatile bool flag_imu_ready = false;

void workfunction_getIMU(){
    if(flag_IMU_init){
        imu.read9AxisRaw(data);
        acc_short[0].ushort_ = data[0];
        acc_short[1].ushort_ = data[1];
        acc_short[2].ushort_ = data[2];

        gyro_short[0].ushort_ = data[3];
        gyro_short[1].ushort_ = data[4];
        gyro_short[2].ushort_ = data[5];

        mag_short[0].ushort_ = data[6];
        mag_short[1].ushort_ = data[7];
        mag_short[2].ushort_ = data[8];

        flag_imu_ready = true;
    }
};
void ISR_IMU(){
    event_queue.call(workfunction_getIMU);
};


// Serial
uint8_t packet_send[256];
uint8_t packet_recv[256];
SerialCommunicatorMbed serial(BAUD_RATE);

void workfunction_readSerial(){
    if(serial.tryToReadSerialBuffer()) { // packet ready!
        int len_recv_message = 0;
        len_recv_message = serial.getReceivedMessage(packet_recv); 

        if( len_recv_message > 0 ) { // Successfully received the packet.
            // do something...
        }
    }
};
void tryToReadSerial(){
    event_queue.call(workfunction_readSerial);
};

// Timer to know how much time elapses.
Timer timer;

int main() {
    // Timer starts.
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;

    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));

    // Start the MPU-9250 
    imu.resetMPU9250();
    ThisThread::sleep_for(1000ms);

    float gyroBias[3]  = {0, 0, 0};
    float accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
    float magCalibration[3] = {0,0,0};

    imu.calibrateMPU9250(gyroBias, accelBias);
    ThisThread::sleep_for(1000ms);

    imu.initMPU9250();
    ThisThread::sleep_for(1000ms);
    
    imu.initAK8963(magCalibration);
    ThisThread::sleep_for(1000ms);
    
    // Start interrupt at rising edge.
    imu_int.rise(ISR_IMU);

    flag_IMU_init = true;

    while (true) {
        // Write if writable.
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> dt_send = time_curr-time_send_prev;
        
        if(flag_imu_ready){
            if(serial.writable()) {
                flag_imu_ready = false;
                packet_send[0]  = acc_short[0].bytes_[0];
                packet_send[1]  = acc_short[0].bytes_[1];
                packet_send[2]  = acc_short[1].bytes_[0];
                packet_send[3]  = acc_short[1].bytes_[1];
                packet_send[4]  = acc_short[2].bytes_[0];
                packet_send[5]  = acc_short[2].bytes_[1];
                
                packet_send[6]  = gyro_short[0].bytes_[0];
                packet_send[7]  = gyro_short[0].bytes_[1];
                packet_send[8]  = gyro_short[1].bytes_[0];
                packet_send[9]  = gyro_short[1].bytes_[1];
                packet_send[10] = gyro_short[2].bytes_[0];
                packet_send[11] = gyro_short[2].bytes_[1];

                packet_send[12] = mag_short[0].bytes_[0];
                packet_send[13] = mag_short[0].bytes_[1];
                packet_send[14] = mag_short[1].bytes_[0];
                packet_send[15] = mag_short[1].bytes_[1];
                packet_send[16] = mag_short[2].bytes_[0];
                packet_send[17] = mag_short[2].bytes_[1];

                serial.send_withChecksum(packet_send, 18);
            }
            time_send_prev = time_curr;
        }            
    
        // Read data if data exists.
        if(serial.readable()) {
            tryToReadSerial();
        }
        
    }
    
    return 0;
}
