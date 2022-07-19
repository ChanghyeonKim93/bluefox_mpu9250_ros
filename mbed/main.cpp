#include "ThisThread.h"
#include "mbed.h"

#include "BufferedSerial.h"
#include "platform/mbed_thread.h"
// #include "serial_comm_mbed.h"
#include "mpu9250_spi.h"
#include "union_struct.h"
#include <chrono>

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// Parameter settings
#define BAUD_RATE 115200

#define SPI2_MOSI PB_15
#define SPI2_MISO PB_14
#define SPI2_SCK  PB_13
#define SPI2_CS   PB_12

#define IMU_INT   PC_9
#define TRIGGER   PB_8

// Serial
// uint8_t packet_send[256];
// uint8_t packet_recv[256];

// SerialCommunicatorMbed serial(BAUD_RATE);
// void ISR_readSerial(){
//     if(serial.tryToReadSerialBuffer()) { // packet ready!
//         int len_recv_message = 0;
//         len_recv_message = serial.getReceivedMessage(packet_recv); 

//         if( len_recv_message > 0 ) { // Successfully received the packet.
//             // do something...
//         }
//     }
// };

BufferedSerial serial(CONSOLE_TX, CONSOLE_RX);


float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float magCalibration[3] = {0,0,0};

MPU9250 imu(SPI2_CS, SPI2_MOSI, SPI2_MISO, SPI2_SCK, AFS_8G, GFS_1000DPS, MFS_16BITS);

InterruptIn imu_int(IMU_INT);
int16_t data[9];

volatile bool flag_IMU_init = false;
void workIMU(){
    if(flag_IMU_init)
        imu.read9AxisRaw(data);
};

void ISR_IMU(){
    event_queue.call(workIMU);
};

// Timer to know how much time elapses.
Timer timer;

int main() {
    // Timer starts.
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;

    serial.set_baud(BAUD_RATE);

    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));

    imu.resetMPU9250();
    ThisThread::sleep_for(1000ms);

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
        

        // serial.write(temp.bytes_,4);
        if(dt_send.count() > 19999){ // 2.5 ms interval
            // imu.read9AxisRaw(data);

            if(serial.writable()) {

            }            
            time_send_prev = time_curr;
        }
    
        // // Read data if data exists.
        // if(serial.readable()) {
        //     ISR_readSerial();
        // }
        
    }
    
    return 0;
}
