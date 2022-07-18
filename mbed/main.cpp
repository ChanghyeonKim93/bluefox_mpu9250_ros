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

#define IMU_INT   PB_8
#define TRIGGER   PC_9

// #define SPI2_MOSI PA_7
// #define SPI2_MISO PA_6
// #define SPI2_SCK  PA_5
// #define SPI2_CS   PB_12
// #define IMU_INT   PB_8
// #define TRIGGER   PC_9

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

SPI spi(SPI2_MOSI, SPI2_MISO, SPI2_SCK);
mpu9250_spi imu(spi, SPI2_CS);

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

    if(imu.init(1,BITS_DLPF_CFG_188HZ)){  //INIT the mpu9250
        serial.write("\nCouldn't initialize MPU9250 via SPI!",36);
    }
    else {
        serial.write("\nInit. the MPU9250 SPI.\n",24);
    }

    char whoami = (char)imu.whoami()+'d';   
    serial.write("who am i:", 9);
    serial.write(&whoami, 1);
    serial.write("\n",1);

    // serial.write("\nWHOAMI=0x%2x\n",imu.whoami()); //output the I2C address to know if SPI is working, it should be 104
    
    // this_thread.sleep_for(1000);
    imu.set_gyro_scale(BITS_FS_2000DPS); // set full scale range for gyros
    ThisThread::sleep_for(1s);
    imu.set_acc_scale(BITS_FS_16G); // set full scale range for accs
    ThisThread::sleep_for(1s);
    imu.AK8963_calib_Magnetometer();
    ThisThread::sleep_for(1s);

    // printf("Gyro_scale=%u\n",imu.set_gyro_scale(BITS_FS_2000DPS));    //Set full scale range for gyros
    // wait(1);  
    // printf("Acc_scale=%u\n",imu.set_acc_scale(BITS_FS_16G));          //Set full scale range for accs
    // wait(1);
    // printf("AK8963 WHIAM=0x%2x\n",imu.AK8963_whoami());
    // wait(0.1);  
    // imu.AK8963_calib_Magnetometer();

    // Loop    
    FILE* mySerialFile = fdopen(&serial, "r+");
    while (true) {
        // Write if writable.
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> dt_send = time_curr-time_send_prev;

        imu.read_all();
        FLOAT_UNION temp;
        temp.float_ = imu.Temperature;
        // temp.float_ = imu.gyroscope_data[0];
        // fprintf(mySerialFile, "%0.3f,\n",  imu.Temperature);
        serial.write(temp.bytes_,4);
        serial.write("\n",1);
        ThisThread::sleep_for(100ms);
        // if(dt_send.count() > 2499){ // 2.5 ms interval
        //     if(serial.writable()) {

        //     }            
        //     time_send_prev = time_curr;
        // }
    
        // // Read data if data exists.
        // if(serial.readable()) {
        //     ISR_readSerial();
        // }
        
    }
    
    return 0;
}
