#include "mbed.h"
#include "rtos.h"
#include "hash/SHA256.h"
#include <stdlib.h>

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards

int8_t orState = 0;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Encoder
InterruptIn channelA(CHA);
InterruptIn channelB(CHB);

//Serial port
Serial pc(SERIAL_TX, SERIAL_RX);

// ---------- BITCOIN VARIABLES ----------
uint8_t sequence[] = {
                        0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
                        0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
                        0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
                        0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
                        0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
                        0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
                     };

volatile uint64_t* key = (uint64_t*)((int)sequence + 48);
volatile uint64_t* nonce = (uint64_t*)((int)sequence + 56);
uint8_t hash[32];
uint32_t hash_count = 0;

// ---------- SPEED/ENCODER VARIABLES ----------
uint32_t speed;
int32_t encoder_state = 0;


// ---------- THREADING VARIABLES ----------
Thread t1;
Thread t2;

int mSpeed;
Mutex mSpeed_mutex;

int bKey = 0;
Mutex bKey_mutex;

volatile bool endT = true; // Shared boolean to end all threads, not sure if need to read
Mutex endT_mutex;

volatile int test = 0;

// ------------- FUNCTIONS -------------
//Set a given drive state
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }

    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);

    //Get the rotor state
    return readRotorState();
}

void updateMotor(){
    int8_t intState = readRotorState();
    motorOut((intState - orState + lead + 6) % 6); //+6 to make sure the remainder is positive
}

void updateEncoder(){
    static int32_t last_state = 0;

    int a = channelA.read();
    int b = channelB.read();

    int current_state = (a << 1) | b;
    int change;

    //check only one thing has changed
    if(((current_state ^ last_state) != 0x3) && (current_state != last_state)){
        change = last_state ^ (current_state >> 1); // 0 = cw; 1 = ccw;

        if(change == 0) change = -1;

        encoder_state += change;
    }

    last_state = current_state;
}

//temporarily inline to speed up
inline void count_speed(){
    pc.printf("Speed: %d Revolutions/s\n", speed);
}

void count_hash(){
    pc.printf("Current Hash Rate: %dH/s\n", hash_count);
    hash_count = 0;
}

void count(){
    count_hash();
    count_speed();
}

void I1_updateMotor(){
    static Timer t;
    t.stop();
    speed = 1.0 / t.read();
    t.reset();
    encoder_state = 0;
    t.start();
    updateMotor();
}

bool valid_key(const char* buff){
    bool valid = false;
    const int string_len = strlen(buff);
    for (int i = 0; i < string_len; ++i){
        if (buff[i] > 47 && buff[i] < 58){
            // 0-9
            valid = true;
        } else if (buff[i] > 64 && buff[i] < 71){
            // A-F
            valid = true;
        } else if (buff[i] > 96 && buff[i] < 103){
            // a-f
            valid = true;
        } else {
            return false;
        }
    }
    return valid;
}

void bitcoin_kernel(){
    // while (endT){
        // bKey_mutex.lock();
        // SHA256::computeHash(hash, sequence, 64);
        // // pc.printf("Key is:%d\n",bKey);
        // // Release lock
        // // bKey_mutex.unlock();
        // //successful nonce
        // if((hash[0] == 0) && (hash[1] == 0)){
        //     // pc.printf("Nonce found: %16x\n", *nonce);
        // }
        // *nonce += 1;
        // hash_count++;
        //pc.printf("Test print from thread");
        // wait(1.0);
    // }
}

void motor_speed(){
  while(true){
    // Get mutex for speed
    // mSpeed_mutex.lock();
    // Motor control stuff
    // pc.printf("Pretending to set motor speed to %d.\n\r",mSpeed);
    // Release mutex for speed
    // mSpeed_mutex.unlock();
    wait(1.0);
    // Thread::yield();
  }
}

int main() {

    //Initialise the serial port
    pc.printf("Welcome to our motor controller\n\r");
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    //set interrupts
    I1.rise(&I1_updateMotor);
    I1.fall(&updateMotor);
    I2.rise(&updateMotor);
    I2.fall(&updateMotor);
    I3.rise(&updateMotor);
    I3.fall(&updateMotor);

    channelA.rise(&updateEncoder);
    channelB.rise(&updateEncoder);
    channelA.fall(&updateEncoder);
    channelB.fall(&updateEncoder);

    //setup timer
    // Ticker t;
    // t.attach(&count, 1.0);

    pc.printf("Before\n");
    pc.printf("%i", test);
    t1.start(&bitcoin_kernel);
    pc.printf("%i", test);
    // pc.printf("1");
    t2.start(&motor_speed);
    pc.printf("2");
    wait(5);

    updateMotor();
    while (true){
        if (pc.readable()){
            pc.printf("What's next commander?\n");
            char c = pc.getc();
            char buffer[32];
            switch (c) {
                case 'R':
                case 'r':
                    pc.printf("Rotate a number of revolutions\n");
                    pc.gets(buffer, 7);
                    pc.printf("%c", c);
                    pc.printf("%s\n", buffer);
                    break;
                case 'V':
                case 'v':
                    pc.printf("Set maximum speed\n");
                    pc.gets(buffer, 7);
                    pc.printf("%c", c);
                    pc.printf("%s\n", buffer);
                    mSpeed_mutex.lock();
                    mSpeed = atof(buffer);
                    mSpeed_mutex.unlock();
                    // pc.printf("%d\n", speed);
                    break;
                case 'K':
                case 'k':
                    // K[0-9a-f]{16}
                    pc.printf("Set bitcoin key\n");
                    pc.gets(buffer, 16);
                    pc.printf("%c", c);
                    pc.printf("%s\n", buffer);
                    if (valid_key(buffer)){
                        bKey_mutex.lock();
                        bKey = (int)strtol(buffer, NULL, 16);
                        bKey_mutex.unlock();
                        pc.printf("Yay");
                    } else {
                        pc.printf("Nay");
                    }
                    break;
                case 'T':
                case 't':
                    pc.printf("Set tune\n");
                    pc.gets(buffer, 16);
                    pc.printf("%c", c);
                    pc.printf("%s\n", buffer);
                    break;
                default:
                    pc.printf("Wrong order commander, think again!\n");
                    break;
            }
        }
    }
}
