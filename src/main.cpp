#include "mbed.h"
#include "hash/SHA256.h"

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

//Serial port
Serial pc(SERIAL_TX, SERIAL_RX);

//Bitcoin initialisation
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

uint64_t* key = (uint64_t*)((int)sequence + 48);
uint64_t* nonce = (uint64_t*)((int)sequence + 56);
uint8_t hash[32];
uint32_t hash_count = 0;
uint32_t speed;

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

//temporarily inline to speed up
inline void count_speed(){
    pc.printf("Speed: %d Revolutions/s\n", speed);
}

void count_hash(){
    pc.printf("Current Hash Rate: %dH/s\n", hash_count);
    hash_count = 0;
    count_speed(); //temporarily here
}

void I1_updateMotor(){
    static Timer t;
    t.stop();
    speed = 1.0 / t.read();
    t.reset();
    t.start();
    updateMotor();
}

volatile float inputs[3];
volatile bool newData = false;

void get_input(){
    static char inBuffer[32];
    static int inCount = 0;

    while (pc.readable()) {
        char inByte = pc.getc();
        // check if end of line
        if ((inByte == 0x0D) || (inByte == 0x0A)) {
            pc.printf("If");
            inBuffer[inCount] == 0;
            float a, b, c;
            if (sscanf(inBuffer,"%f,%f,%f",&a,&b,&c) == 3) {
                inputs[0] = a;
                inputs[1] = b;
                inputs[2] = c;
                newData = true;
            }
            inCount = 0;
        }
        else {
            pc.printf("Else");
            inBuffer[inCount] = inByte;
            if (inCount < 32) {
                inCount++;
            }
        }
    }
}

//Main
int main() {

    //Initialise the serial port
    pc.printf("Welcome to our motor controller\n\r");
    pc.printf("Enter a speed...");
    
    pc.attach(&get_input);
  
    while (true) {
        if (newData){  
            newData = false;
            pc.printf("Got %.3f, %.3f, %.3f\n\r",inputs[0],inputs[1],inputs[2]);
        }
    }
    
    //Run the motor synchronisation
    // orState = motorHome();
    // pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    //set interrupts
    // I1.rise(&I1_updateMotor);
    // I1.fall(&updateMotor);
    // I2.rise(&updateMotor);
    // I2.fall(&updateMotor);
    // I3.rise(&updateMotor);
    // I3.fall(&updateMotor);

    //setup timer
    // Ticker t;
    // t.attach(&count_hash, 1.0);

    // updateMotor();
    // while (1) {
    //     SHA256::computeHash(hash, sequence, 64);
    //     //successful nonce
    //     if((hash[0] == 0) && (hash[1] == 0)) pc.printf("Nonce found: %16x\n", *nonce);
    //     *nonce += 1;
    //     hash_count++;
    // }
}
