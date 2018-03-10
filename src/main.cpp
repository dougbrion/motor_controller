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

const int PWM_PERIOD=256; // us

//Encoder
InterruptIn channelA(CHA);
InterruptIn channelB(CHB);

// ---------- ENUMS ----------
enum printCodes{
    MSG,
    NEW_KEY,
    NEW_SPEED,
    NEW_TUNE,
    NEW_ROTATION,
    ROTOR_ORIGIN,
    NONCE_FOUND,
};

enum msg_types{
    NONE,
    YAY,
    NAY,
    WRONG_ORDER,
    WHATS_NEXT,
    TORQUE,
    VELOCITY
};

// ---------- SERIAL VARIABLES ----------
RawSerial pc(SERIAL_TX, SERIAL_RX);

//TODO: change data to 32 bits and put two messages on the queue for the key
typedef struct{
    printCodes code;
    uint64_t data;
 } message_t;

Mail<message_t,16> outMessages;
Queue<void, 8> charBuffer;

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

volatile uint64_t newKey;

// ---------- SPEED/ENCODER VARIABLES ----------
int32_t velocity = 0;
int16_t velocity_count = 0;
int32_t encoder_state = 0;

uint32_t cmd_torque = PWM_PERIOD;


// ---------- THREADING VARIABLES ----------
Thread serialPrint_th;
Thread decodeCommands_th;
Thread velocityCalc_th;

Mutex newKey_mutex;
// int mSpeed;
// Mutex mSpeed_mutex;
//
// int bKey = 0;
// Mutex bKey_mutex;
//
// volatile bool endT = true; // Shared boolean to end all threads, not sure if need to read
// Mutex endT_mutex;
//
// volatile int test = 0;

// ------------- FUNCTIONS -------------
//Set a given drive state
void motorOut(int8_t driveState, uint32_t pulseWidth){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L1L = 0;
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
    motorOut(0,PWM_PERIOD); // Set to 100% pwm
    wait(1.0);

    //Get the rotor state
    return readRotorState();
}

void updateMotor(){
    static int8_t last_state = 0;
    int8_t intState = readRotorState();
    // Compare with last_state
    if (!(intState==7&&last_state==0)&&((intState==0&&last_state==7) || intState>last_state)){
        velocity_count++;
    } else if ((intState==7&&last_state==0)||intState<last_state){
        velocity_count--;
    }
    last_state = intState;
    motorOut((intState - orState + lead + 6) % 6,cmd_torque); //+6 to make sure the remainder is positive
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

// checks if key is valid and updates
//TODO: update to work with int
// void valid_key(uint64_t key){
//     bool valid = false;
//     const int string_len = strlen(buff);
//     for (int i = 0; i < string_len; ++i){
//         if (buff[i] > 47 && buff[i] < 58){
//             // 0-9
//             valid = true;
//         } else if (buff[i] > 64 && buff[i] < 71){
//             // A-F
//             valid = true;
//         } else if (buff[i] > 96 && buff[i] < 103){
//             // a-f
//             valid = true;
//         } else {
//             valid = false;
//             break;
//         }
//     }
//     if (valid){
//         newKey_mutex.lock();
//
//         newKey = key;
//         queueMessage(MSG, uint64_t(YAY));
//         queueMessage(NEW_KEY, newKey);
//
//         newKey_mutex.unlock();
//     } else {
//         queueMessage(MSG, uint64_t(NAY));
//     }
// }

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

// Checks message queue and prints
void serialPrint(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;

        switch(pMessage->code){
            case MSG:
                switch(static_cast<msg_types>(pMessage->data)){
                    case YAY:
                        pc.printf("Nay!\n\r");
                        break;
                    case NAY:
                        pc.printf("Nay!\n\r");
                        break;
                    case WRONG_ORDER:
                        pc.printf("Wrong order commander, think again!\n\r");
                        break;
                    case WHATS_NEXT:
                        pc.printf("What's next commander?\n\r");
                        break;
                    case TORQUE:
                        pc.printf("Torque: %d\n\r", cmd_torque);
                        break;
                    case VELOCITY:
                        pc.printf("Velocity: %d\n\r", velocity);
                        break;
                }
            break;

            case NEW_KEY:
                pc.printf("Setting bitcoin key to 0x%016x\n\r", pMessage->data);
                break;
            case NEW_SPEED:
                pc.printf("Setting maximum speed to 0x%016x\n\r", pMessage->data);
                break;
            case NEW_TUNE:
                pc.printf("Queuing sea shanty 0x%016x\n\r", pMessage->data);
                break;
            case NEW_ROTATION:
                pc.printf("Rotating 0x%016x\n\r revolutions\n\r", pMessage->data);
                break;
            case ROTOR_ORIGIN:
                pc.printf("Rotor origin at 0x%016x\n\r", pMessage->data);
                break;
            case NONCE_FOUND:
                pc.printf("We've found ourselves a nonce! 0x%016x\n\r", pMessage->data);
                break;
        }

        outMessages.free(pMessage);
    }
}

// Queues a message to be printed later
void queueMessage(printCodes code, uint64_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
}

// When serial input interrupt is triggered -> places char into buffer to be processed later
void serialISR(){
    uint8_t newChar = pc.getc();
    charBuffer.put((void*)newChar);
}

// decodes commands in the char buffer
void decodeCommands(){
    pc.attach(&serialISR);
    char command[32];
    int index = 0;
    while(true){
        osEvent newEvent = charBuffer.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;

        if(newChar != '\r'){
            command[index] = newChar;
            index++;
        }
        else{
            command[index] = '\0';
            index = 0;

            uint64_t tmp;

            switch(command[0]) {
                //TODO: set number of rotations
                case 'R':
                    sscanf(command, "R%10llx", &tmp);
                    queueMessage(NEW_ROTATION, tmp);
                    break;
                case 'r':
                    sscanf(command, "r%10llx", &tmp);
                    queueMessage(NEW_ROTATION, tmp);
                    break;

                //TODO: set speed
                case 'V':
                    sscanf(command, "V%10llx", &tmp);
                    queueMessage(NEW_SPEED, tmp);
                    break;
                case 'v':
                    sscanf(command, "v%10llx", &tmp);
                    queueMessage(NEW_SPEED, tmp);
                    break;

                // K[0-9a-f]{16}
                case 'K':
                    sscanf(command, "K%10llx", &tmp);
                    //valid_key(tmp);
                    break;
                case 'k':

                    sscanf(command, "k%10llx", &tmp);
                    //valid_key(tmp);
                    break;

                //TODO: set tune
                case 'T':
                case 't':
                    sscanf(command, "t%lx", &cmd_torque);
                    queueMessage(MSG, cmd_torque);
                    //queueMessage(NEW_TUNE, 0);
                    break;

                default:
                    queueMessage(MSG, uint64_t(WRONG_ORDER));
                    break;
            }
            queueMessage(MSG, uint64_t(WHATS_NEXT));
        }
    }
}

/*
void signalVelocity(){
  velocityCalc_th.signal_set(0x1);//velSig = 1;
}

void velocityCalc(){
  static uint8_t iter = 0;
  
  while (true){
    // Wait until signal from signalVelocity
    velocityCalc_th.signal_wait(0x1);
    iter++;
    // velocity_count
    velocity = (velocity_count*10)/6;
    velocity_count = 0;
    if (iter==10){
      // Print velocity
      // queueMessage();
      iter = 0;
    }
    velocityCalc_th.signal_set(0);
  }
}
*/
int main() {

    //Run the motor synchronisation
    orState = motorHome();
    queueMessage(ROTOR_ORIGIN, uint64_t(orState));
    
    //set photointerrupter interrupts
    I1.rise(&updateMotor);
    I1.fall(&updateMotor);
    I2.rise(&updateMotor);
    I2.fall(&updateMotor);
    I3.rise(&updateMotor);
    I3.fall(&updateMotor);

    //set encoder interrupts
    channelA.rise(&updateEncoder);
    channelB.rise(&updateEncoder);
    channelA.fall(&updateEncoder);
    channelB.fall(&updateEncoder);

    //setup timer
    // Ticker t;
    // t.attach(&count, 1.0);

    //set up threads
    serialPrint_th.start(&serialPrint);
    decodeCommands_th.start(&decodeCommands);
    
    // Velocity calculation thread
    // velocityCalc_th.start(&velocityCalc);
    // Ticker t;
    // t.attach(&signalVelocity, 0.1); // 100ms
    
    updateMotor();
    while(true){
        //copy new key across
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();

        SHA256::computeHash(hash, sequence, 64);

        //successful nonce
        if((hash[0] == 0) && (hash[1] == 0)){
            queueMessage(NONCE_FOUND, *nonce);
        }
        *nonce += 1;
        hash_count++;
    }
}
