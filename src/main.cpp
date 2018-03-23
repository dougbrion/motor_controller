#include "mbed.h"
#include "rtos.h"
#include "hash/SHA256.h"
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <cmath>

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
int8_t lead = -2;  //2 for forwards, -2 for backwards

static int8_t orState = 0;


//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

const int PWM_PERIOD = 2000; // us
// Duty cycle must be between 0 and 50% of 2000

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
    ROTATION_ERROR,
    HASH_RATE
};

enum msg_types{
    NONE,
    ORDERS,
    YAY,
    NAY,
    WRONG_ORDER,
    WHATS_NEXT,
    TORQUE,
    VELOCITY,
    POSITION,
    ROTATIONS,
    TARGET_POSITION,
    TARGET_VELOCITY,
    TARGET_ROTATIONS,
    K_D_ENUM,
    K_P_ENUM,
    K_I_ENUM
};

// ---------- SERIAL VARIABLES ----------
RawSerial pc(SERIAL_TX, SERIAL_RX);

// Message struct
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
volatile float velocity = 0;
volatile float target_velocity = 50;

volatile int32_t motor_position = 0;
volatile float target_rotations = 15;
volatile int32_t target_position = int32_t(6 * target_rotations);

volatile int32_t rotations = 0;

int32_t encoder_state = 0;

uint32_t cmd_torque = 0.5 * PWM_PERIOD; // Max power (duty cycle) allowed is 50%


volatile uint32_t K_PV = 85;
volatile uint32_t K_PP = 58;
volatile uint32_t K_I = 2;
volatile uint32_t K_D = 36;


volatile float i_error = 0;


volatile float velocity_error = 0;

volatile float position_error = 0;


// ---------- THREADING VARIABLES ----------
Thread serialPrint_th(osPriorityNormal, 1024);
Thread decodeCommands_th(osPriorityNormal, 1024);
Thread velocityCalc_th(osPriorityHigh, 1024);

Mutex newKey_mutex;

// ------------- FUNCTIONS -------------
//Set a given drive state
void motorOut(int8_t driveState, uint32_t pulseWidth){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(pulseWidth);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(pulseWidth);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(pulseWidth);
    if (driveOut & 0x20) L3H = 0;
}

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 0.5 * PWM_PERIOD); // Max duty cycle allowed is 50%
    wait(1.0);
    motor_position = 0;
    velocity = 0;

    //Get the rotor state
    return readRotorState();
}

void updateMotor(){

    static int8_t last_state;
    int8_t intState = readRotorState() - orState;

    motorOut((intState + lead + 6) % 6, cmd_torque); //+6 to make sure the remainder is positive
    
    if(intState - last_state == 5)
    {
      motor_position--;
    }
    else if(intState - last_state == -5)
    {
      motor_position++;
    }
    else
    {
      motor_position += (intState - last_state);
    }
    rotations = int32_t(motor_position / 6);
    last_state = intState;
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

// Checks message queue and prints
void serialPrint(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;

        switch(pMessage->code){
            case MSG:
                switch(static_cast<msg_types>(pMessage->data)){
                    case ORDERS:
                        pc.printf("Set rotations, speed or key. Which would you like commander?\n\r");
                    case YAY:
                        pc.printf("Yay!\n\r");
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
                        pc.printf("Velocity: %lf\n\r", velocity);
                        break;
                    case POSITION:
                        pc.printf("Position: %d\n\r", motor_position);
                        break;
                    case TARGET_ROTATIONS:
                        pc.printf("Target Rotations: %lf\n\r", target_rotations);
                        break;
                    case ROTATIONS:
                        pc.printf("Rotations: %d\n\r", rotations);
                        break;
                    case TARGET_POSITION:
                        pc.printf("Target Position: %d\n\r", target_position);
                        break;
                    case TARGET_VELOCITY:
                        pc.printf("Target Velocity: %lf\n\r", target_velocity);
                        break;
                    case K_P_ENUM:
                        pc.printf("KP: %u\n\r", K_PP);
                        break;
                    case K_I_ENUM:
                        pc.printf("KI: %u\n\r", K_I);
                        break;
                    case K_D_ENUM:
                        pc.printf("KD: %u\n\r", K_D);
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
                pc.printf("Rotating %d revolutions\n\r", pMessage->data);
                break;
            case ROTOR_ORIGIN:
                pc.printf("Rotor origin at 0x%016x\n\r", pMessage->data);
                break;
            case NONCE_FOUND:
                pc.printf("We've found ourselves a nonce! 0x%016x\n\r", pMessage->data);
                break;
            case ROTATION_ERROR:
                pc.printf("Error, your input for setting revolutions was wrong!", pMessage->data);
                break;
            case HASH_RATE:
                pc.printf("Hash Rate is %u\n\r", pMessage->data);
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

// Function for converting 2 unsigned ints to a float for the regex's in decodeCommands
float intsTofloat(int pre_point, int post_point) {
    if (post_point > 0){
        double power = ceil(log10(double(post_point))); 
        float dec = float(post_point) / pow(10.0, power);
        return (pre_point + dec);
    }
    return float(pre_point);
}

// decodes commands in the char buffer
void decodeCommands(){
    pc.attach(&serialISR);
    char command[32];
    int index = 0;
    while(true){
        osEvent newEvent = charBuffer.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        pc.printf("%c", newChar);
        if (index > 31) {
            index = 0;
            std::memset(command, 0, sizeof(command));
        }
        if (newChar != '\r') {
            command[index] = newChar;
            index++;
        }
        else {
            command[index] = '\0';
            index = 0;

            float rev_tmp = 0;
            int before_point = 0;
            int after_point = 0;
            float vel_tmp = 0;
            char neg_check = '\0';
            uint32_t kpp_set = 0;
            uint32_t ki_set = 0;
            uint32_t kd_set = 0;

            switch(command[0]) {
                case 'R':
                case 'r':
                    sscanf(command, "%*[rR]%c%3u.%2u", &neg_check, &before_point, &after_point);
                    if (neg_check != '-') {
                        sscanf(command, "%*[rR]%3u.%2u", &before_point, &after_point);
                    }
                    if (after_point < 0 || before_point < 0) {
                        queueMessage(ROTATION_ERROR, uint32_t(before_point));
                    }
                    rev_tmp = intsTofloat(before_point, after_point);
                    if (neg_check == '-') {
                        rev_tmp = -rev_tmp;
                    }
                    if (rev_tmp == 0){
                        if (lead == -2){
                            target_rotations -= 1;
                        } else {
                            target_rotations += 1;
                        }
                    } else {
                        target_rotations = rotations + rev_tmp;
                    }
                    position_error = 0;
                    velocity_error = 0;
                    i_error = 0;
                    target_position = int32_t(6 * target_rotations);
                    queueMessage(NEW_ROTATION, uint64_t(rev_tmp));
                    updateMotor();
                    break;

                case 'V':
                case 'v':
                    sscanf(command, "%*[vV]%3u.%u", &before_point, &after_point);
                    pc.printf("Before: %d\n\r", before_point);
                    pc.printf("After: %d\n\r", after_point);
                    vel_tmp = intsTofloat(before_point, after_point);
                    if (vel_tmp == 0){
                        target_velocity = 500;
                    } else {
                        target_velocity = vel_tmp;
                    }
                    velocity_error = 0;
                    position_error = 0;
                    i_error = 0;
                    queueMessage(NEW_SPEED, uint64_t(target_velocity));
                    break;

                case 'K':
                case 'k':
                    newKey_mutex.lock();
                    sscanf(command, "%*[kK]%10llx", &newKey);
                    // pc.printf("Key: %16x", newKey);
                    queueMessage(NEW_KEY, uint64_t(newKey));
                    *key = newKey;
                    newKey_mutex.unlock();
                    //valid_key(tmp);
                    break;
                
                case 'Q':
                case 'q':
                    // For setting Kp for rotations
                    sscanf(command, "%*[qQ]%u", &kpp_set);
                    pc.printf("KP: %u\n\r", kpp_set);
                    K_PP = kpp_set;
                    break;

                case 'W':
                case 'w':
                    // For setting Ki for rotations
                    sscanf(command, "%*[wW]%u", &ki_set);
                    pc.printf("KI: %u\n\r", ki_set);
                    K_I = ki_set;
                    break;

                case 'E':
                case 'e':
                    // For setting Kd for rotations
                    sscanf(command, "%*[eE]%u", &kd_set);
                    pc.printf("KD: %u\n\r", kd_set);
                    K_D = kd_set;
                    break;

                default:
                    queueMessage(MSG, uint64_t(WRONG_ORDER));
                    break;
            }
            queueMessage(MSG, uint64_t(WHATS_NEXT));
            std::memset(command, 0, sizeof(command));
        }
    }
}


void signalVelocity(){
  velocityCalc_th.signal_set(0x1); //velSig = 1;
}

Timer t;
void velocityCalc(){
  static uint8_t iter = 0;
  float local_vc = 0;
  float time_passed = 1;

  float velocity_controller = 0;
  float position_controller = 0;

  float controller_used = 0;

  static int32_t oldMotorPosition = 0;

    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&signalVelocity, 100000); // 100ms

  while (true){
    // Wait until signal from signalVelocity
    velocityCalc_th.signal_wait(0x1);
    iter++;
    //Interrupts should be disabled in a critical section while the position is copied into a
    //local variable.
    t.stop();
    __disable_irq();
    local_vc = float(motor_position - oldMotorPosition);
    __enable_irq();

    time_passed = t.read();
    velocity = (local_vc / (time_passed * 6));
    oldMotorPosition = motor_position;

    t.reset();
    t.start();

    // Implementing position control only
    position_error = target_position - motor_position;
    i_error += (position_error * time_passed);
    position_controller = (K_PP * position_error) + (K_I * i_error) + (K_D * position_error) / time_passed;

    // Implementing velocity control only
    velocity_error = target_velocity - abs(velocity);
    velocity_controller = std::copysign(K_PV * velocity_error, position_error); 

    if (velocity < 0) {
      controller_used = max(velocity_controller, position_controller);
    } else {
      controller_used = min(velocity_controller, position_controller);
    }
    
    if (controller_used > 1000){
        cmd_torque = 1000;
    } else {
        cmd_torque = controller_used;
    }

    if (velocity_controller < 0) {
        lead = -2;
    }
    else{
        lead = 2;
    }

    if (iter == 15){
        queueMessage(MSG, uint64_t(VELOCITY));
        queueMessage(MSG, uint64_t(TARGET_VELOCITY)); 
        queueMessage(MSG, uint64_t(ROTATIONS));
        queueMessage(MSG, uint64_t(TARGET_ROTATIONS));
        // queueMessage(MSG, uint64_t(K_P_ENUM));
        // queueMessage(MSG, uint64_t(K_I_ENUM));
        // queueMessage(MSG, uint64_t(K_D_ENUM));
        iter = 0;
    }
  }
}

int main() {
    Timer hash_timer;
    //Run the motor synchronisation
    orState = motorHome();
    queueMessage(ROTOR_ORIGIN, uint64_t(orState));

    // Initialisation PWM period
    L1L.period_us(0.5 * PWM_PERIOD);
    L2L.period_us(0.5 * PWM_PERIOD);
    L3L.period_us(0.5 * PWM_PERIOD);
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

    //set up threads
    serialPrint_th.start(&serialPrint);
    decodeCommands_th.start(&decodeCommands);

    // Velocity calculation thread
    velocityCalc_th.start(&velocityCalc);

    hash_timer.start();

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
        if (hash_timer.read() > 1.0f) {
            queueMessage(HASH_RATE, uint64_t(hash_count));
            hash_count = 0;
            hash_timer.reset();
        }
    }
}
