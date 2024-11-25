#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

#define SWCLK_PIN 2         // GPIO pin for SWCLK
#define SWDIO_PIN 3         // GPIO pin for SWDIO
#define PWR_DBG 0x50000000  // Command to enable debug power
#define AIRCR 0xE000ED0C    // AIRCR address
#define DHCSR 0xE000EDF0    // DHCSR address
#define DEMCR 0xE000EDFC    // DEMCR address
#define HALT 0xA05F0003     // Halt Command
#define RESUME 0xA05F0001   // Resume Command
#define STEP 0xA05F0005     // Step Command
#define RESET 0x05FA0004    // Reset Command

bool parity_bit(uint32_t data) {
    bool parity = 0;
    while (data) {
        parity ^= (data & 1);
        data >>= 1;
    }
    return parity;
}

void pulse_swclk(int cycles) {
    for (int i = 0; i < cycles; i++) {
        gpio_put(SWCLK_PIN, 0);
        sleep_ms(1);
        // busy_wait_ms(1);
        gpio_put(SWCLK_PIN, 1);
        sleep_ms(1);
        // busy_wait_ms(1);
    }
}

void swdSetWriteMode() {
    gpio_set_dir(SWDIO_PIN, GPIO_OUT);
    pulse_swclk(1);
}

bool swdReadBit() {
    bool value = gpio_get(SWDIO_PIN);
    pulse_swclk(1);
    return value;
}

void swdWriteBit(int bit) {
    gpio_put(SWDIO_PIN, bit);
    pulse_swclk(1);
}

void swdWriteBits(uint32_t data, int bits) {
    for (int i = 0; i < bits; i++) {
        swdWriteBit((data >> i) & 0x1);
    }
}

void swd_line_reset() {
    gpio_put(SWDIO_PIN, 1);
    pulse_swclk(56);
    gpio_put(SWDIO_PIN, 0);
    pulse_swclk(4);
}

void jtag_to_swd_config() {
    // We do MSB first
    uint16_t sequence = 0xE79E;
    for (int i = 0; i < 16; i++) {
        swdWriteBit(sequence & 1);
        sequence >>= 1;
    }
}

void swd_send_request(uint8_t request) {
    for (int i = 7; i >= 0; i--) {
        swdWriteBit((request >> i) & 0x1);
    }
}
 
void turnaround() {
    gpio_set_dir(SWDIO_PIN, GPIO_IN);
    pulse_swclk(1);
}

uint32_t read_incoming() {
    uint32_t idcode = 0;
    for (int i = 0; i < 32; i++) {
        gpio_put(SWCLK_PIN, 0);
        sleep_ms(1);
        bool bit = gpio_get(SWDIO_PIN);
        gpio_put(SWCLK_PIN, 1);
        sleep_ms(1);
        idcode |= (bit << i);
    }
    gpio_set_dir(SWDIO_PIN, GPIO_OUT);
    return idcode;
}

void swd_read_ack() {
    bool bit1 = swdReadBit();
    bool bit2 = swdReadBit();
    bool bit3 = swdReadBit();
    if (bit1 == 1 && bit2 == 0 && bit3 == 0) {
        printf("\t\t[Info] ACK is valid: %d%d%d\n", bit1, bit2, bit3);
    } else {
        printf("\t\t[Error] ACK no good: %d%d%d\n", bit1, bit2, bit3);
    }
}

void swd_init() {
    // Initialize GPIO Pins and set them as output. Disable pull-up/pull-down resistors
    gpio_init(SWCLK_PIN);
    gpio_set_dir(SWCLK_PIN, GPIO_OUT);
    gpio_init(SWDIO_PIN);
    gpio_set_dir(SWDIO_PIN, GPIO_OUT);
    gpio_disable_pulls(SWDIO_PIN);
    gpio_disable_pulls(SWCLK_PIN);
}

void choose_your_core(){
    swdSetWriteMode();
    swdWriteBits(0x00,4);
     
    // Write into 0xC (TGTSEL)
    swd_send_request(0b10011001);
    turnaround();
    swd_read_ack();

    uint32_t data = 0x01002927;
    // uint32_t data = 0x11002927;
    bool parity = parity_bit(data);

    swdSetWriteMode();
    swdWriteBits(data, 32);
    swdWriteBit(parity);
}

void arm_wakeup_sequence() {
    swdSetWriteMode();

    // Reset to selection alert sequence
    gpio_put(SWDIO_PIN, GPIO_OUT);
    pulse_swclk(8);

    uint32_t sequence1 = 0x19BC0EA2;
    uint32_t sequence2 = 0xE3DDAFE9;
    uint32_t sequence3 = 0x86852D95;
    uint32_t sequence4 = 0x6209F392;

    swdWriteBits(sequence4, 32);
    swdWriteBits(sequence3, 32);
    swdWriteBits(sequence2, 32);
    swdWriteBits(sequence1, 32);

    // Idle bits
    swdWriteBits(0x00, 4);

    // Sending activation code
    swdWriteBits(0x1A, 8);
}

void writeTAR(uint32_t address){
    bool parity = parity_bit(address);
    swdWriteBits(0x00, 4);

    swd_send_request(0b11010001);
    turnaround();
    swd_read_ack();


    swdSetWriteMode();
    swdWriteBits(address, 32);
    swdWriteBit(parity);

}

void ReadRDBUFF(bool silenced){
    uint32_t response;
    if (!silenced){
        printf("\t[*] Reading RDBUFF\n");
    }
    
    swdSetWriteMode();
    swdWriteBits(0x00,4);
    
    swd_send_request(0b10111101);
    turnaround();
    swd_read_ack();

    response = read_incoming();
    if (!silenced){
        printf("\t[*] RDBUFF Response: 0x%08X\n", response);
    }
}

void dehalt(){
    bool parity = parity_bit(RESUME);

    swdSetWriteMode();
    printf("\t[RESUME] Writing DHCSR into TAR\n");
    writeTAR(DHCSR);

    printf("\t[RESUME] Writing 'resume' to DRW\n");

    // 8-3 Handshake to write into DRW
    swdWriteBits(0x00, 4);

    swd_send_request(0b11011101);
    turnaround();
    swd_read_ack();

    printf("\n[!] Resume!!\n\n");
    swdSetWriteMode();
    swdWriteBits(RESUME, 32);
    swdWriteBit(parity);
}

void shadowStep(){
    bool parity = parity_bit(STEP);

    swdSetWriteMode();
    writeTAR(DHCSR);

    swdWriteBits(0x00,4);
    swd_send_request(0b11011101);
    turnaround();
    swd_read_ack();

    swdSetWriteMode();

    swdWriteBits(STEP, 32);
    swdWriteBit(parity);
}

void halt(){
    bool parity = parity_bit(HALT);

    swdSetWriteMode();
    printf("\t[HALT] Writing DHCSR into TAR\n");
    writeTAR(DHCSR);

    printf("\t[HALT] Writing 'halt' to DRW\n");

    // 8-3 Handshake to write into DRW
    swdWriteBits(0x00, 4);

    swd_send_request(0b11011101);
    turnaround();
    swd_read_ack();

    printf("\n[!] HALT!!\n\n");
    swdSetWriteMode();
    swdWriteBits(HALT, 32);
    swdWriteBit(parity);
}

void readHaltRegister(bool silenced){
    uint32_t response;
    if (!silenced){
        printf("\t[+] Writing DHCSR into TAR\n");
    }
    
    writeTAR(DHCSR);
    if (!silenced){
        printf("\t[*] Reading DHCSR from DWR\n");
    }

    swdWriteBits(0x00, 4);

    swd_send_request(0b11111001);
    turnaround();
    swd_read_ack();

    response = read_incoming();
    if (!silenced){
        printf("\t[*] DWR Response: 0x%08X\n", response);
    }
    ReadRDBUFF(silenced);
}

void reset_and_halt(){
    bool parity = parity_bit(0x00000001);
    swdSetWriteMode();
    printf("\t[RESET] Writing DEMCR into TAR\n");
    writeTAR(DEMCR);
    printf("\t[RESET] Enabling Halt on reset\n");

    swdWriteBits(0x00, 4);

    swd_send_request(0b11011101);
    turnaround();
    swd_read_ack();

    printf("[RESET] Sending 0x00000001 to DEMCR\n");

    swdSetWriteMode();
    swdWriteBits(0x00000001, 32);
    swdWriteBit(parity);

    readHaltRegister(false);

    printf("\t[RESET] Writing AIRCR into TAR\n");
    writeTAR(AIRCR);
    printf("\t[RESET] Sending RESET command\n");
    parity = parity_bit(RESET);

    swdWriteBits(0x00, 4);
    swd_send_request(0b11011101);
    turnaround();
    swd_read_ack();

    swdSetWriteMode();
    swdWriteBits(RESET, 32);
    swdWriteBit(parity);
}

void cswGo(){
    uint32_t data = 0xA2000012;
    bool parity = parity_bit(0x0);

    // 8-3 Handshake to write to weite to APSEL
    swdSetWriteMode();
    swdWriteBits(0x00,4);

    swd_send_request(0b10001101);
    turnaround();
    swd_read_ack();

    swdSetWriteMode();
    swdWriteBits(0x0, 32);
    swdWriteBit(parity);

    // Recalculate parity for new data
    parity = parity_bit(data);

    printf("\t[*] Writing into CSW AP register\n");
    // 8-3 Hadnshake to write to CSW AP register
    swdWriteBits(0x00, 4);
    swd_send_request(0b11000101);

    turnaround();
    swd_read_ack();

    // Write into CSW
    swdSetWriteMode();
    swdWriteBits(data, 32);
    swdWriteBit(parity);
}

void powerDebug(){
    bool parity = parity_bit(PWR_DBG);
    swdSetWriteMode();
    swdWriteBits(PWR_DBG, 32);
    swdWriteBit(parity);
}

void checkClear(){
    uint32_t data = 0x1e;
    bool parity = parity_bit(data);

    swdSetWriteMode();
    swdWriteBits(data,32);
    swdWriteBit(parity);
}

void initialise_debugger(){
    // Declare variables
    uint32_t id_code;
    uint32_t ctrl_stat_before;
    uint32_t ctrl_stat_after;

    // Initialize GPIO 
    swd_init();
    
    arm_wakeup_sequence();

    swd_line_reset();
    
    // Transition from JTAG to SWD (send 16-bit selection sequence)
    jtag_to_swd_config();

    // Reset
    swd_line_reset();

    /*
    Choose which core to debug
    */
    printf("\t[*] Choosing core 0 to debug rn (ignore the ack)\n");  
    choose_your_core();
    
    /*
    Read IDCODE
    */

    printf("\t[*] Scanning IDCODE of the target!\n");
    // Send request to read register 0x00 [IDCODE] (8-3 Handshake)
    swd_send_request(0b10100101);
    turnaround();
    swd_read_ack();

    // Read the IDCODE register
    id_code = read_incoming();
    
    printf("\t[*] IDCODE of the target is: 0x%08X\n", id_code);

    /*
    READ CTRL/STAT (Before Clearing Errors)
    */

    // Send request to READ to register 0x04 [CTRL/STAT] (8-3 Handshake)
    swdSetWriteMode();
    swdWriteBits(0x00,4);
    swd_send_request(0b10110001);
    turnaround();
    swd_read_ack();

    // Read CTRL/STAT
    ctrl_stat_before = read_incoming();
    printf("\t[*] CTRL/STAT register value before clearing errors: 0x%08X\n", ctrl_stat_before);

    /*
    Clear Errors
    */
   
    // Send request to WRITE to register 0x00 [ABORT] (8-3 Handshake)
    swdSetWriteMode();
    swdWriteBits(0x00,4);

    swd_send_request(0b10000001);
    turnaround();
    swd_read_ack();

    // Clear Errors
    printf("\t[-] Clearing error flags...\n");  
    checkClear();

    /*
    READ CTRL/STAT (Before Power Debug)
    */

    // Send request to READ to register 0x04 [CTRL/STAT] (8-3 Handshake)
    swdSetWriteMode();
    swdWriteBits(0x00,4);
    swd_send_request(0b10110001);
    turnaround();
    swd_read_ack();

    // Read CTRL/STAT
    ctrl_stat_before = read_incoming();
    printf("\t[*] CTRL/STAT register value before power debug: 0x%08X\n", ctrl_stat_before);


    /*
    Power Debug
    */
    printf("\t[+] Enabling power debug\n");
    // Send request to WRITE to register 0x04 [CTRL/STAT] (8-3 Handshake)
    swdSetWriteMode();
    swdWriteBits(0x00,4);

    swd_send_request(0b10010101);
    turnaround();
    swd_read_ack();

    powerDebug();

    /*
    READ CTRL/STAT (After Power Debug)
    */

    // Send request to READ to register 0x04 [CTRL/STAT] (8-3 Handshake)
    swdSetWriteMode();
    swdWriteBits(0x00,4);
    
    swd_send_request(0b10110001);
    turnaround();
    swd_read_ack();

    ctrl_stat_after = read_incoming();  // Replace with actual read function
    printf("\t[*] CTRL/STAT register value after power debug: 0x%08X\n", ctrl_stat_after);    
}

int main() {
    stdio_init_all();
    sleep_ms(3000); // Gotta chill lmao i ain't the flash
    printf("\n[*] Debugger beta v.1\n");

    sleep_ms(800);  // Sleep for dramatic effect
    printf("\n[*] Initializing debugger right now bro\n");

    // Start the debugger
    initialise_debugger();

    printf("\n[*] Debugger is ready (i think)\n");

    // Plug inside MEM-AP
    printf("\t[+] Plugging into MEM-AP\n");
    sleep_ms(400);  // Sleep for dramatic effect
    cswGo();
    
    // Read the halt register
    sleep_ms(400);  // Sleep for dramatic effect
    readHaltRegister(false);    

    // Command 1: HALT
    sleep_ms(100); // Sleep for dramatic effect
    halt();
    readHaltRegister(false);

    // Command 2: Step (step 35 times)
    printf("\t[INFO] Stepping CPU\n");
    sleep_ms(100);
    for (int i = 0; i < 35; i++){
        printf("\t\t[Step] Step %d\n", i);
        sleep_ms(60); // Sleep for dramatic effect
        shadowStep();
        readHaltRegister(true);
    }

    // Command 3: Resume
    for (int i = 0; i < 5; i++) {
        sleep_ms(950); // Sleep for dramatic effect
        printf("[INFO] Resuming in %d\n", 5 - i);
    }

    printf("\n\t[+] Resuming CPU\n");
    dehalt();
    readHaltRegister(false); 

    // Command 4: RESET
    for (int i = 0; i < 5; i++) {
        sleep_ms(950); // Sleep for dramatic effect
        printf("[INFO] Reset in %d\n", 5 - i);
    }
    printf("\n\t[-] Attempting to RESET");
    reset_and_halt();
    readHaltRegister(false);

    // Command 3: Resume again
    for (int i = 0; i < 5; i++) {
        sleep_ms(950); // Sleep for dramatic effect
        printf("[INFO] Resuming in %d\n", 5 - i);
    }

    printf("\n\t[+] Resuming CPU\n");
    dehalt();
    readHaltRegister(false); 

    return 0;
}
