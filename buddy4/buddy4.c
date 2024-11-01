#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

#define SWCLK_PIN 2  // GPIO pin for SWCLK
#define SWDIO_PIN 3  // GPIO pin for SWDIO
#define PWR_DBG 0x50000000  // Command to enable debug power
#define DHCSR 0xE000EDF0  // DHCSR address
#define DBGKEY 0xA05F0000 // Debug key

void pulse_swclk(int cycles) {
    for (int i = 0; i < cycles; i++) {
        gpio_put(SWCLK_PIN, 0);
        sleep_ms(1);
        gpio_put(SWCLK_PIN, 1);
        sleep_ms(1);
    }
}

void swdSetWriteMode() {
    gpio_set_dir(SWDIO_PIN, GPIO_OUT);
    pulse_swclk(1);
}

void swdSetReadMode() {
    gpio_set_dir(SWDIO_PIN, GPIO_IN);
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

uint32_t swd_read_idcode() {
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
        printf("\t\t[Warning] ACK is not chill: %d%d%d\n", bit1, bit2, bit3);
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

void swd_send_wakeup_sequence() {
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

void powerDebug(){
    swdSetWriteMode();
    swdWriteBits(PWR_DBG, 32);
    swdWriteBits(0x0, 1);
    swdWriteBits(0x00, 4);
}

void checkClear(){
    swdSetWriteMode();
    swdWriteBits(0x1e,32);
    swdWriteBits(0x0,1);
    swdWriteBits(0x00, 4);
}

void resumeCpu(){
    /*
    APSEL
    */
    printf("\t[*] Plugging into MEM-AP\n");
    // 8-3 Handshake [Write APSEL]
    swdSetWriteMode();
    swdWriteBits(0x00,4);

    swd_send_request(0b10001101);
    turnaround();
    swd_read_ack();

    // Suggest to traverse into MEM-AP at AP0 (0x0)
    swdWriteBits(0x0,32);
    swdWriteBits(0x0,1);
    swdWriteBits(0x00, 4);

    /*
    TAR
    */
    printf("\t[*] Writing DHCSR to TAR\n");
    // 8 -3 Handshake [Write TAR]
    swdSetWriteMode();
    swdWriteBits(0x00,4);

    swd_send_request(0b11010001);
    turnaround();
    swd_read_ack();

    // Whack the address inside TAR
    swdSetWriteMode();
    swdWriteBits(DHCSR, 32);
    swdWriteBits(0x0,1);
    swdWriteBits(0x00, 4);

    /*
    DWR
    */
    printf("\t[*] Sending resume command to DWR\n");
    // 8-3 Handshake [Write DWR]
    swdSetWriteMode();
    swdWriteBits(0x00,4);
    swd_send_request(0b11011101);
    turnaround();
    swd_read_ack();

    // Send Resume command
    swdSetWriteMode();
    swdWriteBits(0x01040001, 32);
    swdWriteBits(0x0,1);
    swdWriteBits(0x00, 4);
}

void initialise_debugger(){
    // Declare variables
    uint32_t id_code;
    uint32_t ctrl_stat_before;
    uint32_t ctrl_stat_after;

    // Initialize GPIO 
    swd_init();
    
    swd_send_wakeup_sequence();

    swd_line_reset();
    
    // Transition from JTAG to SWD (send 16-bit selection sequence)
    jtag_to_swd_config();

    // Reset
    swd_line_reset();
    /*
    Read IDCODE
    */


    // Send request to read register 0x00 [IDCODE] (8-3 Handshake)
    swd_send_request(0b10100101);
    turnaround();
    swd_read_ack();

    // Read the IDCODE register
    id_code = swd_read_idcode();
    
    printf("\t[*] IDCODE of the target is: 0x%08X\n", id_code);

    /*
    READ ME: FOR THE SAKE OF WEEK 10 DELIVERABLES JUST COMMENT OUT EVERYTHING BELOW HERE!!!!!
    READ ME: FOR THE SAKE OF WEEK 10 DELIVERABLES JUST COMMENT OUT EVERYTHING BELOW HERE!!!!!
    READ ME: FOR THE SAKE OF WEEK 10 DELIVERABLES JUST COMMENT OUT EVERYTHING BELOW HERE!!!!!
    */

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
    ctrl_stat_before = swd_read_idcode();
    printf("\t[*] CTRL/STAT register value before power debug: 0x%08X\n", ctrl_stat_before);

    /*
    Power Debug
    */

    // Send request to WRITE to register 0x04 [CTRL/STAT] (8-3 Handshake)
    swdSetWriteMode();
    swdWriteBits(0x00,4);
    swd_send_request(0b10010101);
    turnaround();
    swd_read_ack();

    // Send Power Debug command
    printf("\t[+] Enabling power debug\n");
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

    ctrl_stat_after = swd_read_idcode();  // Replace with actual read function
    printf("\t[*] CTRL/STAT register value after power debug: 0x%08X\n", ctrl_stat_after);
}

int main() {
    stdio_init_all();
    sleep_ms(6500); // Gotta chill lmao i ain't the flash
    printf("[*] Debugger pre-alpha v.1\n");

    // Initialize the debugger
    sleep_ms(800);  // Sleep for dramatic effect
    printf("\n[*] Initializing debugger right now bro\n");

    initialise_debugger();

    // At this stage the CPU should be auto-halted
    printf("\n[*] Debugger is ready! (i think)\n");

    sleep_ms(2000); //Simulate the fact that someone is sending the command

    // Command 1: Resume
    resumeCpu();

    return 0;
}
