#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include <stdio.h>
#include <stdbool.h>

#define APB_CLOCK SYSTEM_CORE_CLOCK

uint8_t i2c_registers[16] = {0x00};

void setup_i2c(uint8_t address) {
    uint16_t tempreg;

    // Enable GPIOC and I2C
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PC1 is SDA, 10MHz Output, alt func, open-drain
    GPIOC->CFGLR &= ~(0xf<<(4*1));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*1);

    // PC2 is SCL, 10MHz Output, alt func, open-drain
    GPIOC->CFGLR &= ~(0xf<<(4*2));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*2);

    // Reset I2C1 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

    // Set module clock frequency
    uint32_t prerate = 2000000; // I2C Logic clock rate - must be higher than Bus clock rate
    I2C1->CTLR2 |= (APB_CLOCK/prerate) & I2C_CTLR2_FREQ;

    // Enable interrupts
    I2C1->CTLR2 |= I2C_CTLR2_ITBUFEN;
    I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN; // Event interrupt
    //I2C1->CTLR2 |= I2C_CTLR2_ITERREN; // Error interrupt

    NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
    //NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt

    // Set clock configuration
    uint32_t clockrate = 1000000; // I2C Bus clock rate - must be lower the Logic clock rate
    I2C1->CKCFGR = ((APB_CLOCK/(3*clockrate))&I2C_CKCFGR_CCR) | I2C_CKCFGR_FS; // Fast mode 33% duty cycle
    //I2C1->CKCFGR = ((APB_CLOCK/(25*clockrate))&I2C_CKCFGR_CCR) | I2C_CKCFGR_DUTY | I2C_CKCFGR_FS; // Fast mode 36% duty cycle
    //I2C1->CKCFGR = (APB_CLOCK/(2*clockrate))&I2C_CKCFGR_CCR; // Standard mode good to 100kHz

    // Set I2C address
    I2C1->OADDR1 = address << 1;

    // Enable I2C
    I2C1->CTLR1 |= I2C_CTLR1_PE;

    printf("I2C1->CTLR1  0x%04x\n", I2C1->CTLR1);
    printf("I2C1->CTLR2  0x%04x\n", I2C1->CTLR2);
    printf("I2C1->OADDR1 0x%04x\n", I2C1->OADDR1);
    printf("I2C1->OADDR2 0x%04x\n", I2C1->OADDR2);
    printf("I2C1->DATAR  0x%04x\n", I2C1->DATAR);
    printf("I2C1->STAR1  0x%04x\n", I2C1->STAR1);
    printf("I2C1->STAR2  0x%04x\n", I2C1->STAR2);
    printf("I2C1->CKCFGR 0x%04x\n", I2C1->CKCFGR);
}

uint8_t transaction_length = 0;
uint8_t transaction_address = 0;
bool transaction_direction = false;
bool transaction_initialized = false;

bool dbg_start_req = false;
bool dbg_txe_req = false;
bool dbg_rxne_req = false;
bool dbg_init = false;
bool dbg_af = false;
bool dbg_stop = false;
bool dbg_unknown = false;

uint16_t dbg_unknown1, dbg_unknown2;

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint16_t STAR1, STAR2, DATAR __attribute__((unused));
    STAR1 = I2C1->STAR1;
    STAR2 = I2C1->STAR2;

    dbg_unknown1 = STAR1;
    dbg_unknown2 = STAR2;

    if (STAR1 & I2C_STAR1_ADDR) {
        // Start of transaction
        transaction_length = 0;
        transaction_address = 0;
        transaction_direction = false;
        transaction_initialized = false;
        I2C1->CTLR1 |= I2C_CTLR1_ACK; // Ack the start bit
        dbg_start_req = true;
    } else if (STAR1 & I2C_STAR1_TXE) {
        // Read requested
        if (transaction_address < sizeof(i2c_registers)) {
            I2C1->DATAR = i2c_registers[transaction_address];
            transaction_address++;
        } else {
            I2C1->DATAR = 0x00;
        }
        I2C1->CTLR1 |= I2C_CTLR1_ACK; // Ack the read
        dbg_txe_req = true;
    } else if (STAR1 & I2C_STAR1_RXNE) {
        // Write requested
        if (transaction_initialized) {
            if (transaction_address < sizeof(i2c_registers)) {
                i2c_registers[transaction_address] = I2C1->DATAR;
                transaction_address++;
            }
            dbg_rxne_req = true;
        } else {
            transaction_address = I2C1->DATAR;
            transaction_initialized = true;
            dbg_init = true;
        }
        I2C1->CTLR1 |= I2C_CTLR1_ACK; // Ack the write
    } else if (STAR1 & I2C_STAR1_AF) {
        // Acknowledge failure (end of transaction)
        I2C1->STAR1 &= ~(I2C_STAR1_AF); // Clear AF bit
        I2C1->CTLR1 |= I2C_CTLR1_STOP; // Send stop bit
        dbg_af = true;
    } else if (STAR1 & I2C_STAR1_STOPF) {
        I2C1->CTLR1 |= I2C_CTLR1_ACK; // Ack the stop
        dbg_stop = true;
    } else {
        I2C1->CTLR1 |= I2C_CTLR1_STOP; // Send stop bit
        dbg_unknown = true;
    }
}

bool error_interrupt_called = false;

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
    error_interrupt_called = true;
    printf("ERROR INTERRUPT\n");
}

int main() {
    SystemInit48HSI();
    SetupDebugPrintf();

    Delay_Ms(100);

    setup_i2c(0x9);

    // Enable GPIOs
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

    // GPIO D0 Push-Pull
    GPIOD->CFGLR &= ~(0xf<<(4*0));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

    while (1) {
        if (dbg_start_req) {
            printf("Start\n");
        }

        if (dbg_txe_req) {
            printf("Read\n");
        }

        if (dbg_rxne_req) {
            printf("Register write, new state: %02x: ", transaction_address);
            for (uint8_t i = 0; i < sizeof(i2c_registers); i++) {
                printf("%02x ", i2c_registers[i]);
            }
            printf("\n");
        }

        if (dbg_init) {
            printf("Init\n");
        }

        if (dbg_af) {
            printf("End\n");
        }

        if (dbg_stop) {
            printf("Stop\n");
        }

        if (dbg_unknown) {
            printf("Unknown %04X %04X\n", dbg_unknown1, dbg_unknown2);
        }

        dbg_start_req = false;
        dbg_txe_req = false;
        dbg_rxne_req = false;
        dbg_init = false;
        dbg_af = false;
        dbg_stop = false;
        dbg_unknown = false;

        if (error_interrupt_called) {
            error_interrupt_called = false;
            printf("Error interrupt called\n");
        }

        if (i2c_registers[0] & 1) {
            GPIOD-> BSHR |= 1 << 16;
        } else {
            GPIOD-> BSHR |= 1;
        }
    }
}
