#define SYSTEM_CORE_CLOCK 48000000

#include "ch32v003fun.h"
#include <stdio.h>
#include <stdbool.h>

#define APB_CLOCK SYSTEM_CORE_CLOCK

volatile uint8_t i2c_registers[16] = {0x00};

void setup_i2c(uint8_t address) {
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

    I2C1->CTLR1 |= I2C_CTLR1_SWRST;
    I2C1->CTLR1 &= ~I2C_CTLR1_SWRST;

    // Set module clock frequency
    uint32_t prerate = 2000000; // I2C Logic clock rate - must be higher than Bus clock rate
    I2C1->CTLR2 |= (APB_CLOCK/prerate) & I2C_CTLR2_FREQ;

    // Enable interrupts
    I2C1->CTLR2 |= I2C_CTLR2_ITBUFEN;
    I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN; // Event interrupt
    I2C1->CTLR2 |= I2C_CTLR2_ITERREN; // Error interrupt

    NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
    NVIC_SetPriority(I2C1_EV_IRQn, 2 << 4);
    NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt

    // Set clock configuration
    uint32_t clockrate = 1000000; // I2C Bus clock rate - must be lower the Logic clock rate
    I2C1->CKCFGR = ((APB_CLOCK/(3*clockrate))&I2C_CKCFGR_CCR) | I2C_CKCFGR_FS; // Fast mode 33% duty cycle
    //I2C1->CKCFGR = ((APB_CLOCK/(25*clockrate))&I2C_CKCFGR_CCR) | I2C_CKCFGR_DUTY | I2C_CKCFGR_FS; // Fast mode 36% duty cycle
    //I2C1->CKCFGR = (APB_CLOCK/(2*clockrate))&I2C_CKCFGR_CCR; // Standard mode good to 100kHz

    // Set I2C address
    I2C1->OADDR1 = address << 1;

    // Enable I2C
    I2C1->CTLR1 |= I2C_CTLR1_PE;

    // Acknowledge the first address match event when it happens
    I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

struct _i2c_state {
    bool first_write;
    uint8_t offset;
    uint8_t position;
} i2c_state = {
    .first_write = false,
    .offset = 0,
    .position = 0,
};


void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint16_t STAR1, STAR2 __attribute__((unused));
    STAR1 = I2C1->STAR1;
    STAR2 = I2C1->STAR2;

    I2C1->CTLR1 |= I2C_CTLR1_ACK;

    if (STAR1 & I2C_STAR1_ADDR) { // Start event
        i2c_state.first_write = true; // Next write will be the offset
        i2c_state.position = i2c_state.offset; // Reset position
    }

    if (STAR1 & I2C_STAR1_RXNE) { // Write event
        if (i2c_state.first_write) { // First byte written, set the offset
            i2c_state.offset = I2C1->DATAR;
            i2c_state.position = i2c_state.offset;
            i2c_state.first_write = false;
        } else { // Normal register write
            if (i2c_state.position < sizeof(i2c_registers)) {
                i2c_registers[i2c_state.position] = I2C1->DATAR;
                i2c_state.position++;
            }
        }
    }

    if (STAR1 & I2C_STAR1_TXE) { // Read event
        if (i2c_state.position < sizeof(i2c_registers)) {
            I2C1->DATAR = i2c_registers[i2c_state.position];
            i2c_state.position++;
        } else {
            I2C1->DATAR = 0x00;
        }
    }
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
    uint16_t STAR1 = I2C1->STAR1;

    if (STAR1 & I2C_STAR1_BERR) { // Bus error
        I2C1->STAR1 &= ~(I2C_STAR1_BERR); // Clear error
    }

    if (STAR1 & I2C_STAR1_ARLO) { // Arbitration lost error
        I2C1->STAR1 &= ~(I2C_STAR1_ARLO); // Clear error
    }

    if (STAR1 & I2C_STAR1_AF) { // Acknowledge failure
        I2C1->STAR1 &= ~(I2C_STAR1_AF); // Clear error
    }
}

int main() {
    SystemInit48HSI();
    SetupDebugPrintf();

    setup_i2c(0x9);

    // Enable GPIOs
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;

    // GPIO D0 Push-Pull
    GPIOD->CFGLR &= ~(0xf<<(4*0));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

    while (1) {
        if (i2c_registers[0] & 1) { // Turn on LED if bit 1 of register 0 is set
            GPIOD-> BSHR |= 1 << 16;
        } else {
            GPIOD-> BSHR |= 1;
        }
    }
}
