#include "stm32f1xx_hal.h"  // or adjust for your STM32 series

class FlashEEPROM {
public:
    static constexpr uint32_t FLASH_PAGE_ADDR = 0x0801FC00; // last 1KB page of 128KB flash
    static constexpr uint32_t PAGE_SIZE = 1024;

    FlashEEPROM() {
        load();
    }

    uint8_t read(uint16_t index) const {
        if (index >= PAGE_SIZE) return 0xFF;
        return buffer[index];
    }

    void write(uint16_t index, uint8_t value) {
        if (index >= PAGE_SIZE) return;
        if (buffer[index] != value) {
            buffer[index] = value;
            dirty = true;
        }
    }

    bool commit() {
        if (!dirty) return true;

        HAL_FLASH_Unlock();

        FLASH_EraseInitTypeDef eraseInit;
        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.PageAddress = FLASH_PAGE_ADDR;
        eraseInit.NbPages = 1;

        uint32_t pageError = 0;
        if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        // Write buffer to flash as halfwords (2 bytes)
        for (uint32_t i = 0; i < PAGE_SIZE; i += 2) {
            uint16_t halfword = buffer[i];
            if (i + 1 < PAGE_SIZE)
                halfword |= (buffer[i + 1] << 8);
            else
                halfword |= 0xFF00;  // pad with 0xFF if odd size

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_PAGE_ADDR + i, halfword) != HAL_OK) {
                HAL_FLASH_Lock();
                return false;
            }
        }

        HAL_FLASH_Lock();
        dirty = false;
        return true;
    }

    void resetBuffer() {
        for (uint16_t i = 0; i < PAGE_SIZE; ++i)
            buffer[i] = 0xFF;
        dirty = true;
    }

private:
    uint8_t buffer[PAGE_SIZE];
    bool dirty = false;

    void load() {
        const uint8_t* flash = reinterpret_cast<const uint8_t*>(FLASH_PAGE_ADDR);
        for (uint16_t i = 0; i < PAGE_SIZE; ++i)
            buffer[i] = flash[i];
        dirty = false;
    }
};



/*    // Example usage
// you need to modify linker script to restrict flash size 
// and FLASH_PAGE_ADDR should be set the correct address to the last 1KB page

FlashEEPROM eeprom;

void setup() {
    HAL_Init();  // if not already called
    // Read a byte
    uint8_t val = eeprom.read(0);

    // Update a byte
    eeprom.write(0, val + 1);

    // Save changes to flash
    eeprom.commit();
}

void loop() {
    // nothing here
}

*/