#pragma once

// AXP192 PMIC register definitions for M5StickC Plus (K016-P) battery fuel gauging.

#include <cstdint>

namespace od_soc::m5stickc_plus::axp192 {

inline constexpr uint8_t I2C_ADDR = 0x34;

// Contiguous status block [0x00..0x01] (2 bytes)
inline constexpr uint8_t REG_POWER_STATUS = 0x00;
inline constexpr uint8_t REG_CHARGE_STATUS = 0x01;
inline constexpr uint8_t POWER_VBUS_PRESENT = 1u << 5;
inline constexpr uint8_t POWER_BAT_CHARGING_DIR = 1u << 2;
inline constexpr uint8_t CHARGE_CHARGING = 1u << 6;
inline constexpr uint8_t CHARGE_BAT_PRESENT = 1u << 5;

// Contiguous VBUS + die temp block [0x5A..0x5F] (6 bytes)
inline constexpr uint8_t REG_VBUS_V_H = 0x5A;  // 0x5A..0x5B (12-bit, 1.7 mV/LSB)
inline constexpr uint8_t REG_VBUS_I_H = 0x5C;  // 0x5C..0x5D (12-bit, 0.375 mA/LSB)
inline constexpr uint8_t REG_TEMP_H   = 0x5E;  // 0x5E..0x5F (12-bit, 0.1 C/LSB - 144.7 C)

// Contiguous battery V + I block [0x78..0x7D] (6 bytes)
inline constexpr uint8_t REG_BAT_V_H    = 0x78;  // 0x78..0x79 (12-bit, 1.1 mV/LSB)
inline constexpr uint8_t REG_BAT_CHG_H  = 0x7A;  // 0x7A..0x7B (13-bit, 0.5 mA/LSB)
inline constexpr uint8_t REG_BAT_DIS_H  = 0x7C;  // 0x7C..0x7D (13-bit, 0.5 mA/LSB)

// ADC enable & sample rate control
inline constexpr uint8_t REG_ADC_ENABLE = 0x82;  // bit 7=Vbat, bit 6=Ibat, bit 3=Vbus, bit 2=Ibus
inline constexpr uint8_t REG_ADC_RATE   = 0x84;  // bits [7:6]: 00=25Hz, 01=50Hz, 10=100Hz, 11=200Hz
inline constexpr uint8_t ADC_RATE_MASK  = 0xC0;
inline constexpr uint8_t ADC_RATE_25HZ  = 0x00;
inline constexpr uint8_t ADC_RATE_50HZ  = 0x40;
inline constexpr uint8_t ADC_RATE_100HZ = 0x80;
inline constexpr uint8_t ADC_RATE_200HZ = 0xC0;

// Contiguous hardware coulomb counter block [0xB0..0xB7] (8 bytes) + control [0xB8]
inline constexpr uint8_t REG_COULOMB_CHG_0 = 0xB0;  // 32-bit big-endian charge accumulator
inline constexpr uint8_t REG_COULOMB_DIS_0 = 0xB4;  // 32-bit big-endian discharge accumulator
inline constexpr uint8_t REG_COULOMB_CTRL  = 0xB8;
inline constexpr uint8_t COULOMB_ENABLE    = 1u << 7;
inline constexpr uint8_t COULOMB_PAUSE     = 1u << 6;
inline constexpr uint8_t COULOMB_CLEAR     = 1u << 5;

}  // namespace od_soc::m5stickc_plus::axp192
