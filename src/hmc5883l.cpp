/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Digital Compass Arduino Library.
Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <hmc5883l.h>
#include <driver/i2c.h>
#include <cmath>

HMC5883L::HMC5883L(){}

void HMC5883L::setRange(hmc5883l_range_t range)
{
    writeRegister(HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_B, range << 5);
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = readRegister(HMC5883L_ADDRESS,HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_MODE, value);
}

void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A, value);
}

void HMC5883L::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister(HMC5883L_ADDRESS,HMC5883L_REG_CONFIG_A, value);
}

void HMC5883L::writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t read_data)
{
    uint8_t buff[2];
    buff[0] = reg_addr;
    buff[1] = read_data;
    
    i2c_master_write_to_device(I2C_NUM_0,
                                dev_addr,
                                buff,
                                2,
                                pdMS_TO_TICKS(500)
                                );
}

uint8_t HMC5883L::readRegister(uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t rx_data[1];
    uint8_t data = 0;

    i2c_master_write_read_device(I2C_NUM_0,
                                 dev_addr,
                                 &reg_addr,
                                 1,
                                 rx_data,
                                 1,
                                 pdMS_TO_TICKS(10));

    data = rx_data[0];

    return data;
}

void HMC5883L::setOffset(int16_t x, int16_t y)
{
    xOffset = x;
    yOffset = y;
}

void HMC5883L::setDeclination(float d)
{
    declination = d;
} 


float HMC5883L::getDegrees()
{
    uint8_t lenData = 6;
    uint8_t rx_data[lenData];
    uint8_t device_addr = 0x1E;
    uint8_t data_addr = 0x03;   //0x03 Data Output X MSB Register  0x04 Data Output X LSB Register
                                //0x07 Data Output Y MSB Register  0x08 Data Output Y LSB Register
    int16_t xData = 0;
    int16_t yData = 0;

    i2c_master_write_read_device(I2C_NUM_0,
                                 device_addr,
                                 &data_addr,
                                 1,
                                 rx_data,
                                 lenData,
                                 pdMS_TO_TICKS(10));

    xData = (rx_data[0] << 8 | rx_data[1]) - xOffset + declination;
    yData = (rx_data[4] << 8 | rx_data[5]) - yOffset + declination;
    
    float data = atan2(yData,xData)*180/3.141592;

    return data;
}