#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <driver/i2c.h>

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_DEFAULT_SCL_IO           (gpio_num_t)26               /*!<gpio number for i2c slave clock  */
#define I2C_DEFAULT_SDA_IO           (gpio_num_t)25               /*!<gpio number for i2c slave data */
#define I2C_DEFAULT_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_DEFAULT_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_DEFAULT_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

//#define I2C_DEFAULT_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
//#define I2C_DEFAULT_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
//#define I2C_DEFAULT_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_SLAVE_RX_BUF_DIMENSION  250                /*!< I2C master do not need buffer */
#define I2C_SLAVE_TX_BUF_DIMENSION  250                /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

//#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
//#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
//#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */


class I2Cdevice
{
  protected :
    i2c_port_t i2c_port;
    i2c_config_t conf;

  public:
    I2Cdevice(i2c_port_t i2cNum, gpio_num_t signalGPIO, gpio_num_t clockGPIO);
    ~I2Cdevice();
    esp_err_t reset();
    void setTimeout(int timeoutTick);
    int getTimeout();
    void setEndian(bool isBigEndian);
    virtual esp_err_t init() {return(ESP_OK);};
  private:
};

class I2Cmaster : public I2Cdevice
{
  private:

  public:
    I2Cmaster();
    I2Cmaster(i2c_port_t i2cNum);
    I2Cmaster(i2c_port_t i2cNum, gpio_num_t signalGPIO, gpio_num_t clockGPIO);
    ~I2Cmaster();
    esp_err_t write(uint8_t address, uint8_t* data_wr, size_t size);
    esp_err_t read(uint8_t address, uint8_t* data_rd, size_t size);
    esp_err_t init();
  private:
};

class I2Cslave : public I2Cdevice
{
  private:
    size_t rxBufferDim;
    size_t txBufferDim;

  public:
    I2Cslave();
    I2Cslave(i2c_port_t i2cNum);
    I2Cslave(i2c_port_t i2cNum, gpio_num_t signalGPIO, gpio_num_t clockGPIO);
    ~I2Cslave();
    void setAddress(uint8_t address);
    void setBufferDim(size_t rxBufferSize, size_t txBufferSize);
    esp_err_t write(uint8_t* data_wr, size_t size);
    size_t read(uint8_t* data_rd, size_t maxSize);
    esp_err_t init();
  private:
};

#endif
