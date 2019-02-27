#include "i2c.h"
#include "esp_log.h"

I2Cdevice::I2Cdevice(i2c_port_t i2cNum, gpio_num_t signalGPIO, gpio_num_t clockGPIO)
{
  if (i2cNum != I2C_NUM_0 && i2cNum != I2C_NUM_1) i2cNum = I2C_DEFAULT_NUM;
  i2c_port = i2cNum;
  conf.sda_io_num = signalGPIO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = clockGPIO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

  ESP_LOGD("i2c", "Set i2c %d %d %d",i2cNum, signalGPIO, clockGPIO);
// call the specific settings for derivate classes
//  setup(i2cNum, signalGPIO, clockGPIO);
}
I2Cdevice::~I2Cdevice()
{
}

esp_err_t I2Cdevice::reset()
{
  esp_err_t err;
  err = i2c_reset_tx_fifo(i2c_port);
  err |= i2c_reset_rx_fifo(i2c_port);
  return err;
}

void I2Cdevice::setTimeout(int timeoutTick)
{
  // units APB 80Mhz clock cycle
  esp_err_t err;
  err = i2c_set_timeout(i2c_port, timeoutTick);
  return;
}
int I2Cdevice::getTimeout()
{
  // units APB 80Mhz clock cycle
  esp_err_t err;
  int timeoutTick;
  err = i2c_get_timeout(i2c_port, &timeoutTick);
  return timeoutTick;
}

void I2Cdevice::setEndian(bool isBigEndian)
{
  i2c_trans_mode_t mode;
  esp_err_t err;
  mode = isBigEndian ? I2C_DATA_MODE_MSB_FIRST : I2C_DATA_MODE_LSB_FIRST;
  err = i2c_set_data_mode(i2c_port, mode, mode);
  return;
}

// ------------------

I2Cmaster::I2Cmaster():I2Cdevice(I2C_DEFAULT_NUM, I2C_DEFAULT_SCL_IO, I2C_DEFAULT_SDA_IO) {};
I2Cmaster::I2Cmaster(i2c_port_t i2cNum):I2Cdevice(i2cNum, I2C_DEFAULT_SCL_IO, I2C_DEFAULT_SDA_IO) {};
I2Cmaster::I2Cmaster(i2c_port_t i2cNum, gpio_num_t signalGPIO, gpio_num_t clockGPIO):I2Cdevice(i2cNum, signalGPIO, clockGPIO) {};
I2Cmaster::~I2Cmaster() {};

esp_err_t I2Cmaster::init()
{
  //reset();
  ESP_LOGD("i2c", "Init the driver");
  conf.mode = I2C_MODE_MASTER;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_port, &conf);
  return(i2c_driver_install(i2c_port, conf.mode,I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

esp_err_t I2Cmaster::write(uint8_t address, uint8_t* data_wr, size_t size)
{
	esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err = i2c_master_start(cmd);
    err = i2c_master_write_byte(cmd, ( address << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    err = i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    err = i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGD("i2c", "Exit to write with %d", err);
    return err;
}

esp_err_t I2Cmaster::read(uint8_t address, uint8_t* data_rd, size_t size)
{
	esp_err_t err;
	if (size == 0) {
		return ESP_OK;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	err = i2c_master_start(cmd);
   	err = i2c_master_write_byte(cmd, ( address << 1 ) | READ_BIT, ACK_CHECK_EN);
	if (size > 1) {
		err = i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)ACK_VAL);
	}
	err = i2c_master_read_byte(cmd, data_rd + size - 1, (i2c_ack_type_t)NACK_VAL);
	err = i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
    ESP_LOGD("i2c", "Exit to read with %d", err);
    return err;
}

// -------------------------------------
I2Cslave::I2Cslave():I2Cdevice(I2C_DEFAULT_NUM, I2C_DEFAULT_SCL_IO, I2C_DEFAULT_SDA_IO) {};
I2Cslave::I2Cslave(i2c_port_t i2cNum):I2Cdevice(i2cNum, I2C_DEFAULT_SCL_IO, I2C_DEFAULT_SDA_IO) {};
I2Cslave::I2Cslave(i2c_port_t i2cNum, gpio_num_t signalGPIO, gpio_num_t clockGPIO):I2Cdevice(i2cNum, signalGPIO, clockGPIO) {};
I2Cslave::~I2Cslave()
{
}

esp_err_t I2Cslave::init()
{
  rxBufferDim = I2C_SLAVE_RX_BUF_DIMENSION;
  txBufferDim = I2C_SLAVE_TX_BUF_DIMENSION;
  conf.mode = I2C_MODE_SLAVE;
  conf.slave.addr_10bit_en = I2C_ADDR_BIT_7; // I2C_ADDR_BIT_10
  conf.slave.slave_addr = 0x00;
  i2c_param_config(i2c_port, &conf);
  return(i2c_driver_install(i2c_port, conf.mode,rxBufferDim,txBufferDim, 0));
}

void I2Cslave::setAddress(uint8_t address)
{
  conf.slave.slave_addr = address;
  return;
}

void I2Cslave::setBufferDim(size_t rxBufferSize, size_t txBufferSize)
{
  rxBufferDim = rxBufferSize;
  txBufferDim = txBufferSize;
  return;
}

// needs best buffer management
esp_err_t I2Cslave::write(uint8_t* data_wr, size_t size)
{
  esp_err_t ret;
  size_t d_size = i2c_slave_write_buffer(i2c_port, data_wr, size, 1000 / portTICK_RATE_MS);
  if (d_size == 0) {
    ret = ESP_FAIL;
  } else {
    ret = ESP_OK;
  }
  return ret;
}

size_t I2Cslave::read(uint8_t* data_rd, size_t maxSize)
{
  size_t read_size;
  read_size = i2c_slave_read_buffer(i2c_port, data_rd, maxSize, 1000 / portTICK_RATE_MS);
  return read_size;
}
