/** @file * @defgroup tw_sensor_example main.c * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.*/

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bmp280.h"
#include "bmp280_defs.h"


// TWI instance ID
#define TWI_INSTANCE_ID     0

// Common addresses definition for temperature sensor
#define BMP280_I2C_ADDR         BMP280_I2C_ADDR_PRIM

// Nordic functions use this address (shifted)
// BMP280 driver uses the original address (not shifted)  
#define BMP280_ADDR          ( BMP280_I2C_ADDR >> 1) 

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

//==========================================================
// BMP280 Sensor - Specific variables and functions - BEGIN
//==========================================================

struct bmp280_dev bmp;

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 * Implement the I2C write routine according to the Nordic pca100056. 
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
/* Writing is done by sending the slave address in write mode (RW = ‘0’),
resulting in slave address 111011X0 (‘X’ is determined by state of SDO pin.)
Then the master sends pairs of register addresses and register data.
The transaction is ended by a stop condition.  */

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr,
                     uint8_t *reg_data, uint16_t length)
{
    int8_t rslt;
    uint8_t index;
    ret_code_t err_code;
    uint8_t bmp280_write_addr = ( i2c_addr << 1); 

    uint8_t temp_buff[8]; // Typically not to write more than 4 registers

    temp_buff[0] = reg_addr;
    temp_buff[1] = reg_data[0];

    if (length > 1)
    {
        for (index = 1; index < length; index = index+2) // length is always odd:1,3,5...
        {
            temp_buff[index + 1]  = reg_data[index];
            temp_buff[index + 2]  = reg_data[index + 1];
        }
    }
    
    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi, bmp280_write_addr,
                             temp_buff, (length+1) , false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    rslt = BMP280_OK;
    if (err_code)
    {
        rslt = BMP280_E_COMM_FAIL;
    }

    return rslt;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
/* To be able to read registers, first
the register address must be sent
in write mode (slave address 111011X0).
Then either a stop or a repeated start condition must be generated.
After this the slave is addressed in read mode (RW = ‘1’)
at address 111011X1,
after which the slave sends out data from
auto-incremented register addresses until
a NOACKM and stop condition occurs. */

int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr,
                    uint8_t *reg_data, uint16_t length)
{/* Implement the I2C read routine according to the target machine. */
    int8_t rslt;
    uint8_t index;
    ret_code_t err_code;
    uint8_t bmp280_write_addr = ( i2c_addr << 1);
    uint8_t bmp280_read_addr = ( i2c_addr << 1) | 0x01;
    

    uint8_t temp_buff[8]; // Typically not to write more than 4 registers

    return rslt;
}

void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        NRF_LOG_INFO("BMP280 driver - %s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            NRF_LOG_INFO("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            NRF_LOG_INFO("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            NRF_LOG_INFO("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            NRF_LOG_INFO("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            NRF_LOG_INFO("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

void bmp280_setup(void)
{
    int8_t rslt;  // error message from bme280 driver functions
    
    /* Map the delay function pointer with the function
     responsible for implementing the delay */
    bmp.delay_ms = nrf_delay_ms;

    /* Assign device I2C address based on the status of 
    SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with
    the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);
}

//==========================================================
// END - END ---- BMP280 - Specific variables and functions 
//==========================================================


/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    /* ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code); */
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{   
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    bmp280_setup();
    
    //LM75B_set_mode();

    while (true)
    {
        nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        read_sensor_data();
        NRF_LOG_FLUSH();
    }
}

/** @} */
