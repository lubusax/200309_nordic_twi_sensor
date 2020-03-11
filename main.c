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

// Common address definition for temperature sensor
#define BMP280_I2C_ADDR         BMP280_I2C_ADDR_PRIM

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

//==========================================================
// BMP280 Sensor - Specific variables and functions - BEGIN
//==========================================================

struct bmp280_dev           bmp;
struct bmp280_config        conf;
struct bmp280_uncomp_data   ucomp_data;
int32_t                     temp32;
double                      temp;
int8_t                      rslt;//error msg -bme280 driver functions
uint8_t                     chipSelectPin; // for I2C has to be set high


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
/**
 * @brief Function for sending data to a TWI slave.
 *
 * The transmission will be stopped when an error occurs. If a transfer is ongoing,
 * the function returns the error code @ref NRFX_ERROR_BUSY.
 *
 * @note This function is deprecated. Use @ref nrfx_twim_xfer instead.
 *
 * @note Peripherals using EasyDMA (including TWIM) require the transfer buffers
 *       to be placed in the Data RAM region. If this condition is not met,
 *       this function fails with the error code NRFX_ERROR_INVALID_ADDR.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] address    Address of a specific slave device (only 7 LSB).
 * @param[in] p_data     Pointer to a transmit buffer.
 * @param[in] length     Number of bytes to send. Maximum possible length is
 *                       dependent on the used SoC (see the MAXCNT register
 *                       description in the Product Specification). The driver
 *                       checks it with assertion.
 * @param[in] no_stop    If set, the stop condition is not generated on the bus
 *                       after the transfer has completed successfully (allowing
 *                       for a repeated start in the next transfer).
 *
 * @retval NRFX_SUCCESS                 The procedure is successful.
 * @retval NRFX_ERROR_BUSY              The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_INTERNAL          An unexpected transition occurred on the bus.
 * @retval NRFX_ERROR_INVALID_ADDR      The provided buffer is not placed in the Data RAM region.
 * @retval NRFX_ERROR_DRV_TWI_ERR_ANACK NACK is received after sending the address in polling mode.
 * @retval NRFX_ERROR_DRV_TWI_ERR_DNACK NACK is received after sending a data byte in polling mode.
 */
/* nrfx_err_t nrfx_twim_tx(nrfx_twim_t const * p_instance,
                        uint8_t             address,
                        uint8_t const *     p_data,
                        size_t              length,
                        bool                no_stop); */

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

    err_code = nrf_drv_twi_tx(&m_twi, BMP280_I2C_ADDR_PRIM,
                             temp_buff, (length+1) , false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    rslt = BMP280_OK;

    if (err_code != NRFX_SUCCESS )
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
 *  @param[out] reg_data: Pointer to the data buffer to store the read data.
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


    //first the register address must be sent
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(  &m_twi, i2c_addr,
                                reg_addr, 1 , false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    //the slave sends out data from
    //auto-incremented register addresses until
    //a NOACKM and stop condition occurs
    if (err_code != NRFX_SUCCESS )
    {
        rslt = BMP280_E_COMM_FAIL; 
    }
    else
    {
        m_xfer_done = false;
        err_code = nrf_drv_twi_rx(&m_twi, BMP280_I2C_ADDR_PRIM,
                                &reg_data, length);
        APP_ERROR_CHECK(err_code);
        while (m_xfer_done == false);

    
        if (err_code != NRFX_SUCCESS )
        {
            rslt = BMP280_E_COMM_FAIL; 
        }
    }

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
    /* Map the delay function pointer with the function
     responsible for implementing the delay */
    bmp.delay_ms = nrf_delay_ms;

    /* Assign device I2C address based on the status of 
    SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with
    the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);

    /* Always read the current settings before writing,
     * especially when all the configuration is not modified
     * */
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);

    /* configuring the temperature oversampling,
     * filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter     = BMP280_FILTER_COEFF_2;

    /* Temperature oversampling set at 4x */
    conf.os_temp    = BMP280_OS_4X;

    /* Pressure over sampling none (disabling pressure measurement) */
    conf.os_pres    = BMP280_OS_NONE;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr        = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);
}
//==========================================================
// END - END ---- BMP280 - Specific variables and functions 
//==========================================================

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp      Temperature in Celsius degrees
 *                      read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("data handler - NRF_DRV_TWI_EVT_DONE", temp);
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
    /* Reading the raw data from sensor */
    rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

    /* Getting the 32 bit compensated temperature */
    rslt = bmp280_get_comp_temp_32bit(&temp32,
                                ucomp_data.uncomp_temp, &bmp);

    /* Getting the compensated temperature as floating point value */
    rslt = bmp280_get_comp_temp_double(&temp,
                                    ucomp_data.uncomp_temp, &bmp);
    NRF_LOG_INFO("UT: %ld, T32: %ld, T: %f \r\n",
         ucomp_data.uncomp_temp, temp32, temp);

    /* Sleep time between measurements = BMP280_ODR_1000_MS */
    bmp.delay_ms(1000);
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

    while (true)
    {
        // nrf_delay_ms(1000);

        // do
        // {
        //     __WFE();
        // }while (m_xfer_done == false);

        read_sensor_data(); // has a delay of 1 second inside
        NRF_LOG_FLUSH();
    }
}
/** @} */