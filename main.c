#include "nrf_drv_gpiote.h"
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

int main(void)
{   
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("\r\n GPIO example started.");
    NRF_LOG_FLUSH();
    

    while (true)
    {
        nrf_delay_ms(4000);

        NRF_LOG_FLUSH();
    }
}
