/* Firmware entry point: bring up the board, then run the main loop forever. */
#include "app.h"
#include "hal.h"

int main(void)
{
    hal_init();
    app_init();
    hal_start();

    uint32_t last = hal_millis();
    for (;;) {
        app_poll();
        while (last != hal_millis()) { // catch up if the loop was slow, so no tick is lost
            last++;
            app_tick();
        }
    }
}
