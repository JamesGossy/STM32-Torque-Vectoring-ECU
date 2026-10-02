/* Dials from the steering wheel panel, received over CAN. Each is a fraction
   from 0 to 1. Before the first frame they sit at 1 (no change). If the panel
   goes quiet the last values are kept, so a lost cable never raises a limit. */
#ifndef PANEL_H
#define PANEL_H

#include <stdint.h>

typedef struct {
    float tv;    // torque vectoring strength
    float power; // share of the drive torque limit
    float regen; // share of the regen torque limit
} PanelDials;

void panel_init(void);

// Returns 1 if the frame was a panel frame.
int panel_handle_frame(uint32_t id, const uint8_t *data, uint8_t len);

const PanelDials *panel_dials(void);

#endif
