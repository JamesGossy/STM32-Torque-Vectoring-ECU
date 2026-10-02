/* Steering wheel panel frames. The panel also lights its LEDs from the ECU's
   own STATUS frame (state and inhibit bits), so the ECU sends nothing extra. */
#include "panel.h"
#include "can_protocol.h"
#include "util.h"

static PanelDials dials;

void panel_init(void)
{
    dials = (PanelDials) { 1.0f, 1.0f, 1.0f };
}

static float fraction(uint8_t percent)
{
    return clampf(percent / 100.0f, 0.0f, 1.0f);
}

int panel_handle_frame(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (CAN_NODE(id) != PANEL_NODE || CAN_MSG(id) != PANEL_MSG_DIALS || len < 3) return 0;
    dials.tv    = fraction(data[0]);
    dials.power = fraction(data[1]);
    dials.regen = fraction(data[2]);
    return 1;
}

const PanelDials *panel_dials(void)
{
    return &dials;
}
