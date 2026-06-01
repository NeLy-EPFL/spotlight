#pragma once

#include <cstddef>

#include <comm_protocol/protocol.h>

#include "trigger_firmware/serial_io.h"
#include "trigger_firmware/status_display.h"


class TriggerController{
  public:
    TriggerController();

    /** Run one cycle of the control loop: check for new commands, update the
     * display, and set outputs accordingly. */
    void update();

  private:
    SerialIO serialIO_;
    StatusDisplay display_;
};
