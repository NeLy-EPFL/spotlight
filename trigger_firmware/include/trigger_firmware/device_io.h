#pragma once

#include <comm_protocol/protocol.h>

/**
 * Tracks and drives the state of every output the trigger controller owns: the
 * behavior and muscle camera triggers, the IR and blue excitation LEDs, and the
 * optogenetics channels. Each setter is idempotent: it only touches the
 * corresponding pin when the cached state actually changes.
 *
 * There is a single set of physical outputs, so this is a singleton: access the
 * sole instance through DeviceIO::get_instance().
 */
class DeviceIO {
  public:
    static DeviceIO &get_instance();

    DeviceIO(const DeviceIO &) = delete;
    DeviceIO &operator=(const DeviceIO &) = delete;

    // Behavior camera trigger and IR illumination LED.
    void start_beh_cam_trigger();
    void stop_beh_cam_trigger();
    void turn_on_beh_led();
    void turn_off_beh_led();

    // Muscle camera trigger and blue excitation LED.
    void start_musc_cam_trigger();
    void stop_musc_cam_trigger();
    bool is_musc_common_time();
    void turn_on_musc_led();
    void turn_off_musc_led();

    // Optogenetics channels. `channel` is ch2 or ch3 for a single channel, or
    // all to act on both at once. Returns false for an unknown channel.
    bool turn_on_opto_ch(OptoChannel channel);
    bool turn_off_opto_ch(OptoChannel channel);
    bool is_opto_ch_on(OptoChannel channel);

    // Drive every output to its safe default (triggers stopped, LEDs and opto
    // channels off).
    void reset();

  private:
    DeviceIO();

    bool is_beh_cam_trigger_on_;
    bool is_beh_led_on_;
    bool is_musc_cam_trigger_on_;
    bool is_musc_led_on_;
    bool is_opto_ch2_on_;
    bool is_opto_ch3_on_;
};
