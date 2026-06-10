// HARDWARE test (requires a real JAI camera on the Euresys frame grabber).
//
// This is intentionally NOT part of the normal `recorder_tests` suite: that one
// never touches a physical device, whereas this opens and configures the behavior
// camera. It lives in its own executable (`recorder_hardware_tests`) and is tagged
// with the CTest label "hardware" so it is excluded from the default run. See
// docs/setup/building.md for how to run (or skip) it.
//
// Purpose: isolate behavior-camera bring-up (`BehaviorCamera` -> GenTL discovery +
// GenICam configuration) from the rest of the apps. This target does NOT link the
// PCO SDK, so a failure here implicates the JAI/Euresys path alone, independent of
// the muscle camera. On failure it reports the exception's dynamic type and (for
// Euresys exceptions) the GenTL error code/description, so the real cause is
// visible instead of a bare "std::exception".

#include "recorder/peripherals/behavior_camera.h"

#include <memory>
#include <tuple>
#include <typeinfo>

#include <EGenTLErrors.h> // Euresys::gentl_error
#include <gtest/gtest.h>

namespace {
// ROI matching the example profile (recorder/etc/profile_example/recorder_config.yaml):
// full sensor 2560x2048, behaviour ROI 1984x1500, centered. The exact values do
// not matter for a configuration smoke test, only that they are valid.
constexpr int kFullFrameWidth = 2560;
constexpr int kFullFrameHeight = 2048;
constexpr int kRoiWidth = 1984;
constexpr int kRoiHeight = 1500;
constexpr const char *kTriggerLine = "TTLIO12";
} // namespace

// Constructing a BehaviorCamera runs the full bring-up: GenTL discovery, opening
// the grabber, and applying the GenICam configuration (configure()). If any of
// that throws, the test fails with the real exception details.
TEST(BehaviorCameraHardware, ConstructsAndConfigures) {
    int width = roundToNearestValidBehaviorCamDimension(kRoiWidth);
    int height = roundToNearestValidBehaviorCamDimension(kRoiHeight);
    auto [xOffset, yOffset] =
        getCenteredOffsets(width, height, kFullFrameWidth, kFullFrameHeight);

    std::unique_ptr<BehaviorCamera> camera;
    try {
        camera = std::make_unique<BehaviorCamera>(
            static_cast<unsigned int>(width),
            static_cast<unsigned int>(height),
            static_cast<unsigned int>(xOffset),
            static_cast<unsigned int>(yOffset),
            kTriggerLine);
    } catch (const Euresys::gentl_error &e) {
        FAIL() << "BehaviorCamera construction threw Euresys::gentl_error "
               << static_cast<int>(e.gc_err)
               << (e.description.empty() ? "" : " (" + e.description + ")")
               << ": " << e.what();
    } catch (const std::exception &e) {
        FAIL() << "BehaviorCamera construction threw [" << typeid(e).name()
               << "]: " << e.what();
    }

    EXPECT_TRUE(camera->isReady())
        << "BehaviorCamera constructed but did not report ready.";
}
