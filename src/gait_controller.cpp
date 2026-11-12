#include "gait_controller.h"

#include <math.h>
#include "config.h"
#include "servo_driver.h"
#include "kinematics.h"


static int8_t currentDirection = 0;
static uint8_t currentSpeed = 0;
static bool active = false;
static uint32_t tickCounter = 0;

static float legPhaseOffsets[Servo::LEGS] = {0.0f, 0.5f, 0.75f, 0.25f};

void gaitInit() {
    active = false;
    tickCounter = 0;
}

void gaitSetType(uint8_t gaitType) {
    switch (static_cast<Gait::Type>(gaitType)) {
        case Gait::WALK:
            legPhaseOffsets[0] = 0.0f;
            legPhaseOffsets[1] = 0.5f;
            legPhaseOffsets[2] = 0.75f;
            legPhaseOffsets[3] = 0.25f;
            break;
        case Gait::TROT:
            legPhaseOffsets[0] = 0.0f;
            legPhaseOffsets[1] = 0.5f;
            legPhaseOffsets[2] = 0.5f;
            legPhaseOffsets[3] = 0.0f;
            break;
        case Gait::CREEP:
            legPhaseOffsets[0] = 0.0f;
            legPhaseOffsets[1] = 0.25f;
            legPhaseOffsets[2] = 0.5f;
            legPhaseOffsets[3] = 0.75f;
            break;
    }
}

void gaitSetDirection(const int8_t direction, const uint8_t speed) {
    currentDirection = direction;
    currentSpeed = speed;
    active = true;
}

void gaitStop() {
    active = false;
    currentDirection = 0;
    currentSpeed = 0;
}

void gaitTick() {
    if (!active) return;

    tickCounter++;
    constexpr float cycleLength = 50.0f;
    const float phase = fmod(static_cast<float>(tickCounter) / cycleLength, 1.0f);

    for (uint8_t leg = 0; leg < Servo::LEGS; leg++) {
        const float legPhase = fmod(phase + legPhaseOffsets[leg], 1.0f);

        FootPosition foot{};
        kinematicsGetHomePosition(foot);

        const float stride = static_cast<float>(currentSpeed) / 255.0f * 30.0f;

        if (legPhase < 0.5f) {
            const float swingPhase = legPhase * 2.0f;
            foot.z += stride * (swingPhase - 0.5f) * static_cast<float>(currentDirection);
            foot.y += 20.0f * sin(swingPhase * 3.14159265f);
        }
        else {
            const float stancePhase = (legPhase - 0.5f) * 2.0f;
            foot.z += stride * (0.5f - stancePhase) * static_cast<float>(currentDirection);
        }

        LegAngles angles{};
        if (inverseKinematics(foot, angles) == IkStatus::OK)
            servoSetLegAngles(leg, angles);
    }
}

bool gaitIsActive() {
    return active;
}
