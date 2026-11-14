#include "gait_controller.h"

#include "config.h"
#include "kinematics.h"

#include <math.h>


GaitController::GaitController::GaitController() {
    active = false;
    tickCounter = 0;
}

void GaitController::GaitController::setType(uint8_t type) {
    switch (static_cast<Gait::Type>(type)) {
        case Gait::WALK:
            legPhaseOffsets[Servos::LEG_LF] = 0.0f;
            legPhaseOffsets[Servos::LEG_RB] = 0.25f;
            legPhaseOffsets[Servos::LEG_RF] = 0.5f;
            legPhaseOffsets[Servos::LEG_LB] = 0.75f;
            break;
        case Gait::TROT:
            legPhaseOffsets[Servos::LEG_LF] = 0.0f;
            legPhaseOffsets[Servos::LEG_RB] = 0.0f;
            legPhaseOffsets[Servos::LEG_RF] = 0.5f;
            legPhaseOffsets[Servos::LEG_LB] = 0.5f;
            break;
        case Gait::CREEP:
            legPhaseOffsets[Servos::LEG_LF] = 0.0f;
            legPhaseOffsets[Servos::LEG_RF] = 0.25f;
            legPhaseOffsets[Servos::LEG_LB] = 0.5f;
            legPhaseOffsets[Servos::LEG_RB] = 0.75f;
            break;
    }
}

void GaitController::GaitController::setDirection(const int8_t direction, const uint8_t speed) {
    currentDirection = direction;
    currentSpeed = speed;
    active = true;
}

void GaitController::GaitController::stop() {
    active = false;
    currentDirection = 0;
    currentSpeed = 0;
}

void GaitController::GaitController::tick() {
    if (!active) return;

    tickCounter++;
    constexpr float cycleLength = 50.0f;
    const float phase = fmod(static_cast<float>(tickCounter) / cycleLength, 1.0f);

    for (uint8_t leg = 0; leg < Servos::LEGS; leg++) {
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
            servoDriver.setLegAngles(leg, angles);
    }
}

bool GaitController::GaitController::isActive() const {
    return active;
}
