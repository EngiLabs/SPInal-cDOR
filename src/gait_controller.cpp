#include "gait_controller.h"

#include "config.h"
#include "kinematics.h"

#include <math.h>



GaitController::GaitController::GaitController(ServoDriver::Driver &driver)
    : servoDriver(driver),
      currentDirection(0),
      currentSpeed(0),
      active(false),
      tickCounter(0),
      swingRatio(0.25f)
{
    setType(Gait::WALK);
}

void GaitController::GaitController::setType(const uint8_t type) {
    switch (static_cast<Gait::Type>(type)) {
        case Gait::TROT:
            swingRatio = 0.5f;
            legPhaseOffsets[Servos::LEG_LF] = 0.0f;
            legPhaseOffsets[Servos::LEG_RB] = 0.0f;
            legPhaseOffsets[Servos::LEG_RF] = 0.5f;
            legPhaseOffsets[Servos::LEG_LB] = 0.5f;
            break;

        case Gait::CREEP:
            swingRatio = 0.25f;
            legPhaseOffsets[Servos::LEG_LF] = 0.0f;
            legPhaseOffsets[Servos::LEG_RF] = 0.25f;
            legPhaseOffsets[Servos::LEG_LB] = 0.5f;
            legPhaseOffsets[Servos::LEG_RB] = 0.75f;
            break;

        case Gait::WALK:
        default:
            swingRatio = 0.25f;
            legPhaseOffsets[Servos::LEG_RB] = 0.0f;
            legPhaseOffsets[Servos::LEG_RF] = 0.25f;
            legPhaseOffsets[Servos::LEG_LB] = 0.5f;
            legPhaseOffsets[Servos::LEG_LF] = 0.75f;
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
    const float phase = fmod(static_cast<float>(tickCounter) / Gait::CYCLE_LENGTH_TICK, 1.0f);

    for (uint8_t leg = 0; leg < Servos::LEGS; leg++) {
        const float legPhase = fmod(phase + legPhaseOffsets[leg], 1.0f);

        FootPosition foot{};
        kinematicsGetHomePosition(foot);

        const float stride = static_cast<float>(currentSpeed) / 255.0f * Gait::MAX_STRIDE;
        const bool isRightLeg = leg == Servos::LEG_RF || leg == Servos::LEG_RB;
        const bool isStrafe = currentDirection == 4 || currentDirection == 5;

        if (isStrafe) {
            const float strafeDir = currentDirection == 4
                ? (isRightLeg ? -1.0f :  1.0f)
                : (isRightLeg ?  1.0f : -1.0f);
            const float strafeStride = stride * 0.5f;

            foot.x = 0.0f;
            if (legPhase < swingRatio) { // перенос
                const float swingPhase = legPhase / swingRatio;
                foot.z = Kinematics::HOME_R + strafeStride * (swingPhase - 0.5f) * strafeDir;
                foot.y += Gait::LIFT_HEIGHT * sin(swingPhase * 3.14159265f);
            }
            else { // опора
                const float stancePhase = (legPhase - swingRatio) / (1.0f - swingRatio);
                foot.z = Kinematics::HOME_R + strafeStride * (0.5f - stancePhase) * strafeDir;
                foot.y = Kinematics::HOME_Y;
            }
        }
        else {
            float legDir = 0.0f;
            if      (currentDirection == 1)  legDir = 1.f;
            else if (currentDirection == -1) legDir = -1.f;
            else if (currentDirection == 2)  legDir = isRightLeg ?  1.f : -1.f;
            else if (currentDirection == 3)  legDir = isRightLeg ? -1.f :  1.f;

            if (legPhase < swingRatio) { // перенос
                const float swingPhase = legPhase / swingRatio;
                foot.x += stride * (swingPhase - 0.5f) * legDir;
                foot.y += Gait::LIFT_HEIGHT * sin(swingPhase * 3.14159265f);
            }
            else { // опора
                const float stancePhase = (legPhase - swingRatio) / (1.0f - swingRatio);
                foot.x += stride * (0.5f - stancePhase) * legDir;
                foot.y = Kinematics::HOME_Y;
            }

            constexpr float r2 = Kinematics::HOME_R * Kinematics::HOME_R;
            const float x2 = foot.x * foot.x;
            if (r2 > x2) foot.z = sqrt(r2 - x2);
            else foot.z = Kinematics::HOME_R;
        }

        LegAngles angles{};
        if (inverseKinematics(foot, angles) == IkStatus::OK) {
            servoDriver.setLegAngles(leg, angles);
        }
    }
}

bool GaitController::GaitController::isActive() const {
    return active;
}
