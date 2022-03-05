#include "main.h"
#include "robot-config.h"
#include "robot-functions.h"
#include "odom-utilities.h"
#include "auton-drive.h"
#include "controller-menu.h"

#include "odometry.h"

#include <bits/stdc++.h>

std::shared_ptr<ImuOdom> imu_odom;

void odom_init() {
  imu_odom = std::make_shared<ImuOdom>(TimeUtilFactory().createDefault(),
      chassis->getOdometry()->getModel(),
      chassis->getOdometry()->getScales());
  pros::Task([&](){ imu_odom->loop(); });
}

ImuOdom::ImuOdom(const TimeUtil &itimeUtil,
    const std::shared_ptr<ReadOnlyChassisModel> &imodel,
    const ChassisScales &ichassisScales,
    const std::shared_ptr<Logger> &logger)
    : TwoEncoderOdometry(itimeUtil, imodel, ichassisScales, logger) {
  // if (ichassisScales.middle == 0) {
  //   std::string msg = "ThreeEncoderOdometry: Middle scale cannot be zero.";
  //   LOG_ERROR(msg);
  //   throw std::invalid_argument(msg);
  // }
}

void ImuOdom::step() {
  const auto deltaT = timer->getDt();

  if (deltaT.getValue() != 0) {
    newTicks = model->getSensorVals();
    tickDiff = newTicks - lastTicks;
    lastTicks = newTicks;

    newImu = imu.get_rotation() * degree;
    imuDiff = newImu - lastImu;
    lastImu = newImu;

    const auto newState = ImuOdomMathStep(tickDiff, deltaT, imuDiff);

    state.x += newState.x;
    state.y += newState.y;
    state.theta += newState.theta;
  }
}

void ImuOdom::loop() {
  LOG_INFO_S("Started ImuOdom task.");

  Rate rate;
  while (true) {
    step();
    state.theta = imu.get_rotation()*degree;
    rate.delayUntil(10_ms);
  }
  LOG_INFO_S("Stopped ImuOdom task.");
}

void ImuOdom::setState(const OdomState &istate, const StateMode &imode) {
  LOG_DEBUG("State set to: " + istate.str());
  if (imode == StateMode::FRAME_TRANSFORMATION) {
    state = istate;
  } else {
    state = OdomState{istate.y, istate.x, istate.theta};
  }
  imu.set_rotation(istate.theta.convert(degree));
}

OdomState ImuOdom::getState(const StateMode &imode) const {
  if (imode == StateMode::FRAME_TRANSFORMATION) {
    return state;
  } else {
    return OdomState{state.y, state.x, state.theta};
  }
}

OdomState ImuOdom::ImuOdomMathStep(
    const std::valarray<std::int32_t> &itickDiff,
    const QTime &ideltaT, QAngle imuDiff) {

  if (itickDiff.size() < 2) {
    LOG_ERROR_S("TwoEncoderOdometry: itickDiff did not have at least three elements.");
    return OdomState{};
  }

  for (auto &&elem : itickDiff) {
    if (std::abs(elem) > maximumTickDiff) {
      LOG_ERROR("TwoEncoderOdometry: A tick diff (" + std::to_string(elem) +
                ") was greater than the maximum allowable diff (" +
                std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
      return OdomState{};
    }
  }

  const double deltaL = itickDiff[0] / chassisScales.straight;
  const double deltaR = itickDiff[1] / chassisScales.straight;

  // double deltaTheta = (deltaL - deltaR) / chassisScales.wheelTrack.convert(meter);
  double deltaTheta = imuDiff.convert(radian);
  if (abs(deltaTheta) > 0.01) {
    deltaTheta = (deltaL - deltaR) / chassisScales.wheelTrack.convert(meter);
  }

  double localOffX, localOffY;

  if (deltaTheta != 0) {
  localOffX = 2 * std::sin(deltaTheta / 2) * chassisScales.middleWheelDistance.convert(meter);
  localOffY = 2 * std::sin(deltaTheta / 2) *
              (deltaR / deltaTheta + chassisScales.wheelTrack.convert(meter) / 2);
  } else {
  localOffX = 0;
  localOffY = deltaR;
  }

  double avgA = state.theta.convert(radian) + (deltaTheta / 2);

  double polarR = std::sqrt((localOffX * localOffX) + (localOffY * localOffY));
  double polarA = std::atan2(localOffY, localOffX) - avgA;

  double dX = std::sin(polarA) * polarR;
  double dY = std::cos(polarA) * polarR;

  if (isnan(dX)) {
    dX = 0;
  }

  if (isnan(dY)) {
    dY = 0;
  }

  if (isnan(deltaTheta) || isinf(deltaTheta)) {
    deltaTheta = 0;
  }

  // controllermenu::partner_print_array[0] = "dt " + std::to_string(deltaTheta);
  // controllermenu::partner_print_array[1] = "imu " + std::to_string(imu.get_rotation());

  return OdomState{dX * meter, dY * meter, deltaTheta * radian};
}