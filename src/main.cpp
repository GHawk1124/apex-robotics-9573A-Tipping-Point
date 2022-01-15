#include "main.h"
#include "constants.h"

#define TANK
#define AUTON
#define DEBUG
static Controller masterController;

static ControllerButton liftButton(ControllerDigital::R2);
static ControllerButton liftShutoff(ControllerDigital::L2);
static ControllerButton lift90Button(ControllerDigital::R1);
static ControllerButton lift90Shutoff(ControllerDigital::L1);
static ControllerButton lowerHookButton(ControllerDigital::A);
static ControllerButton liftHookButton(ControllerDigital::B);
#ifdef DEBUG
static ControllerButton autoButton(ControllerDigital::X);
#endif

static Motor LFM(15, true, AbstractMotor::gearset::blue,
                 AbstractMotor::encoderUnits::rotations);
static Motor LMM(11, true, AbstractMotor::gearset::blue,
                 AbstractMotor::encoderUnits::rotations);
static Motor LBM(1, true, AbstractMotor::gearset::blue,
                 AbstractMotor::encoderUnits::rotations);
static Motor RFM(16, false, AbstractMotor::gearset::blue,
                 AbstractMotor::encoderUnits::rotations);
static Motor RMM(20, false, AbstractMotor::gearset::blue,
                 AbstractMotor::encoderUnits::rotations);
static Motor RBM(6, false, AbstractMotor::gearset::blue,
                 AbstractMotor::encoderUnits::rotations);
static Motor liftMotor(18, false, AbstractMotor::gearset::red,
                       AbstractMotor::encoderUnits::rotations);
static Motor lift90Motor(8, false, AbstractMotor::gearset::red,
                         AbstractMotor::encoderUnits::rotations);

static MotorGroup leftG({LFM, LMM, LBM});
static MotorGroup rightG({RFM, RMM, RBM});

static bool lowerHookState = false;
static bool liftHookState = false;
static pros::ADIDigitalOut lowerHook(7, lowerHookState);
static pros::ADIDigitalOut liftHook(6, liftHookState);

// static Potentiometer liftPotent('H');
// static std::shared_ptr<Potentiometer> liftPotAddr =
// std::make_shared<Potentiometer>(&liftPot);

static std::shared_ptr<Potentiometer> liftPot(new Potentiometer('H'));
static std::shared_ptr<Potentiometer> lift90Pot(new Potentiometer('E'));

static std::shared_ptr<ChassisController> drive;
static std::shared_ptr<OdomChassisController> autoDrive;

static std::shared_ptr<AsyncPositionController<double, double>> liftController;
static std::shared_ptr<AsyncPositionController<double, double>>
    lift90Controller;

void taskLift() {
  bool liftUp = false;
  while (true) {
    if (liftButton.changedToPressed()) {
      liftUp = !liftUp;
    }
    /*if (liftShutoff.changedToPressed()) {
      break;
    }*/
    if (liftUp) {
      if (liftPot->get() < 2550) {
        liftMotor.moveVoltage(
            12000 * std::clamp(std::fabs((2550 - liftPot->get()) / 1000.0l),
                               0.9l, 1.0l));
      } else {
        liftMotor.moveVoltage(
            -12000 * std::clamp(std::fabs((2550 - liftPot->get()) / 1000.0l),
                                0.05l, 1.0l));
      }
    } else {
      if (liftPot->get() > 600) {
        liftMotor.moveVoltage(
            -12000 * std::clamp(std::fabs((600 - liftPot->get()) / 1000.0l),
                                0.75l, 1.0l));
      } else {
        liftMotor.moveVoltage(
            12000 * std::clamp(std::fabs((600 - liftPot->get()) / 1000.0l),
                               0.05l, 1.0l));
      }
    }
    pros::Task::delay(20);
  }
  liftMotor.moveVoltage(0);
}

int liftAutoVal;
bool liftAuto;

void taskLiftAuto() {
  if (liftAuto) {
    while (true) {
      if (liftPot->get() < liftAutoVal) {
        liftMotor.moveVoltage(
            12000 * std::clamp(std::fabs((2550 - liftPot->get()) / 1000.0l),
                               0.9l, 1.0l));
      } else {
        liftMotor.moveVoltage(
            -12000 * std::clamp(std::fabs((2550 - liftPot->get()) / 1000.0l),
                                0.05l, 1.0l));
      }
      if (liftPot->get() > 600) {
        liftMotor.moveVoltage(
            -12000 * std::clamp(std::fabs((600 - liftPot->get()) / 1000.0l),
                                0.75l, 1.0l));
      } else {
        liftMotor.moveVoltage(
            12000 * std::clamp(std::fabs((600 - liftPot->get()) / 1000.0l),
                               0.05l, 1.0l));
      }
      pros::Task::delay(20);
    }
  }
  liftMotor.moveVoltage(0);
}

int lift90val;
bool task90Auto;

void task90LiftAuto() {
  if (task90Auto) {
    while (true) {
      if (lift90Pot->get() < lift90val) {
        lift90Motor.moveVoltage(
            12000 * std::clamp(std::fabs((3250 - lift90Pot->get()) / 1000.0l),
                               0.3l, 1.0l));
      } else {
        lift90Motor.moveVoltage(
            -12000 * std::clamp(std::fabs((3250 - lift90Pot->get()) / 1000.0l),
                                0.025l, 1.0l));
      }
      if (lift90Pot->get() > 1400) {
        lift90Motor.moveVoltage(
            -12000 * std::clamp(std::fabs((1400 - lift90Pot->get()) / 1000.0l),
                                0.25l, 1.0l));
      } else {
        lift90Motor.moveVoltage(
            12000 * std::clamp(std::fabs((1400 - lift90Pot->get()) / 1000.0l),
                               0.025l, 1.0l));
      }
    }
    pros::Task::delay(20);
  }
  lift90Motor.moveVoltage(0);
}

void task90Lift() {
  bool lift90Up = true;
  while (true) {
    if (lift90Button.changedToPressed()) {
      lift90Up = !lift90Up;
    }
    if (lift90Up) {
      if (lift90Pot->get() < 3000) {
        lift90Motor.moveVoltage(
            12000 * std::clamp(std::fabs((3000 - lift90Pot->get()) / 1000.0l),
                               0.5l, 1.0l));
      } else {
        lift90Motor.moveVoltage(
            -12000 * std::clamp(std::fabs((3000 - lift90Pot->get()) / 1000.0l),
                                0.05l, 1.0l));
      }
    } else {
      if (lift90Pot->get() > 1350) {
        lift90Motor.moveVoltage(
            -12000 * std::clamp(std::fabs((1350 - lift90Pot->get()) / 1000.0l),
                                0.25l, 1.0l));
      } else {
        lift90Motor.moveVoltage(
            12000 * std::clamp(std::fabs((1350 - lift90Pot->get()) / 1000.0l),
                               0.05l, 1.0l));
      }
    }
    pros::Task::delay(20);
  }
  lift90Motor.moveVoltage(0);
}

void initialize() {
#ifdef DEBUG
  pros::lcd::initialize();
  if (pros::battery::get_capacity() > 20) {
    if (!leftG.isOverCurrent() && !rightG.isOverCurrent() &&
        !leftG.isOverTemp() && !rightG.isOverTemp()) {
#endif
      drive =
          ChassisControllerBuilder()
              .withMotors(leftG, rightG)
              .withDimensions({AbstractMotor::gearset::blue, (36.0f / 60.0f)},
                              {{2.75_in, 14.5_in}, imev5BlueTPR})
              .build();
      leftG.setBrakeMode(AbstractMotor::brakeMode::coast);
      rightG.setBrakeMode(AbstractMotor::brakeMode::coast);
      liftMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
      lift90Motor.setBrakeMode(AbstractMotor::brakeMode::hold);
#ifdef DEBUG
    }
  } else {
    while (true) {
      std::cout << "Battery Too Low"
                << "\n";
      pros::delay(1000);
    }
  }
#endif
}

void disabled() {
  leftG.moveVoltage(0);
  rightG.moveVoltage(0);
  liftMotor.moveVoltage(0);
  lift90Motor.moveVoltage(0);
  // lowerHook.set_value(true);
  // liftHook.set_value(true);
}

void competition_initialize() {}

void Reverse() {
  leftG.setReversed(!LFM.isReversed());
  rightG.setReversed(!RFM.isReversed());
}

void autonomous() {
#ifdef AUTON
  liftAuto = true;
  liftAutoVal = 2550;
  // lift90val = 3250;
  // pros::Task lift90AutoTask(task90LiftAuto);
  pros::Task liftAutoTask(taskLiftAuto);
  autoDrive =
      ChassisControllerBuilder()
          .withMotors(leftG, rightG)
          .withGains({0.0006, 0.0, 0.0}, {0.0005, 0.0, 0.0}, {0.0005, 0.0, 0.0})
          .withDimensions({AbstractMotor::gearset::blue, (36.0f / 60.0f)},
                          {{2.75_in, 14.5_in}, imev5BlueTPR})
          .withOdometry({{2.75_in, 14.5_in}, imev5BlueTPR * (60.0f / 36.0f)},
                        StateMode::FRAME_TRANSFORMATION)
          .buildOdometry();
  // liftController = AsyncPosControllerBuilder()
  // .withMotor(liftMotor)
  // .withGearset(AbstractMotor::gearset::red)
  // .withGains({0.0, 0.0, 0.0})
  // .build();
  // lift90Controller = AsyncPosControllerBuilder()
  // .withMotor(lift90Motor)
  // .withGearset(AbstractMotor::gearset::red)
  // .withGains({0.0, 0.0, 0.0})
  // .build();
  autoDrive->setState({0_in, 0_in, 0_deg});
  // Reverse();
  // lift90val = 1350;
  pros::delay(1000);
  autoDrive->driveToPoint({4.6_ft, 0_ft}, false);
  // autoDrive->setState({0_in, 0_in, 0_deg});
  // lift90val = 3250;
  // autoDrive->driveToPoint({4.5_ft, 0_ft});
  liftHook.set_value(false);
  liftAuto = false;
  // autoDrive->driveToPoint({1.25_ft, 0_ft}, true);
  // autoDrive->driveToPoint({2.25_ft, 0_ft});
  // Reverse();
  // autoDrive->turnAngle(90_deg);
  // autoDrive->driveToPoint({2.25_ft, 1.25_ft});
  // liftHook.set_value(true);
  // Reverse();
  // autoDrive->driveToPoint({2.25_ft, 0_ft});
  // Reverse();
#endif
}

void opcontrol() {
  pros::Task lift90Task(task90Lift);
  pros::Task liftTask(taskLift);
#ifdef DEBUG
  Logger::setDefaultLogger(
      std::make_shared<Logger>(TimeUtilFactory::createDefault().getTimer(),
                               "/ser/sout", Logger::LogLevel::debug));
#endif
  while (true) {
    std::uint32_t now = pros::millis();
#ifdef DEBUG
    if (autoButton.changedToPressed()) {
      autonomous();
    }
    // pros::lcd::set_text(7, std::to_string(liftPot->get()));
    // pros::lcd::set_text(7, std::to_string(lift90Pot->get()));
    // std::cout << liftPot->get() << "\n";
#endif
    if (lowerHookButton.changedToPressed()) {
      lowerHookState = !lowerHookState;
    }
    if (liftHookButton.changedToPressed()) {
      liftHookState = !liftHookState;
    }
    lowerHook.set_value(lowerHookState);
    liftHook.set_value(liftHookState);
#ifdef TANK
    drive->getModel()->tank(
        masterController.getAnalog(ControllerAnalog::leftY),
        masterController.getAnalog(ControllerAnalog::rightY));
#endif
#ifdef ARCADE
    drive->getModel()->arcade(
        masterController.getAnalog(ControllerAnalog::leftY),
        masterController.getAnalog(ControllerAnalog::rightX));
#endif
    // if (lift90UpButton.isPressed()) {
    // lift90Motor.moveVoltage(12000);
    // } else if (lift90DownButton.isPressed()) {
    // lift90Motor.moveVoltage(-12000);
    // } else {
    // lift90Motor.moveVoltage(1000);
    // }
    // if (liftButton.changedToPressed()) {
    // liftController->setTarget(2.5);
    // liftController->setTarget(190);
    // pros::Task liftTask(toggleLift);
    // }
    pros::Task::delay_until(&now, driver_control_update);
  }
}
