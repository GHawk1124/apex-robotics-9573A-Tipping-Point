#include "main.h"
#include "constants.h"

#define DEBUG

static Controller masterController;

static ControllerButton liftUpButton(ControllerDigital::R2);
static ControllerButton liftDownButton(ControllerDigital::L2);
static ControllerButton lift90UpButton(ControllerDigital::L1);
static ControllerButton lift90DownButton(ControllerDigital::R1);
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

static Potentiometer liftPot(8);

static std::shared_ptr<ChassisController> drive;
static std::shared_ptr<OdomChassisController> autoDrive;

static std::shared_ptr<AsyncPositionController<double, double>> liftController;
static std::shared_ptr<AsyncPositionController<double, double>>
    lift90Controller;

void initialize() {
  pros::lcd::initialize();
#ifdef DEBUG
  if (pros::battery::get_capacity() > 20) {
#endif
    if (!leftG.isOverCurrent() && !rightG.isOverCurrent() &&
        !leftG.isOverTemp() && !rightG.isOverTemp()) {
      drive =
          ChassisControllerBuilder()
              .withMotors(leftG, rightG)
              .withDimensions({AbstractMotor::gearset::blue, (36.0f / 60.0f)},
                              {{2.75_in, 14.5_in}, imev5BlueTPR})
              .build();
      liftController = AsyncPosControllerBuilder()
                           .withMotor(liftMotor)
                           .withGearset(AbstractMotor::gearset::red)
                           // .withGains({0.0, 0.0, 0.0})
                           .build();
      lift90Controller = AsyncPosControllerBuilder()
                             .withMotor(lift90Motor)
                             .withGearset(AbstractMotor::gearset::red)
                             // .withGains({0.0, 0.0, 0.0})
                             .build();
      leftG.setBrakeMode(AbstractMotor::brakeMode::coast);
      rightG.setBrakeMode(AbstractMotor::brakeMode::coast);
      liftMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
      lift90Motor.setBrakeMode(AbstractMotor::brakeMode::hold);
    }
#ifdef DEBUG
  } else {
    while (true) {
      std::cout << "Battery Too Low" << std::endl;
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
}

void competition_initialize() {}

void Reverse() {
  leftG.setReversed(!LFM.isReversed());
  rightG.setReversed(!RFM.isReversed());
}

void autonomous() {
  autoDrive =
      ChassisControllerBuilder()
          .withMotors(leftG, rightG)
          .withGains({0.0005, 0.0, 0.0}, {0.0005, 0.0, 0.0}, {0.0005, 0.0, 0.0})
          .withDimensions({AbstractMotor::gearset::blue, (36.0f / 60.0f)},
                          {{2.75_in, 14.5_in}, imev5BlueTPR})
          .withOdometry({{2.75_in, 14.5_in}, imev5BlueTPR * (60.0f / 36.0f)},
                        StateMode::FRAME_TRANSFORMATION)
          .buildOdometry();
  autoDrive->setState({0_in, 0_in, 0_deg});
  Reverse();
  autoDrive->driveToPoint({-4.5_ft, 0_ft});
  lowerHook.set_value(true);
  autoDrive->driveToPoint({-2.25_ft, 0_ft});
  Reverse();
  autoDrive->turnAngle(-90_deg);
  autoDrive->driveToPoint({-2.25_ft, 1.25_ft});
  liftHook.set_value(true);
  Reverse();
  autoDrive->driveToPoint({-2.25_ft, 0_ft});
  Reverse();
}

void opcontrol() {
  while (true) {
    std::uint32_t now = pros::millis();
#ifdef DEBUG
    if (autoButton.changedToPressed()) {
      autonomous();
    }
#endif
    std::cout << liftPot.get() << "\n";
    pros::lcd::set_text(7, std::to_string(liftPot.get()));
    if (lowerHookButton.changedToPressed()) {
      lowerHookState = !lowerHookState;
    }
    if (liftHookButton.changedToPressed()) {
      liftHookState = !liftHookState;
    }
    lowerHook.set_value(lowerHookState);
    liftHook.set_value(liftHookState);
    drive->getModel()->tank(
        masterController.getAnalog(ControllerAnalog::leftY),
        masterController.getAnalog(ControllerAnalog::rightY));
    if (lift90UpButton.isPressed()) {
      lift90Motor.moveVoltage(12000);
    } else if (lift90DownButton.isPressed()) {
      lift90Motor.moveVoltage(-12000);
    } else {
      lift90Motor.moveVoltage(1000);
    }
    if (liftUpButton.changedToPressed()) {
      liftController->setTarget(2.5);
    } else if (liftDownButton.changedToPressed()) {
      liftController->setTarget(0);
    }
    pros::Task::delay_until(&now, driver_control_update);
  }
}
