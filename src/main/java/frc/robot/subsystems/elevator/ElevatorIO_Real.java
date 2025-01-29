package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ElevatorIO_Real implements ElevatorIO {

  TalonFX leaderMotor = new TalonFX(CAN.Elevetor_Leader.id);
  TalonFX followMotor = new TalonFX(CAN.Elevator_Follower.id);

  private double kSetpoint = Constants.Elevator.minHeight;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public ElevatorIO_Real() {

    // Configure both motors
    var leaderConfigurator = leaderMotor.getConfigurator();
    var followConfigurator = followMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio =
        1.0 / Constants.Elevator.gearing; // Sets default output to pivot rotations
    motorConfigs.Slot0 = Constants.Elevator.motorSlot0; // PID Constants
    motorConfigs.CurrentLimits = Constants.Elevator.currentConfigs; // Current Limits
    // motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TODO verify
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotionMagic = Constants.Elevator.kMotionMagicConfig;
    leaderConfigurator.apply(motorConfigs); // Configure leader motor
    followConfigurator.apply(motorConfigs); // Configure follow motor to the same thing
    followMotor.setControl(
        new Follower(
            CAN.Elevetor_Leader.id,
            false)); // Only difference with the follow motor configuration is this line

    // grab important numbers for logging
    var motorPostition =
        leaderMotor.getPosition(); // I don't think we need to track this for both motors
    var motorVelocity = leaderMotor.getVelocity();
    // acceleration?

    var leaderMotorTemp = leaderMotor.getDeviceTemp();
    var followMotorTemp = followMotor.getDeviceTemp();
    var leaderMotorVoltage = leaderMotor.getMotorVoltage();
    var followMotorVoltage = followMotor.getMotorVoltage();
    var leaderMotorCurrent =
        leaderMotor
            .getSupplyCurrent(); // Supply current, not stator current right? for auto logging
    var followMotorCurrent = followMotor.getSupplyCurrent();

    leaderMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    followMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    motorPostition.setUpdateFrequency(Constants.mainLoopFrequency);
    motorVelocity.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);

    var closedLoopReferenceSignal = leaderMotor.getClosedLoopReference();
    closedLoopReferenceSignal.setUpdateFrequency(Constants.mainLoopFrequency);

    // redices CAN bus usage
    leaderMotor.optimizeBusUtilization();
    followMotor.optimizeBusUtilization();

    changeSetpoint(Constants.Elevator.minHeight);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.kSetpoint = kSetpoint;
    inputs.kPosition =
        Units.inchesToMeters(
            leaderMotor.getPosition().getValueAsDouble()
                * 1.7567); // 1.7567 is the inch length a rotation yeilds
    inputs.kVelocity =
        Units.inchesToMeters(leaderMotor.getVelocity().getValueAsDouble() * 1.7567); // also here

    inputs.leaderMotorTemp = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.followMotorTemp = followMotor.getDeviceTemp().getValueAsDouble();
    inputs.leaderMotorCurrent = leaderMotor.getSupplyCurrent().getValueAsDouble();
    inputs.followMotorCurrent = followMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leaderMotorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.followMotorVoltage = followMotor.getMotorVoltage().getValueAsDouble();

    leaderMotor.setControl(motionMagicVoltage.withPosition(Units.degreesToRotations(Units.metersToInches(kSetpoint / 1.7567)))); // fix the unit conversion at some point, I am sure I made a mistake
  }

  @Override
  public void changeSetpoint(double setpoint) {
    kSetpoint = MathUtil.clamp(Units.inchesToMeters(setpoint), Constants.Elevator.minHeight, Constants.Elevator.maxHeight);
  }
}
