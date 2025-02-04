package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import org.littletonrobotics.junction.Logger;

public class ElevatorIO_Real implements ElevatorIO {

  TalonFX leaderMotor = new TalonFX(CAN.Elevetor_Leader.id);
  TalonFX followMotor = new TalonFX(CAN.Elevator_Follower.id);

  private double kSetpoint = Constants.Elevator.minHeight;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private boolean useRawVoltage = true;

  public ElevatorIO_Real() {

    // Configure both motors
    var leaderConfigurator = leaderMotor.getConfigurator();
    var followConfigurator = followMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio =
        Constants.Elevator.gearing; // Sets default output to rotations
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
    var motorPostition = leaderMotor.getPosition();
    var motorVelocity = leaderMotor.getVelocity();
    var motorAcceleration = leaderMotor.getAcceleration();

    var leaderMotorTemp = leaderMotor.getDeviceTemp();
    var followMotorTemp = followMotor.getDeviceTemp();
    var leaderMotorVoltage = leaderMotor.getMotorVoltage();
    var followMotorVoltage = followMotor.getMotorVoltage();
    var leaderMotorCurrent = leaderMotor.getSupplyCurrent();
    var followMotorCurrent = followMotor.getSupplyCurrent();

    leaderMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    followMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    motorPostition.setUpdateFrequency(Constants.mainLoopFrequency);
    motorVelocity.setUpdateFrequency(Constants.mainLoopFrequency);
    motorAcceleration.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);

    var closedLoopReferenceSignal = leaderMotor.getClosedLoopReference();
    closedLoopReferenceSignal.setUpdateFrequency(Constants.mainLoopFrequency);

    // reduces CAN bus usage
    leaderMotor.optimizeBusUtilization();
    followMotor.optimizeBusUtilization();

    leaderMotor.setPosition(0);

    changeSetpoint(Constants.Elevator.minHeight);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.kSetpoint = kSetpoint;
    inputs.kPosition =
        Units.inchesToMeters(
            leaderMotor.getPosition().getValueAsDouble()
                * Constants.Elevator.sprocketPD
                * Constants.Elevator.stages
                * Math.PI);
    inputs.kVelocity =
        Units.inchesToMeters(
            leaderMotor.getVelocity().getValueAsDouble()
                * Constants.Elevator.sprocketPD
                * Constants.Elevator.stages
                * Math.PI);
    inputs.kAcceleration =
        Units.inchesToMeters(
            leaderMotor.getAcceleration().getValueAsDouble()
                * Constants.Elevator.sprocketPD
                * Constants.Elevator.stages
                * Math.PI);
    inputs.leaderMotorTemp = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.followMotorTemp = followMotor.getDeviceTemp().getValueAsDouble();
    inputs.leaderMotorCurrent = leaderMotor.getSupplyCurrent().getValueAsDouble();
    inputs.followMotorCurrent = followMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leaderMotorVoltage = leaderMotor.get() * RobotController.getBatteryVoltage();
    inputs.followMotorVoltage = followMotor.getMotorVoltage().getValueAsDouble();

    if (!useRawVoltage) {
      // Stop if we are at 0
      if (kSetpoint == Constants.Elevator.minHeight
          && Math.abs(inputs.kPosition - inputs.kSetpoint)
              < Constants.Elevator.setpointTollerance) {
        leaderMotor.setControl(new VoltageOut(0));
      } else {
        leaderMotor.setControl(
            motionMagicVoltage.withPosition(
                Units.metersToInches(kSetpoint / Constants.Elevator.stages)
                    / (Constants.Elevator.sprocketPD * Math.PI)));
      }

      followMotor.setControl(new Follower(CAN.Elevetor_Leader.id, false));

      // Logging for motion magic internal variables for tuning purposes.
      Logger.recordOutput("Elevator/MotionMagicPosition", motionMagicVoltage.Position);
      Logger.recordOutput(
          "Elevator/MotionMagicSetpoint", leaderMotor.getClosedLoopReference().getValueAsDouble());
    }
  }

  @Override
  public void changeSetpoint(double setpoint) {
    useRawVoltage = false;
    kSetpoint = setpoint;
  }

  @Override
  public void setRawVoltage(double voltage) {
    useRawVoltage = true;
    leaderMotor.setControl(new VoltageOut(voltage));
    followMotor.setControl(new Follower(CAN.Elevetor_Leader.id, false));
  }
}
