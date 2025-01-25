package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ElevatorIO_Real implements ElevatorIO {

  TalonFX leaderMotor = new TalonFX(CAN.Elevetor_Leader.id);
  TalonFX followMotor = new TalonFX(CAN.Elevator_Follower.id);

  private double kSetpoint = Constants.Elevator.minHeight;

  public ElevatorIO_Real() {

    // Configure both motors
    var leaderConfigurator = leaderMotor.getConfigurator();
    var followConfigurator = followMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio =
        1.0 / Constants.Elevator.gearing; // Sets default output to pivot rotations
    motorConfigs.Slot0 = Constants.Elevator.motorSlot0; // PID Constants
    motorConfigs.CurrentLimits = Constants.Elevator.currentConfigs; // Current Limits
    //motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TODO verify
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotionMagic = Constants.Elevator.kMotionMagicConfig;
    leaderConfigurator.apply(motorConfigs); // Configure leader motor
    followConfigurator.apply(motorConfigs); // Configure follow motor to the same thing
    followMotor.setControl(new Follower(CAN.Elevetor_Leader.id, false)); // Only difference with the follow motor configuration is this line
    
    // grab important numbers for logging
    var motorPostition = leaderMotor.getPosition(); // I don't think we need to track this for both motors
    var motorVelocity = leaderMotor.getVelocity();
    // acceleration?

    var leaderMotorTemp = leaderMotor.getDeviceTemp();
    var followMotorTemp = followMotor.getDeviceTemp();
    var leaderMotorVoltage = leaderMotor.getMotorVoltage();
    var followMotorVoltage = followMotor.getMotorVoltage();
    var leaderMotorCurrent = leaderMotor.getSupplyCurrent(); // Supply current, not stator current right? for auto logging
    var followMotorCurrent = followMotor.getSupplyCurrent();

    leaderMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency/4);
    followMotorTemp.setUpdateFrequency(Constants.mainLoopFrequency/4);
    motorPostition.setUpdateFrequency(Constants.mainLoopFrequency);
    motorVelocity.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    leaderMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);
    followMotorCurrent.setUpdateFrequency(Constants.mainLoopFrequency);

    var closedLoopReferenceSignal = leaderMotor.getClosedLoopReference(); // what does this do?
    closedLoopReferenceSignal.setUpdateFrequency(Constants.mainLoopFrequency);

    // redices CAN bus usage
    leaderMotor.optimizeBusUtilization();
    followMotor.optimizeBusUtilization();

    changeSetpoint(Constants.Elevator.minHeight);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){
    inputs.kSetpoint = kSetpoint;
    inputs.kPosition = leaderMotor.getPosition().getValueAsDouble(); //TODO need to do unit conversions
    inputs.kVelocity = leaderMotor.getVelocity().getValueAsDouble(); // also here
    
    inputs.leaderMotorTemp = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.followMotorTemp = followMotor.getDeviceTemp().getValueAsDouble();
    inputs.leaderMotorCurrent = leaderMotor.getSupplyCurrent().getValueAsDouble();
    inputs.followMotorCurrent = followMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leaderMotorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.followMotorVoltage = followMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void changeSetpoint(double setpoint) {
    kSetpoint = setpoint;
  }
}
