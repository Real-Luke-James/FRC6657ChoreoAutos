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
    

    

  }

  @Override
  public void changeSetpoint(double setpoint) {}
}
