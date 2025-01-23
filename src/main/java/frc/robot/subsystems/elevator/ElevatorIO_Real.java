package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ElevatorIO_Real implements ElevatorIO {

  TalonFX leaderMotor = new TalonFX(CAN.Elevetor_Leader.id);
  TalonFX followMotor = new TalonFX(CAN.Elevator_Follower.id);

  public ElevatorIO_Real() {

    // Configure the Leader motor
    var configurator = leaderMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio =
        1.0 / Constants.Elevator.gearing; // Sets default output to pivot rotations
    motorConfigs.Slot0 = Constants.Elevator.motorSlot0; // PID Constants
    motorConfigs.CurrentLimits = Constants.Elevator.currentConfigs; // Current Limits
    motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TODO verify, possibly have another one for the other motor
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotionMagic = Constants.Elevator.kMotionMagicConfig;
    configurator.apply(motorConfigs);
    leaderMotor.setNeutralMode(NeutralModeValue.Brake);//This line feels unnecesary. Aren't we already configurating this?

    

  }

  @Override
  public void changeSetpoint(double setpoint) {}
}
