package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberIO_Real implements ClimberIO {

  // motor controller
  private final SparkMax climberMotor;

  RelativeEncoder climberEncoder;

  // store/log setpoints
  @AutoLogOutput(key = "Climber/Angle Setpoint")
  private double angleSetpoint = Constants.Climber.maxAngle;

  @AutoLogOutput(key = "Climber/Speed Setpoint")
  private double speedSetpoint = 0;

  public ClimberIO_Real() {
    // motor configs
    climberMotor = new SparkMax(Constants.CAN.Climber.id, MotorType.kBrushless);
    climberMotor.setCANTimeout(250);
    SparkMaxConfig mConfig = new SparkMaxConfig();
    mConfig.inverted(false);
    mConfig.voltageCompensation(12);
    mConfig.smartCurrentLimit(Constants.Climber.currentLimit);
    mConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(mConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // status signals
    var positionSignal = climberEncoder.getPosition();
    var velocitySignal = climberEncoder.getVelocity();
    var tempSignal = climberMotor.getMotorTemperature();
    var voltageSignal = climberMotor.getBusVoltage();
    var currentSignal = climberMotor.getOutputCurrent();
  }
}
