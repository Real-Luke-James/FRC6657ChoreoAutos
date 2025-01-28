package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class OuttakeIO_Real implements OuttakeIO {

  // Roller Motor Controller
  TalonFX rollerMotor = new TalonFX(Constants.CAN.OuttakeMotor.id);

  DigitalInput beambreak = new DigitalInput(7); // Placeholder port number
  
  private double rpmSetpoint = 0;

  public OuttakeIO_Real() {

    var motorConfigurator = rollerMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio =
        1.0 / Constants.Elevator.gearing; // Sets default output to pivot rotations
    motorConfigs.Slot0 = Constants.Outtake.motorSlot0; // PID Constants
    motorConfigs.CurrentLimits = Constants.Outtake.currentConfigs; // Current Limits
    // motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TODO verify
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // I think coast makes more sense
    motorConfigs.MotionMagic = Constants.Outtake.kMotionMagicConfig;
    motorConfigurator.apply(motorConfigs); // Configure leader motor

    
  }

  @Override
  public void changeSetpoint(double setpoint) {}
}
