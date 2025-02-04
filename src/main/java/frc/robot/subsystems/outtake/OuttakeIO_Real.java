package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class OuttakeIO_Real implements OuttakeIO {

  // Roller Motor Controller
  TalonFX rollerMotor = new TalonFX(Constants.CAN.OuttakeMotor.id);

  DigitalInput beambreak = new DigitalInput(7);

  private double rollerSetpoint = 0;

  public OuttakeIO_Real() {

    var motorConfigurator = rollerMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Slot0 = Constants.Outtake.motorSlot0; // PID Constants
    motorConfigs.CurrentLimits = Constants.Outtake.currentConfigs; // Current Limits
    // motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfigurator.apply(motorConfigs); // Configure leader motor

    var kTemp = rollerMotor.getDeviceTemp();
    var kVoltage = rollerMotor.getMotorVoltage();
    var kCurrent = rollerMotor.getSupplyCurrent();

    kTemp.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    kVoltage.setUpdateFrequency(Constants.mainLoopFrequency);
    kCurrent.setUpdateFrequency(Constants.mainLoopFrequency);

    rollerMotor.optimizeBusUtilization();

    changeSetpoint(0);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.beamBroken = !beambreak.get();

    inputs.kTemp = rollerMotor.getDeviceTemp().getValueAsDouble();
    inputs.kCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.kVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();

    inputs.kSetpoint = rollerSetpoint;

    rollerMotor.setControl(new VoltageOut(rollerSetpoint));
  }

  @Override
  public void changeSetpoint(double setpoint) {
    rollerSetpoint = setpoint;
  }
}
