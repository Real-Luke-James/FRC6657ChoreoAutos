package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIO_Real implements IntakeIO {

  // Pivot Motor Controller
  TalonFX pivotMotor = new TalonFX(Constants.CAN.IntakePivot.id);

  // Roller Motor Controller
  TalonFX rollerMotor = new TalonFX(Constants.CAN.IntakeRoller.id);

  // Absolute Encoder
  private Canandmag encoder;

  // Variables to store/log the setpoints
  @AutoLogOutput(key = "Intake/Angle Setpoint")
  private double angleSetpoint = Constants.Intake.maxAngle;

  @AutoLogOutput(key = "Intake/Speed Setpoint")
  private double speedSetpoint = 0;

  private DutyCycleOut rollerSetpoint = new DutyCycleOut(0);
  private MotionMagicVoltage pivotSetpoint = new MotionMagicVoltage(Constants.Intake.maxAngle);

  public IntakeIO_Real() {
    encoder = new Canandmag(Constants.CAN.IntakeEncoder.id); 

    // Configure the pivot motor
    var pivotConfigurator = pivotMotor.getConfigurator();
    var pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.Feedback.SensorToMechanismRatio =
        1.0 / Constants.Intake.pivotGearing; // Sets default output to pivot rotations
    pivotConfigs.Slot0 = Constants.Intake.kPivotSlot0; // PID Constants
    pivotConfigs.CurrentLimits = Constants.Intake.kPivotCurrentConfigs; // Current Limits
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivotConfigs.MotionMagic = Constants.Intake.kPivotMotionMagicConfig;
    pivotConfigurator.apply(pivotConfigs);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    // Pivot Status Signals
    var pivotPositionSignal = pivotMotor.getPosition();
    var pivotVelocitySignal = pivotMotor.getVelocity();
    var pivotAccelerationSignal = pivotMotor.getAcceleration();
    var pivotTempSignal = pivotMotor.getDeviceTemp();
    var pivotVoltageSignal = pivotMotor.getMotorVoltage();
    var pivotCurrentSignal = pivotMotor.getSupplyCurrent();
    var pivotClosedLoopReferenceSignal = pivotMotor.getClosedLoopReference();

    pivotPositionSignal.setUpdateFrequency(Constants.mainLoopFrequency);
    pivotVelocitySignal.setUpdateFrequency(Constants.mainLoopFrequency);
    pivotAccelerationSignal.setUpdateFrequency(Constants.mainLoopFrequency);
    pivotTempSignal.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    pivotVoltageSignal.setUpdateFrequency(Constants.mainLoopFrequency);
    pivotCurrentSignal.setUpdateFrequency(Constants.mainLoopFrequency);
    pivotClosedLoopReferenceSignal.setUpdateFrequency(Constants.mainLoopFrequency);

    pivotMotor.optimizeBusUtilization(); // Reduces CAN bus usage

    // Set the default pivot location
    pivotMotor.setPosition(Units.degreesToRotations(Constants.Intake.maxAngle));

    // Configure the leading roller motor
    var rollerConfigurator = rollerMotor.getConfigurator();
    var rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.Feedback.SensorToMechanismRatio =
        1.0 / Constants.Intake.rollerGearing; // Sets default output to roller rotations
    rollerConfigs.CurrentLimits = Constants.Intake.kRollersCurrentConfigs; // Current Limits
    rollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: verify
    rollerConfigurator.apply(rollerConfigs);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);

    // Roller Status Signals
    var rollerVelocitySignal = rollerMotor.getVelocity();
    var rollerTempSignal = rollerMotor.getDeviceTemp();
    var rollerVoltageSignal = rollerMotor.getMotorVoltage();
    var rollerCurrentSignal = rollerMotor.getSupplyCurrent();

    rollerVelocitySignal.setUpdateFrequency(Constants.mainLoopFrequency);
    rollerTempSignal.setUpdateFrequency(Constants.mainLoopFrequency / 4);
    rollerVoltageSignal.setUpdateFrequency(Constants.mainLoopFrequency);
    rollerCurrentSignal.setUpdateFrequency(Constants.mainLoopFrequency);

    rollerMotor.optimizeBusUtilization(); // Reduces CAN bus usage


    // Feed the PID with default values
    changePivotSetpoint(Constants.Intake.maxAngle);
    changeRollerSpeed(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Update the pivot inputs
    inputs.pivotMotorPosition =
        Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()); // Degrees
    inputs.pivotMotorVelocity =
        Units.rotationsToDegrees(pivotMotor.getVelocity().getValueAsDouble()); // Degrees per second
    inputs.pivotMotorAcceleration =
        Units.rotationsToDegrees(pivotMotor.getAcceleration().getValueAsDouble());
    inputs.pivotMotorTemp = pivotMotor.getDeviceTemp().getValueAsDouble(); // Celcius
    inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.pivotMotorCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble(); // Amps
    inputs.atSetpoint = MathUtil.isNear(angleSetpoint, inputs.pivotMotorPosition, Constants.Intake.pivotAtSetpointTolerance);
    inputs.pivotMotorSetpoint = angleSetpoint;

    // Update the roller inputs
    inputs.rollerMotorVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60; // RPM
    inputs.rollerMotorTemp = rollerMotor.getDeviceTemp().getValueAsDouble(); // Celcius
    inputs.rollerMotorVoltage = rollerMotor.getMotorVoltage().getValueAsDouble(); // Volts
    inputs.rollerMotorCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble(); // Amps
    inputs.rollerMotorSetpoint = speedSetpoint;

    rollerMotor.setControl(rollerSetpoint.withOutput(speedSetpoint));
    pivotMotor.setControl(
        pivotSetpoint.withPosition(
            Units.degreesToRotations(angleSetpoint))); // Degrees to Native Rotations

    /*
    Measurement measurement;
    try {
      measurement = sensor.getMeasurement();
    }catch(Exception e){
      measurement = null;
    }
    Logger.recordOutput("Intake/TOFStatus", measurement.status);

    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.tofDistance = Units.metersToInches(sensor.getMeasurement().distance_mm * 0.001);
      inputs.tofUnplugged = false;
    } else {
      inputs.tofDistance = 100;
      inputs.tofUnplugged = true;
    }
    */

    inputs.encoderAbsPosition = encoder.getAbsPosition() * 2 * Math.PI;
    inputs.encoderRelPosition = encoder.getPosition() * 2 * Math.PI;
    inputs.encoderVelocity = encoder.getVelocity() * 2 * Math.PI;
  }

  @Override
  public void changePivotSetpoint(double setpoint) {
    angleSetpoint = setpoint;
  }

  @Override
  public void changeRollerSpeed(double speed) {
    speedSetpoint = speed;
  }
}
