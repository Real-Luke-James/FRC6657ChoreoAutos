package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class IntakeIO_Real implements IntakeIO {

  // Pivot Motor Controller
  private SparkMax pivotMotor;

  // Roller Motor Controller
  private TalonFX rollerMotor;

  // Absolute Encoder
  private Canandmag encoder;

  // Variables to store/log the setpoints
  private double angleSetpoint = Constants.Intake.minAngle;

  private double speedSetpoint = 0;

  private ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          0, 0, 0, new Constraints(Units.degreesToRadians(50), Units.degreesToRadians(50)));

  public IntakeIO_Real() {

    pivotMotor = new SparkMax(Constants.CAN.IntakePivot.id, MotorType.kBrushless);
    rollerMotor = new TalonFX(Constants.CAN.IntakeRoller.id);
    encoder = new Canandmag(Constants.CAN.IntakeEncoder.id);

    pivotMotor.configure(
        new SparkMaxConfig()
            .apply(new EncoderConfig().positionConversionFactor(1d / Constants.Intake.pivotGearing))
            .smartCurrentLimit(40),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Configure the leading roller motor
    var rollerConfigurator = rollerMotor.getConfigurator();
    var rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.Feedback.SensorToMechanismRatio = 1.0 / Constants.Intake.rollerGearing;
    rollerConfigs.CurrentLimits = Constants.Intake.kRollersCurrentConfigs;
    rollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

    pivotMotor.setCANTimeout(250);
    var pivotConfigurator = new SparkMaxConfig();

    pivotConfigurator.voltageCompensation(12);
    pivotConfigurator.smartCurrentLimit(Constants.Intake.kPivotSupplyLimit);
    pivotConfigurator.idleMode(IdleMode.kBrake);
    pivotConfigurator.inverted(false);
    pivotMotor.configure(
        pivotConfigurator, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerMotor.optimizeBusUtilization(); // Reduces CAN bus usage

    // Feed the PID with default values
    changePivotSetpoint(Constants.Intake.minAngle);
    changeRollerSpeed(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    double pidOutput =
        pivotPID.calculate(Units.rotationsToRadians(inputs.encoderAbsPosition), angleSetpoint);
    pivotMotor.setVoltage(pidOutput);

    rollerMotor.set(speedSetpoint);

    inputs.encoderAbsPosition = Units.rotationsToRadians(encoder.getAbsPosition());
    inputs.encoderRelPosition = Units.rotationsToRadians(encoder.getPosition());
    inputs.encoderVelocity = Units.rotationsToRadians(encoder.getVelocity());

    inputs.pivotMotorPosition = Units.rotationsToRadians(pivotMotor.getEncoder().getPosition());
    inputs.pivotMotorVelocity = Units.rotationsToRadians(pivotMotor.getEncoder().getVelocity());
    inputs.pivotMotorCurrent = pivotMotor.getOutputCurrent();
    inputs.pivotMotorVoltage = pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.pivotMotorTemp = pivotMotor.getMotorTemperature();
    inputs.pivotMotorSetpoint = angleSetpoint;

    inputs.rollerMotorVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60; // RPM
    inputs.rollerMotorTemp = rollerMotor.getDeviceTemp().getValueAsDouble();
    inputs.rollerMotorVoltage = rollerMotor.get() * RobotController.getBatteryVoltage();
    inputs.rollerMotorCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rollerMotorSetpoint = speedSetpoint;
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
