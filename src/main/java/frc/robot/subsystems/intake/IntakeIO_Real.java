package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeIO_Real implements IntakeIO {

  // Pivot Motor Controller
  private SparkMax pivotMotor;

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

    pivotMotor = new SparkMax(Constants.CAN.IntakePivot.id, MotorType.kBrushless);
    encoder = new Canandmag(Constants.CAN.IntakeEncoder.id);

    // Feed the PID with default values
    changePivotSetpoint(Constants.Intake.maxAngle);
    changeRollerSpeed(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

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
