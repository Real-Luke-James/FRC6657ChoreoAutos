package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double pivotMotorPosition = 0.0; // Degrees
    public double pivotMotorVelocity = 0.0; // Degrees per second
    public double pivotMotorTemp = 0.0; // Celcius
    public double pivotMotorVoltage = 0.0; // Volts
    public double pivotMotorCurrent = 0.0; // Amps
    public double pivotMotorSetpoint = 0.0;

    public double rollerMotorVelocity = 0.0; // RPM
    public double rollerMotorTemp = 0.0; // Celcius
    public double rollerMotorVoltage = 0.0; // Volts
    public double rollerMotorCurrent = 0.0; // Amps
    public double rollerMotorSetpoint = 0.0; // -1 to 1

    public double encoderAbsPosition = 0.0; // Rad
    public double encoderRelPosition = 0.0; // Rad
    public double encoderVelocity = 0.0; // Rad/s

  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changePivotSetpoint(double setpoint) {}

  public default void changeRollerSpeed(double speed) {}
}
