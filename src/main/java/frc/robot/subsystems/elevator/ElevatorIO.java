package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double setpoint = 0.0;
    public double position = 0.0;

    public double leaderMotorTemp = 0.0; // Celcius
    public double leaderMotorVoltage = 0.0; // Volts
    public double leaderMotorCurrent = 0.0; // Amps
    public double leaderMotorVelocity = 0.0; // RPM

    public double followMotorTemp = 0.0; // Celcius
    public double followMotorVoltage = 0.0; // Volts
    public double followMotorCurrent = 0.0; // Amps
    public double followMotorVelocity = 0.0; // RPM

    
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
