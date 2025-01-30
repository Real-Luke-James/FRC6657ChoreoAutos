package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public boolean beamBroken = false;

    public double kSetpoint = 0.0;
    //public double kVelocity = 0.0; // should not be needed
    public double kTemp = 0.0;
    public double kVoltage = 0.0;
    public double kCurrent = 0.0;

  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
