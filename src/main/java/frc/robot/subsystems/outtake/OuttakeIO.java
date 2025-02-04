package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {

  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}
}
