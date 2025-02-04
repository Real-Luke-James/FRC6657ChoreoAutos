package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {

  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changePivotSetpoint(double setpoint) {}

  public default void changeRollerSpeed(double speed) {}
}
