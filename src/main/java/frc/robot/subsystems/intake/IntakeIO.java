package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void changePivotSetpoint(double setpoint) {}

  public default void changeRollerSpeed(double speed) {}

  public default Pose3d getMechanismPose() {
    return new Pose3d();
  }
}
