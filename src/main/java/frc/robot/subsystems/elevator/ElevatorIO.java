package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void changeSetpoint(double setpoint) {}

  public default Pose3d[] getStagePoses() {
    return new Pose3d[3];
  }
}
