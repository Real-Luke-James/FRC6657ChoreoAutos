package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Pose3d[] get3DPoses() {
    return new Pose3d[] {
      new Pose3d(0, 0, 0 * (1d / 3), new Rotation3d()),
      new Pose3d(0, 0, 0 * (2d / 3), new Rotation3d()),
      new Pose3d(0, 0, 0 * (3d / 3), new Rotation3d()),
    };
  }
}
