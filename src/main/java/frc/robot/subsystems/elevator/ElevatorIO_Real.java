package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;

public class ElevatorIO_Real implements ElevatorIO {

  public ElevatorIO_Real() {}

  @Override
  public void changeSetpoint(double setpoint) {}

  @Override
  public Pose3d[] getStagePoses() {
    return new Pose3d[3];
  }
}
