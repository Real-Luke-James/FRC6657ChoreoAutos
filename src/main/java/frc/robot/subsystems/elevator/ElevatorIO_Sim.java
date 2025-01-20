package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;

public class ElevatorIO_Sim implements ElevatorIO {

  public ElevatorIO_Sim() {}

  @Override
  public void changeSetpoint(double setpoint) {}

  @Override
  public Pose3d[] getStagePoses() {
    return new Pose3d[3];
  }
}
