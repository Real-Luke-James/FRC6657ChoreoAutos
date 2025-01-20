package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;

public class IntakeIO_Real implements IntakeIO {

  public IntakeIO_Real() {}

  @Override
  public void changePivotSetpoint(double setpoint) {}

  @Override
  public void changeRollerSpeed(double speed) {}

  @Override
  public Pose3d getMechanismPose() {
    return new Pose3d();
  }
}
