package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command changePivotSetpoint(double angle) {
    return this.runOnce(
        () ->
            io.changePivotSetpoint(
                MathUtil.clamp(angle, Constants.Intake.minAngle, Constants.Intake.maxAngle)));
  }

  public Command changeRollerSpeed(double speed) {
    return this.runOnce(() -> io.changeRollerSpeed(MathUtil.clamp(speed, -1, 1)));
  }

  @AutoLogOutput(key = "Intake/AtSetpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(
        inputs.pivotMotorSetpoint, inputs.encoderAbsPosition, Units.degreesToRadians(5));
  }

  @Override
  public void periodic() {}

  public Pose3d get3DPose() {
    return new Pose3d(0.3175, 0, 0.2286, new Rotation3d(0, 0, 0));
  }
}
