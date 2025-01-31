package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command changePivotSetpoint(double angleDegrees) {
    return this.runOnce(
        () ->
            io.changePivotSetpoint(
                MathUtil.clamp(
                    angleDegrees, Constants.Intake.minAngle, Constants.Intake.maxAngle)));
  }

  public Command changeRollerSpeed(double speed) {
    return this.runOnce(() -> io.changeRollerSpeed(MathUtil.clamp(speed, -1, 1)));
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  public boolean extended() {
    return inputs.pivotMotorPosition < 0 && inputs.atSetpoint;
  }

  public boolean pivotSetpointIsMin() {
    return inputs.pivotMotorSetpoint == Constants.Intake.minAngle;
  }

  public boolean pivotSetpointIsMax() {
    return inputs.pivotMotorSetpoint == Constants.Intake.maxAngle;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Pose3d get3DPose() {
    return new Pose3d(0.3175, 0, 0.2286, new Rotation3d(0, Units.degreesToRadians(0), 0));
  }
}
