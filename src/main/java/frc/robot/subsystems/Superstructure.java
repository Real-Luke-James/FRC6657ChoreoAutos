package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  Swerve drivebase;
  Intake intake;
  Elevator elevator;
  Outtake outtake;

  public Superstructure(Swerve drivebase, Intake intake, Elevator elevator, Outtake outtake) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.elevator = elevator;
    this.outtake = outtake;
  }

  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = intake.get3DPose();
    mechanismPoses[1] = elevator.get3DPoses()[0];
    mechanismPoses[2] = elevator.get3DPoses()[1];
    mechanismPoses[3] = elevator.get3DPoses()[2];

    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  // Simple Test Auto that just runs a path.
  public AutoRoutine testAuto(AutoFactory factory) {

    final AutoRoutine routine = factory.newRoutine("Test");

    final AutoTrajectory testPath = routine.trajectory("Test");

    routine
        .active()
        .onTrue(
            drivebase
                .resetOdometry(
                    testPath
                        .getInitialPose()
                        .orElseGet(
                            () -> {
                              routine.kill();
                              return new Pose2d();
                            }))
                .andThen(testPath.cmd()));

    return routine;
  }
}
