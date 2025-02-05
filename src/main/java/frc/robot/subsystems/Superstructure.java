package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefSlot;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  Swerve drivebase;
  Intake intake;
  Elevator elevator;
  Outtake outtake;

  private String selectedReef = "Left";

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

  public Pose2d getNearestReef() {

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    ReefSlot[] reefSlots = new ReefSlot[6];

    if (alliance == Alliance.Red) {
      reefSlots =
          new ReefSlot[] {
            Constants.FieldConstants.ReefPoses.Reef_1.red,
            Constants.FieldConstants.ReefPoses.Reef_2.red,
            Constants.FieldConstants.ReefPoses.Reef_3.red,
            Constants.FieldConstants.ReefPoses.Reef_4.red,
            Constants.FieldConstants.ReefPoses.Reef_5.red,
            Constants.FieldConstants.ReefPoses.Reef_6.red
          };
    } else {
      reefSlots =
          new ReefSlot[] {
            Constants.FieldConstants.ReefPoses.Reef_1.blue,
            Constants.FieldConstants.ReefPoses.Reef_2.blue,
            Constants.FieldConstants.ReefPoses.Reef_3.blue,
            Constants.FieldConstants.ReefPoses.Reef_4.blue,
            Constants.FieldConstants.ReefPoses.Reef_5.blue,
            Constants.FieldConstants.ReefPoses.Reef_6.blue
          };
    }

    List<Pose2d> reefMiddles = new ArrayList<>();
    for (ReefSlot reefSlot : reefSlots) {
      reefMiddles.add(reefSlot.middle);
    }

    Pose2d currentPos = drivebase.getPose();
    Pose2d nearestReefMiddle = currentPos.nearest(reefMiddles);
    ReefSlot nearestReefSlot = reefSlots[reefMiddles.indexOf(nearestReefMiddle)];

    if (selectedReef == "Left") {
      return nearestReefSlot.left;
    } else if (selectedReef == "Right") {
      return nearestReefSlot.right;
    }

    return new Pose2d();
  }

  public Command selectReef(String reef) {
    return Commands.runOnce(() -> selectedReef = reef);
  }

  public Command ReefAlginLeft() {
    return Commands.sequence(selectReef("Left"), drivebase.goToPose(() -> getNearestReef()));
  }

  public Command ReefAlginRight() {
    return Commands.sequence(selectReef("Right"), drivebase.goToPose(() -> getNearestReef()));
  }

  public Command ElevatorL4() {
    return Commands.sequence(elevator.changeSetpoint(58), Commands.waitUntil(elevator::atSetpoint));
  }

  public Command ScoreCoral() {
    return Commands.sequence(
        Commands.print("Score Coral"),
        Commands.waitSeconds(0.25),
        elevator.changeSetpoint(0),
        Commands.waitUntil(elevator::atSetpoint));
  }

  public Command ScoreL4() {
    return Commands.sequence(
        elevator.changeSetpoint(58),
        Commands.waitUntil(elevator::atSetpoint),
        // Outtake Here
        Commands.waitSeconds(0.5),
        elevator.changeSetpoint(0),
        Commands.waitUntil(elevator::atSetpoint));
  }

  private Command scoreL4Coral(String reef) {
    return Commands.sequence(
        selectReef(reef),
        drivebase.goToPose(() -> getNearestReef()),
        elevator.changeSetpoint(58),
        Commands.waitUntil(elevator::atSetpoint),
        // Outtake Here
        Commands.waitSeconds(0.5),
        elevator.changeSetpoint(0),
        Commands.waitUntil(elevator::atSetpoint));
  }

  // Simple Test Auto that just runs a path.
  public AutoRoutine testAuto(AutoFactory factory) {

    final AutoRoutine routine = factory.newRoutine("Test");

    final AutoTrajectory S_P1 = routine.trajectory("Test 3 Piece", 0);
    final AutoTrajectory P1_I1 = routine.trajectory("Test 3 Piece", 1);
    final AutoTrajectory I1_P2 = routine.trajectory("Test 3 Piece", 2);
    final AutoTrajectory P2_I2 = routine.trajectory("Test 3 Piece", 3);
    final AutoTrajectory I2_P3 = routine.trajectory("Test 3 Piece", 4);

    S_P1.atTime("Score").onTrue(scoreL4Coral("Left").andThen(P1_I1.cmd().andThen(I1_P2.cmd())));
    I1_P2.atTime("Score").onTrue(scoreL4Coral("Left").andThen(P2_I2.cmd().andThen(I2_P3.cmd())));
    I2_P3.atTime("Score").onTrue(scoreL4Coral("Right"));

    routine
        .active()
        .onTrue(
            drivebase
                .resetOdometry(
                    S_P1.getInitialPose()
                        .orElseGet(
                            () -> {
                              routine.kill();
                              return new Pose2d();
                            }))
                .andThen(S_P1.cmd()));

    return routine;
  }
}
