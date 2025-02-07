package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefSlot;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  // Subsystems
  Swerve drivebase;
  Elevator elevator;
  Outtake outtake;
  Intake intake;
  Climber climber;

  @AutoLogOutput(key = "States/Selected Reef")
  private String selectedReef = "Left"; // Selected Reef Pole

  @AutoLogOutput(key = "States/Elevator Level")
  private int elevatorLevel = 2; // Selected Reef Level

  private double[] elevatorSetpoints = {
    0, 0, Units.inchesToMeters(14), Units.inchesToMeters(30), Units.inchesToMeters(55)
  }; // Array for easily grabbing setpoint heights.

  // Constructor
  public Superstructure(Swerve drivebase, Elevator elevator, Outtake outtake, Intake intake, Climber climber) {
    this.drivebase = drivebase;
    this.elevator = elevator;
    this.outtake = outtake;
    this.intake = intake;
    this.climber = climber;
  }

  /*
   * Grab and Log the 3D positions of all robot mechanisms for 3D visualization.
   */
  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = intake.get3DPose();
    mechanismPoses[1] = elevator.get3DPoses()[0];
    mechanismPoses[2] = elevator.get3DPoses()[1];
    mechanismPoses[3] = elevator.get3DPoses()[2];

    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  // Gets the closest reef sector to the robot.
  public Pose2d getNearestReef() {

    // Grab the alliance color
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Create an array of reef slots based on the alliance color
    ReefSlot[] reefSlots = new ReefSlot[6];

    // Set the reef slots based on the alliance color
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

    // Create a list of reef middle poses the robot scores offcenter but this is used instead just
    // to select the sector.
    List<Pose2d> reefMiddles = new ArrayList<>();
    for (ReefSlot reefSlot : reefSlots) {
      reefMiddles.add(reefSlot.middle);
    }

    Pose2d currentPos = drivebase.getPose(); // Get the current robot pose
    Pose2d nearestReefMiddle = currentPos.nearest(reefMiddles); // Get the nearest reef middle pose
    ReefSlot nearestReefSlot =
        reefSlots[
            reefMiddles.indexOf(
                nearestReefMiddle)]; // Get the reef slot of the nearest reef middle pose

    // Return the reef slot based on the selected reef
    if (selectedReef == "Left") {
      return nearestReefSlot.left;
    } else if (selectedReef == "Right") {
      return nearestReefSlot.right;
    }

    // If the selected reef is invalid return the robot's current pose.
    Logger.recordOutput("Errors", "Invalid Reef Selected '" + selectedReef + "'");
    return currentPos;
  }

  // Simple command to change the selected reef level.
  public Command selectElevatorHeight(int height) {
    return Commands.runOnce(() -> elevatorLevel = height);
  }

  // Simple command to change the selected reef pole.
  public Command selectReef(String reef) {
    return Commands.runOnce(() -> selectedReef = reef);
  }

  public Command raiseElevator() {
    return elevator.changeSetpoint(() -> elevatorSetpoints[elevatorLevel]);
  }

  // Command to algin to the reef and get ready to score a coral.
  // This command aligns the drivebase to the nearest reef and raises the elevator to the selected
  // reef level.
  // Command will end when the drivebase is aligned and the elevator is at the selected reef level.
  public Command ReefAlgin(String side, int level) {
    return Commands.sequence(
        selectReef(side),
        selectElevatorHeight(level),
        Commands.parallel(
            drivebase.goToPose(() -> getNearestReef()),
            elevator
                .changeSetpoint((() -> elevatorSetpoints[elevatorLevel]))
                .andThen(Commands.waitUntil(elevator::atSetpoint))));
  }

  // Command to score a coral.
  // Command will lower the elevator when finished.
  // Ends when the elevator is at the bottom.
  public Command ScoreCoral() {
    return Commands.sequence(
        Commands.print("Score Coral"),
        Commands.waitSeconds(0.25),
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

    S_P1.atTime("Score")
        .onTrue(
            ReefAlgin("Left", 4).andThen(ScoreCoral()).andThen(P1_I1.cmd().andThen(I1_P2.cmd())));
    I1_P2
        .atTime("Score")
        .onTrue(
            ReefAlgin("Left", 4).andThen(ScoreCoral()).andThen(P2_I2.cmd().andThen(I2_P3.cmd())));
    I2_P3.atTime("Score").onTrue(ReefAlgin("Right", 4).andThen(ScoreCoral()));

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
