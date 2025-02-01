// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve.ModuleInformation;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO_Real;
import frc.robot.subsystems.elevator.ElevatorIO_Sim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO_Real;
import frc.robot.subsystems.intake.IntakeIO_Sim;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO_Real;
import frc.robot.subsystems.outtake.OuttakeIO_Sim;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);

  private Swerve drivebase;

  private Intake intake;
  private Elevator elevator;
  private Outtake outtake;

  // private ApriltagCamera[] cameras;

  private Superstructure superstructure;

  // private final AutoFactory autoFactory;

  public Robot() {

    drivebase =
        new Swerve(
            RobotBase.isReal()
                ? new ModuleIO[] {
                  new ModuleIO_Real(ModuleInformation.frontLeft),
                  new ModuleIO_Real(ModuleInformation.frontRight),
                  new ModuleIO_Real(ModuleInformation.backLeft),
                  new ModuleIO_Real(ModuleInformation.backRight)
                }
                : new ModuleIO[] {
                  new ModuleIO_Sim(ModuleInformation.frontLeft),
                  new ModuleIO_Sim(ModuleInformation.frontRight),
                  new ModuleIO_Sim(ModuleInformation.backLeft),
                  new ModuleIO_Sim(ModuleInformation.backRight)
                },
            RobotBase.isReal() ? new GyroIO_Real() : new GyroIO() {});

    intake = new Intake(RobotBase.isReal() ? new IntakeIO_Real() : new IntakeIO_Sim());
    elevator = new Elevator(RobotBase.isReal() ? new ElevatorIO_Real() : new ElevatorIO_Sim());
    outtake = new Outtake(RobotBase.isReal() ? new OuttakeIO_Real() : new OuttakeIO_Sim());

    // cameras =
    //     new ApriltagCamera[] {
    //       new ApriltagCamera(
    //           RobotBase.isReal()
    //               ? new ApriltagCameraIO_Real(VisionConstants.cameraInfo)
    //               : new ApriltagCameraIO_Sim(VisionConstants.cameraInfo),
    //           VisionConstants.cameraInfo)
    //     };

    superstructure = new Superstructure(drivebase, intake, elevator, outtake);

    // autoFactory =
    //     new AutoFactory(
    //             drivebase::getPose,
    //             drivebase::resetOdometry,
    //             drivebase::followTrajectory,
    //             true,
    //             drivebase)
    //         .bind("nothing", Commands.none());
  }

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {

    Logger.recordMetadata("Arborbotics 2025", "Arborbotics 2025");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    drivebase.setDefaultCommand(
        drivebase.drive(
            () ->
                new ChassisSpeeds(
                    MathUtil.applyDeadband(-driver.getLeftY(), 0.1) * 2,
                    MathUtil.applyDeadband(-driver.getLeftX(), 0.1) * 2,
                    MathUtil.applyDeadband(-driver.getRightX(), 0.1) * 2)));

    driver
        .a()
        .onTrue(elevator.changeSetpoint(Units.inchesToMeters(60)))
        .onFalse(elevator.changeSetpoint(Units.inchesToMeters(0)));
    ;

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    // for (var camera : cameras) {
    //   if (RobotBase.isSimulation()) {
    //     camera.updateSimPose(drivebase.getPose());
    //   }
    //   camera.updateInputs();
    //   drivebase.addVisionMeasurement(
    //       camera.getEstimatedPose(), camera.getLatestTimestamp(), camera.getLatestStdDevs());
    // }

    superstructure.update3DPose();

    // Logger.recordOutput(
    //     "ReefCam Pose",
    //     new Pose3d(drivebase.getPose()).transformBy(VisionConstants.cameraInfo.robotToCamera));

    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void autonomousInit() {
    // superstructure.testAuto(autoFactory).cmd().schedule();
  }
}
