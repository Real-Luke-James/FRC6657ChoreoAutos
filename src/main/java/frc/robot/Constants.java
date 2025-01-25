package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static double mainLoopFrequency = 50d; // Hz

  public static enum CAN { // TODO: Verify
    Swerve_FL_D(1),
    Swerve_FR_D(2),
    Swerve_BL_D(3),
    Swerve_BR_D(4),
    Swerve_FL_T(5),
    Swerve_FR_T(6),
    Swerve_BL_T(7),
    Swerve_BR_T(8),
    Swerve_FL_E(9),
    Swerve_FR_E(10),
    Swerve_BL_E(11),
    Swerve_BR_E(12),
    Gyro(13),
    Elevetor_Leader(15),
    Elevator_Follower(16);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }

  public static class AutoConstants {

    // Choreo
    public static final PIDController kXController_Choreo =
        new PIDController(8, 0, 0); // TODO: Tune
    public static final PIDController kYController_Choreo =
        new PIDController(8, 0, 0); // TODO: Tune
    public static final PIDController kThetaController_Choreo =
        new PIDController(6, 0, 0); // TODO: Tune

    // Repulsor
    public static final PIDController kXController_Repulsor =
        new PIDController(100, 0, 0); // TODO: Tune
    public static final PIDController kYController_Repulsor =
        new PIDController(100, 0, 0); // TODO: Tune
    public static final PIDController kThetaController_Repulsor =
        new PIDController(100, 0, 0); // TODO: Tune

    // Position PID
    public static final PIDController kXController_Position =
        new PIDController(5, 0, 0); // TODO: Tune
    public static final PIDController kYController_Position =
        new PIDController(5, 0, 0); // TODO: Tune
    public static final PIDController kThetaController_Position =
        new PIDController(5, 0, 0); // TODO: Tune
  }

  public static class VisionConstants {

    public static class CameraInfo {

      public String cameraName;
      public Transform3d robotToCamera;
      public Rotation2d diagFOV;
      public int[] cameraRes;

      public CameraInfo(
          String cameraName, Transform3d robotToCamera, Rotation2d diagFOV, int[] cameraRes) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        this.diagFOV = diagFOV;
        this.cameraRes = cameraRes;
      }
    }

    public static CameraInfo cameraInfo =
        new CameraInfo(
            "Camera",
            new Transform3d(
                new Translation3d(-0.302561, -0.294302, 0.24),
                new Rotation3d(
                    0, Units.degreesToRadians(-10), Math.PI - Units.degreesToRadians(50))),
            Rotation2d.fromDegrees(95),
            new int[] {1280, 800});

    public static final Matrix<N3, N1> singleTagStdDev =
        VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
    public static final Matrix<N3, N1> multiTagStdDev = VecBuilder.fill(0.2, 0.2, Double.MAX_VALUE);
  }

  public static class Motors {
    public static double FalconRPS =
        Units.radiansPerSecondToRotationsPerMinute(DCMotor.getFalcon500(1).freeSpeedRadPerSec) / 60;
  }

  public static class Swerve {

    public enum DriveGearing {
      L1(19d / 25d),
      L2(17d / 27d),
      L3(16d / 28d);

      public double reduction;

      DriveGearing(double reduction) {
        this.reduction = reduction * (50d / 14d) * (45d / 15d);
      }
    }

    public static double WheelDiameter = Units.inchesToMeters(4);
    public static double TrackWidth = Units.inchesToMeters(29 - 5.25);
    public static double TrackLength = Units.inchesToMeters(29 - 5.25);

    public static Translation2d[] ModulePositions =
        new Translation2d[] {
          new Translation2d(TrackWidth / 2, TrackLength / 2),
          new Translation2d(TrackWidth / 2, -TrackLength / 2),
          new Translation2d(-TrackWidth / 2, TrackLength / 2),
          new Translation2d(-TrackWidth / 2, -TrackLength / 2)
        };

    public static double TurnGearing = 150d / 7d;

    public static class ModuleInformation {

      public String name;
      public int driveID;
      public int turnID;
      public int encoderID;

      public ModuleInformation(String name, int driveID, int turnID, int encoderID) {
        this.name = name;
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
      }

      public static ModuleInformation frontLeft =
          new ModuleInformation(
              "Front Left ", CAN.Swerve_FL_D.id, CAN.Swerve_FL_T.id, CAN.Swerve_FL_E.id);
      public static ModuleInformation frontRight =
          new ModuleInformation(
              "Front Right ", CAN.Swerve_FR_D.id, CAN.Swerve_FR_T.id, CAN.Swerve_FR_E.id);
      public static ModuleInformation backLeft =
          new ModuleInformation(
              "Back Left ", CAN.Swerve_BL_D.id, CAN.Swerve_BL_T.id, CAN.Swerve_BL_E.id);
      public static ModuleInformation backRight =
          new ModuleInformation(
              "Back Right ", CAN.Swerve_BR_D.id, CAN.Swerve_BR_T.id, CAN.Swerve_BR_E.id);
    }
  }

  public static class Intake {
    public static double gearing = (20d / 1) * (72d / 28); // TODO: Verify
    public static double maxAngle = Units.degreesToRadians(117);
  }

  public static class Elevator {
    public static double gearing = (5d / 1) * (66d / 22); // TODO: Verify
    public static double maxHeight = Units.inchesToMeters(60);
    public static double minHeight = Units.inchesToMeters(0);// TODO: this number is made up
    public static int stages = 3;
    public static double setpointTollerance = Units.inchesToMeters(1);

    public static Slot0Configs motorSlot0 = //TODO tune
        new Slot0Configs()
            .withKS(0)
            .withKV(12d / ((6380d / 60) * gearing)) // Volts/Mechanism RPS 
            .withKP(70)
            .withKI(0)
            .withKD(0);

    public static final double kSupplyLimit = 40;
    public static final double kStatorLimit = 70;

    public static final CurrentLimitsConfigs currentConfigs =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(kStatorLimit)
          .withSupplyCurrentLimit(kSupplyLimit)
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLowerLimit(kSupplyLimit)
          .withSupplyCurrentLowerTime(0);

    public static MotionMagicConfigs kMotionMagicConfig =
        new MotionMagicConfigs()//TODO tune
            .withMotionMagicCruiseVelocity(Units.degreesToRotations(800))
            .withMotionMagicAcceleration(Units.degreesToRotations(1200));
  }
}
