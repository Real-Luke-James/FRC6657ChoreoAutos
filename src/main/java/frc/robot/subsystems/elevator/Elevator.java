package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Pose3d[] get3DPoses() {
    return new Pose3d[] {
      new Pose3d(0, 0, inputs.kPosition * (1d / 3), new Rotation3d()),
      new Pose3d(0, 0, inputs.kPosition * (2d / 3), new Rotation3d()),
      new Pose3d(0, 0, inputs.kPosition * (3d / 3), new Rotation3d()),
    };
  }

  /**
   * @param setpoint in Carriage Meters
   * @return
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(
              MathUtil.clamp(setpoint, Constants.Elevator.minHeight, Constants.Elevator.maxHeight));
        });
  }

  public Command changeSetpoint(DoubleSupplier setpointSupplier) {
    return this.runOnce(
        () ->
            io.changeSetpoint(
                MathUtil.clamp(
                    setpointSupplier.getAsDouble(),
                    Constants.Elevator.minHeight,
                    Constants.Elevator.maxHeight)));
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.kSetpoint, inputs.kPosition, Units.inchesToMeters(1));
  }

  public Command setRawVoltage(double voltage) {
    return this.runOnce(
        () -> {
          io.setRawVoltage(voltage);
        });
  }

  public double position() {
    return inputs.kPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);
  }
}
