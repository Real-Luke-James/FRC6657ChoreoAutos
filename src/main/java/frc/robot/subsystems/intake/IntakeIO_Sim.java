package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeIO_Sim implements IntakeIO {

  // Voltage tracking
  private double pivotVoltage = 0;
  private double rollerVoltage = 0;

  // Setpoint Teacking
  @AutoLogOutput(key = "Intake/Speed Setpoint")
  private double speedSetpoint = 0;

  @AutoLogOutput(key = "Intake/Angle Setpoint")
  private double angleSetpoint = Constants.Intake.minAngle;

  // Simulated Motors
  private DCMotorSim pivotSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), 0.0001, Constants.Intake.pivotGearing),
          DCMotor.getNEO(1));
  private DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, 1d / Constants.Intake.rollerGearing),
          DCMotor.getFalcon500(1));

  private PIDController pivotPID = new PIDController(10, 0, 0);

  public IntakeIO_Sim() {
    pivotSim.setState(Units.degreesToRadians(Constants.Intake.minAngle), 0);
    pivotPID.setTolerance(Units.degreesToRadians(5));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    updatePID(); // update PID Controllers

    // update sims
    pivotSim.update(1 / Constants.mainLoopFrequency);
    rollerSim.update(1 / Constants.mainLoopFrequency);

    // Pivot Inputs
    inputs.pivotMotorPosition = pivotSim.getAngularPositionRad();
    inputs.encoderAbsPosition = pivotSim.getAngularPositionRad();
    inputs.pivotMotorVelocity = pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotMotorTemp = 0; // Celcius
    inputs.pivotMotorVoltage = pivotVoltage; // Volts (set to 0)
    inputs.pivotMotorCurrent = pivotSim.getCurrentDrawAmps(); // Amps
    inputs.pivotMotorSetpoint = angleSetpoint;

    // Roller Inputs
    inputs.rollerMotorVelocity = rollerSim.getAngularVelocityRPM(); // RPM
    inputs.rollerMotorTemp = 0; // Celcius
    inputs.rollerMotorVoltage = rollerVoltage; // Volts (set to 0)
    inputs.rollerMotorCurrent = rollerSim.getCurrentDrawAmps(); // Amps
  }

  /**
   * Change the setpoint of the shooter pivot
   *
   * @param angleDegrees The new setpoint in radians
   *     <p>Acceptable Range: Find it.
   */
  @Override
  public void changePivotSetpoint(double angle) {
    angleSetpoint = angle;
  }

  /** Acceptable Range: [-1, 1] Positive RPM may be shooting? */
  @Override
  public void changeRollerSpeed(double speed) {
    rollerVoltage = speed;
  }

  private void updatePID() {
    // Pivot
    double pivotPIDEffort = pivotPID.calculate(pivotSim.getAngularPositionRad(), angleSetpoint);
    pivotVoltage = MathUtil.clamp(pivotPIDEffort, -12, 12);
    pivotSim.setInput(pivotVoltage);

    // Roller
    rollerVoltage = speedSetpoint;
    rollerSim.setInput(speedSetpoint * RobotController.getBatteryVoltage());
  }
}
