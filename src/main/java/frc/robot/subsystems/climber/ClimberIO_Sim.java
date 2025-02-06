package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ClimberIO_Sim implements ClimberIO {
    // voltage tracking
    private double voltage = 0;

    // setpoint tracking
    @AutoLogOutput(key = "Climber/Speed Setpoint")
    private double speedSetpoint = 0;

    @AutoLogOutput(key = "Climber/Angle Setpoint")
    private double angleSetpoint = Constants.Climber.minAngle;

    private DCMotorSim climberSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(speedSetpoint, angleSetpoint), null, null)
}
