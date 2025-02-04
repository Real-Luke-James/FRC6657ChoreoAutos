package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  public Outtake(OuttakeIO io) {
    this.io = io;
  }

  public Command changeRollerSetpoint(double setpoint){
    return this.runOnce(
      () -> {
        io.changeSetpoint(setpoint);
      });
  }

  public boolean coralDetected(){
    return inputs.beamBroken;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
