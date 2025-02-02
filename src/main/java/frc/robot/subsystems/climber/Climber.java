// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  public final ClimberIO io;
  public final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(IntakeIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double angleDegrees) {
    return this.runOnce(
      () ->
          io.changeSetpoint(
            MathUtil.clamp(
                angleDegrees, Constants.Climber.minAngle, Constants.Climber.maxAngle)
          ) 
    );
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
