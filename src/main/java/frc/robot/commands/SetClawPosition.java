// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.swervedrive.ClawSubsystem;


public class SetClawPosition extends Command {
  ClawSubsystem m_claw;
  double setpoint;

  /** Creates a new Command. */
  public SetClawPosition(ClawSubsystem claw, double setpoint) {
    m_claw = claw;
    this.setpoint = setpoint;

    //means this command will take priority over others using same subsystem
    addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
    
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_claw.m_clawPosition.getAbsolutePosition().getValueAsDouble());
    if (m_claw.m_clawPosition.getAbsolutePosition().getValueAsDouble() < setpoint - kEps_claw)
    {
      m_claw.raiseClaw();
    }
    else if (m_claw.m_clawPosition.getAbsolutePosition().getValueAsDouble() > setpoint + kEps_claw)
    {
      m_claw.lowerClaw();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopClawRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    
    return (m_claw.m_clawPosition.getAbsolutePosition().getValueAsDouble() < setpoint + kEps_claw)&&(m_claw.m_clawPosition.getAbsolutePosition().getValueAsDouble() > setpoint - kEps_claw);
  }
}
