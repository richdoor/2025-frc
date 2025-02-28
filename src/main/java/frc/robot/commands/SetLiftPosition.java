// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LiftConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.LiftSubsystem;


public class SetLiftPosition extends Command {
  LiftSubsystem m_lift;
  double setpoint;

  /** Creates a new Command. */
  public SetLiftPosition(LiftSubsystem lift, double setpoint) {
    m_lift = lift;
    this.setpoint = setpoint;

    //means this command will take priority over others using same subsystem
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
    
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_lift.m_liftLeader.getPosition().getValueAsDouble());
    if (m_lift.m_liftLeader.getPosition().getValueAsDouble() < setpoint - kEps_lift)
    {
      m_lift.raiseLift();
    }
    else if (m_lift.m_liftLeader.getPosition().getValueAsDouble() > setpoint + kEps_lift)
    {
      m_lift.lowerLift();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    
    return (m_lift.m_liftLeader.getPosition().getValueAsDouble() < setpoint + kEps_lift)&&(m_lift.m_liftLeader.getPosition().getValueAsDouble() > setpoint - kEps_lift);
  }
}
