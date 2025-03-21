// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LiftConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  //TO DO: -make a max as to not stress the motors/overextend elevator
  //       -make a min/slow down zone to avoid crashing into base (maybe PID loop)
  @Override
  public void execute() {
    System.out.println("lift pos:" + m_lift.m_liftLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Relative Lift Pos", m_lift.m_liftLeader.getPosition().getValueAsDouble());
    /* 
    if (m_lift.m_liftLeader.getPosition().getValueAsDouble() < setpoint - kEps_lift)
    {
      m_lift.raiseLift();
    }
    else if (m_lift.m_liftLeader.getPosition().getValueAsDouble() > setpoint + kEps_lift)
    {
      m_lift.lowerLift();
    }
      */
      //m_lift.setLiftSpeed(kLiftSpeed);
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
    
    //return (m_lift.m_liftLeader.getPosition().getValueAsDouble() < setpoint + kEps_lift)&&(m_lift.m_liftLeader.getPosition().getValueAsDouble() > setpoint - kEps_lift);
    return false;
  }
}
