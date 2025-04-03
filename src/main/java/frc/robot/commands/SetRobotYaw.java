// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.ctre.phoenix6.hardware.Pigeon2;
import static frc.robot.Constants.SetYawConstants.*;

public class SetRobotYaw extends Command {
  SwerveSubsystem m_drivetrain;
  Pigeon2 imu = new Pigeon2(13);
  double setpoint;
  double error;
  /** Creates a new Command. */
  public SetRobotYaw(SwerveSubsystem drivetrain, double setpoint) {
    m_drivetrain = drivetrain;
    this.setpoint = setpoint;
    //means this command will take priority over others using same subsystem
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fluctuate between +- 0.005 from kCoralDist
    double yaw = imu.getYaw().getValueAsDouble();
    double adjustedYaw = 0;
    if (yaw >= 0)
    {
      adjustedYaw = yaw % 360;
    }
    else 
    {
      adjustedYaw = Math.abs((yaw % 360)+360);
    }
    error = adjustedYaw-setpoint;
    
    //System.out.println("error"+error);

    double maxSpeed = kYawRotateSpeed;
    double slowDown = kAngleSlowDown;

    double kp = 1/slowDown;
    double speedPercent = kp*error;
    double setSpeed = speedPercent*maxSpeed;
    
    if (Math.abs(setSpeed) > maxSpeed)
    {
      setSpeed = maxSpeed*Math.signum(setSpeed);
    }
    
    

    if (error > 0)
    {
      if (error < 180)
      {
        m_drivetrain.drive(new Translation2d(0,0), -setSpeed, false);
      } 
      else 
      {
        m_drivetrain.drive(new Translation2d(0,0), setSpeed, false);
      }
    }
    else
    {
      if (error > -180)
      {
        m_drivetrain.drive(new Translation2d(0,0), -setSpeed, false);
      } 
      else 
      {
        m_drivetrain.drive(new Translation2d(0,0), setSpeed, false);
      }
    }
      
      /*if (yaw >=0 && yaw <=360)
      {
        if ()
        {
          m_drivetrain.drive(new Translation2d(0,0), setSpeed, false);
        } 
        else
        {
          m_drivetrain.drive(new Translation2d(0,0), -setSpeed, false);
        }
      }
      else if (yaw < 0)
      {

      }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0,0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout

    return  false;
  }
}
