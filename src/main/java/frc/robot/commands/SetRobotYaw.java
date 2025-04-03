// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ClawConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.SetYawConstants;

public class SetRobotYaw extends Command {
  SwerveSubsystem m_drivetrain;
  Pigeon2 imu;
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
    if ((imu.getYaw().getValueAsDouble() < 180) && (imu.getYaw().getValueAsDouble()> 0))
    {
      m_drivetrain.drive(new Translation2d(0,0), -0.3, false);
    }
    else
    {
      m_drivetrain.drive(new Translation2d(0,0), 0.3, false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fluctuate between +- 0.005 from kCoralDist
    if (imu.getYaw().getValueAsDouble()>0)
    {
      error = imu.getYaw().getValueAsDouble()-setpoint;
    }
    else
    {
      error = ((imu.getYaw().getValueAsDouble()+360)-setpoint);
    }
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

    return  (Math.abs(error) < SetYawConstants.kYawEps);
  }
}
