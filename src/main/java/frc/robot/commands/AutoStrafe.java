// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PipelineConstants;
import frc.robot.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoStrafe extends Command {
  SwerveDrive swerveDrive;
  SwerveSubsystem m_drivetrain;
  Limelight m_Limelight;
  double error;
  double pipeline;
  double desired;


  /** Creates a new Command. */
  public AutoStrafe(SwerveSubsystem drivetrain, Limelight limelight, int pipeline, double desired) {
    m_drivetrain = drivetrain;
    m_Limelight = limelight;
    this.pipeline = pipeline;
    this.desired = desired;

    // indicate that this command requires the launcher system
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double maxSpeed = 0.6;
    double targetValid = m_Limelight.validityEntry.getDouble(0.0);
    if (targetValid==1.0)
    {

    //calculate distance from vert angle
      double txOffsetCurr_rad = m_Limelight.xOffEntry.getDouble(0.0) * (3.14159 / 180.0);
      double tyOffsetCurr_rad = m_Limelight.yOffEntry.getDouble(0.0) * (3.14159 / 180.0);
      double distanceToTarget = (m_Limelight.targetHeight-m_Limelight.limelightHeight)/Math.tan(txOffsetCurr_rad);
      double currPos = Math.tan(tyOffsetCurr_rad)*distanceToTarget;

      //calculate distance speed based on established values
      double error = currPos - desired;

      double slowDown = 3;

      double kp = 1/slowDown;
      double speedPercent = kp*error;
      double setSpeed = speedPercent*maxSpeed;
      
      if (Math.abs(setSpeed) > maxSpeed)
      {
        setSpeed = maxSpeed*Math.signum(setSpeed);
      }

      m_drivetrain.drive(new Translation2d(0, setSpeed), 0, false);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0,0), 0, false);
    m_Limelight.limelightTable.getEntry("pipeline").setNumber(PipelineConstants.kPipeline_default);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    
    //return (m_lift.m_liftLeader.getPosition().getValueAsDouble() < setpoint + kEps_claw)&&(m_lift.m_liftLeader.getPosition().getValueAsDouble() > setpoint - kEps_claw);
    return false;
  }
}
