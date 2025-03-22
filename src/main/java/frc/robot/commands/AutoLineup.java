// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import swervelib.SwerveDrive;
import frc.robot.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.PipelineConstants;

// import frc.robot.subsystems.CANLauncher;

public class AutoLineup extends Command {
  SwerveDrive swerveDrive;
  SwerveSubsystem m_drivetrain;
  Limelight m_Limelight;
  double time_prev = 0;
  double error_distance;
  double error_angle;
  double eps_distance;
  double eps_angle;
  double pipeline;
  

  // CANLauncher m_ampLauncher;

  /** Creates a new PrepareLaunch. */
  public AutoLineup(SwerveSubsystem drivetrain, Limelight limelight, int pipeline) {
    // save the launcher system internally
    m_drivetrain = drivetrain;
    m_Limelight = limelight;
    this.pipeline = pipeline;

    // indicate that this command requires the launcher system
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.limelightTable.getEntry("pipeline").setNumber(pipeline);

    /*System.out.println("drive command start");
    m_drivetrain.driveCommand(
        ()->{return 0.5;},
        ()->{return 0.5;},
        ()->{return 90;});
    System.out.println("drive command start");
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time_curr = Timer.getFPGATimestamp();
    double targetValid = m_Limelight.validityEntry.getDouble(0.0);

    //DISTANCE CONTROLLER
    //could make some variables constants for organization; for now here for convenience
    //values to tune
    eps_distance = 4.0;
    double maxSpeed_distance = 0.3; 
    double slowDown_distance = 5; //this is used for the k_p value
    double desired_distance = 51;

    //calculate distance from vert angle
    double targetOffsetAngle_Vertical = m_Limelight.yOffEntry.getDouble(0.0);
    double angleToGoalDegrees = m_Limelight.limelightAngle + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double actual_distance = (m_Limelight.targetHeight - m_Limelight.limelightHeight)/Math.tan(angleToGoalRadians);

    //calculate distance speed based on established values
    double kp_distance = 1/slowDown_distance;
    double error_distance = actual_distance - desired_distance;
    double speedPercent_distance = kp_distance*error_distance;
    double setSpeed_distance = speedPercent_distance*maxSpeed_distance;

    //we are clipping max controller effort
    if (Math.abs(setSpeed_distance) > maxSpeed_distance)
    {
      setSpeed_distance = maxSpeed_distance*Math.signum(setSpeed_distance);
    }

    



    //ANGLE CONTROLLER
    eps_angle = 4.0;
    double maxSpeed_angle = 0.5; 
    double slowDown_angle = 20; //this is used for the k_p value
    double desired_angle = 0;

    //calculate angle from vert angle
    double actual_angle = m_Limelight.xOffEntry.getDouble(0.0);

    //calculate angle speed based on established values
    double kp_angle = 1/slowDown_angle;
    error_angle = actual_angle - desired_angle;
    double speedPercent_angle = kp_angle*error_angle;
    double setSpeed_angle = speedPercent_angle*maxSpeed_angle;

    //we are clipping max controller effort
    if (Math.abs(setSpeed_angle) > maxSpeed_angle)
    {
      setSpeed_angle = maxSpeed_angle*Math.signum(setSpeed_angle);
    }


    //DRIVE
    if (targetValid==1.0)
    {
      m_drivetrain.drive(new Translation2d(setSpeed_distance,0), setSpeed_angle, false);
    }
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
    m_drivetrain.drive(new Translation2d(0,0), 0, false);
    m_Limelight.limelightTable.getEntry("pipeline").setNumber(PipelineConstants.kPipeline_default);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    // decorator on the command to end it.
    //CHECKER
    boolean inPosition = false;
    if ((Math.abs(error_distance) < eps_distance) && (Math.abs(error_angle) < eps_angle))
    {
      inPosition = true;
    }
    return inPosition;
  }
}
