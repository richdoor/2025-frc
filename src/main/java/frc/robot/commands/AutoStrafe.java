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
import frc.robot.subsystems.swervedrive.ClawSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.PipelineConstants;
import frc.robot.Constants.AutoStrafeConstants;


// import frc.robot.subsystems.CANLauncher;

public class AutoStrafe extends Command {
  SwerveDrive swerveDrive;
  SwerveSubsystem m_drivetrain;
  Limelight m_Limelight;
  double error;
  double pipeline;
  

  // CANLauncher m_ampLauncher;

  /** Creates a new PrepareLaunch. */
  public AutoStrafe(SwerveSubsystem drivetrain, Limelight limelight, int pipeline) {
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
    double slowDownDist = 5; //this is used for the k_p value
    double desired = AutoStrafeConstants.kTxOffset;

    //calculate distance from vert angle
    double txOffsetCurr = m_Limelight.xOffEntry.getDouble(0.0);

    //calculate distance speed based on established values
    double kp = 1/slowDownDist;
    double error = txOffsetCurr - desired;
    double speedPercent = kp*error;
    double setSpeed = speedPercent*AutoStrafeConstants.kMaxStrafeSpeed;

    //we are clipping max controller effort
    if (Math.abs(setSpeed) > AutoStrafeConstants.kMaxStrafeSpeed)
    {
      setSpeed = AutoStrafeConstants.kMaxStrafeSpeed*Math.signum(setSpeed);
    }

    //DRIVE
    if (targetValid==1.0)
    {
      m_drivetrain.drive(new Translation2d(0,setSpeed), 0, false);
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
    return (Math.abs(error) < AutoStrafeConstants.kStrafeEps);
  }
}
