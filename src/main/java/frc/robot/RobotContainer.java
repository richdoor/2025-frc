// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoStrafeConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PipelineConstants;
import frc.robot.Constants.SetYawConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;


import frc.robot.subsystems.swervedrive.LiftSubsystem;
import frc.robot.subsystems.swervedrive.ClawSubsystem;
import frc.robot.commands.AlgaeShoot;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.PID_SetClawPosition;
import frc.robot.commands.PID_SetLiftPosition;
import frc.robot.commands.SetRobotYaw;
import frc.robot.commands.AutoStrafe;
import frc.robot.commands.CoralShoot;
import frc.robot.Constants.LiftConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController mechXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  private final LiftSubsystem m_lift = new LiftSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();

  private final Limelight m_limelight = new Limelight(9.75, 11.125, 0);

  UsbCamera coralCamera;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("R_ClawPos", new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2));
    NamedCommands.registerCommand("L4_LiftPos", new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint4, m_claw));
    NamedCommands.registerCommand("ShootCoral", new CoralShoot(m_claw));
    NamedCommands.registerCommand("L1_LiftPos", new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint1, m_claw));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    coralCamera = CameraServer.startAutomaticCapture();
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.start().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.back().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.povUp().whileTrue(drivebase.centerModulesCommand());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      /*driverXbox.b().whileTrue(
        drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                            );
        */
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(Commands.none());

      //mechXbox.leftBumper().whileTrue(m_lift.raiseLift());
      //mechXbox.rightBumper().whileTrue(m_lift.lowerLift());
      //mechXbox.leftTrigger().whileTrue(m_claw.raiseClaw());
      //mechXbox.rightTrigger().whileTrue(m_claw.lowerClaw());
      
      mechXbox.leftTrigger().whileTrue(m_claw.wheelForward());
      mechXbox.leftBumper().whileTrue(m_claw.wheelBackward());
      mechXbox.rightTrigger().whileTrue(new CoralIntake(m_claw));
      mechXbox.rightBumper().whileTrue(m_claw.wheelBackward());


      // mechXbox.rightTrigger().whileAWTrue(m_claw.wheelForward());
      mechXbox.a().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint1, m_claw));
      mechXbox.x().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint2, m_claw));
      mechXbox.y().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint3, m_claw));
      mechXbox.b().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint4, m_claw));
      
      mechXbox.back().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kAlgaeSetpoint1, m_claw));
      mechXbox.start().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kAlgaeSetpoint2, m_claw));
      mechXbox.rightStick().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kAlgaeSetpoint3, m_claw));

      mechXbox.povUp().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint1));
      mechXbox.povRight().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2));
      mechXbox.povDown().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3));
      mechXbox.povLeft().whileTrue(new AlgaeShoot(m_claw, ClawConstants.kClawSetpoint2, m_lift));
    
      driverXbox.leftTrigger().whileTrue(m_lift.lowerLift());
      driverXbox.rightTrigger().whileTrue(m_lift.raiseLift());
      
      driverXbox.povUp().whileTrue(new SetRobotYaw(drivebase, SetYawConstants.kYawSetpoint4));
      driverXbox.povDown().whileTrue(new SetRobotYaw(drivebase, SetYawConstants.kYawSetpoint1));
      driverXbox.povUpLeft().whileTrue(new SetRobotYaw(drivebase, SetYawConstants.kYawSetpoint5));
      driverXbox.povUpRight().whileTrue(new SetRobotYaw(drivebase, SetYawConstants.kYawSetpoint3));
      driverXbox.povRight().whileTrue(new SetRobotYaw(drivebase, SetYawConstants.kYawSetpoint2));
      driverXbox.povLeft().whileTrue(new SetRobotYaw(drivebase, SetYawConstants.kYawSetpoint6));
      driverXbox.x().whileTrue(new AutoStrafe(drivebase, m_limelight, PipelineConstants.kPipeline_reef, AutoStrafeConstants.kLeftTarget));
      driverXbox.b().whileTrue(new AutoStrafe(drivebase, m_limelight, PipelineConstants.kPipeline_reef, AutoStrafeConstants.kRightTarget));
    
      //driverXbox.y().whileTrue(new SetRobotYaw(drivebase,SetYawConstants.kYawSetpoint1).andThen(new AutoStrafe(drivebase, m_limelight, PipelineConstants.kPipeline_reef)));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("CenterAuto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
