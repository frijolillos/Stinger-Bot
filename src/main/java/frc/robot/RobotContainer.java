// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.CommandManager;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VisionShot;
import frc.robot.commands.autoCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climbers.Climbers;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.subsystems.transfer.Transfer;

public class RobotContainer {
  //Controllers
  private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverControllerPort);
  public final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorControllerPort);

  private final SwerveBase swerveBase;
  private final Intake intakeSub;
  private final Transfer transferSub;
  private final Arm armSub;
  private final Shooter shooterSub;
  private final Climbers climberSub;

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    intakeSub = new Intake();

    transferSub = new Transfer();

    armSub = new Arm();

    shooterSub = new Shooter();

    climberSub = new Climbers();


    //Try moving where bindings are configured
    //Configures Control Bindings
    configureDriverBindings();
    configureOperatorBindings();

    // Register Named Commands
    NamedCommands.registerCommand("autoIntake", autoCommands.intakeNote(intakeSub, transferSub));
    NamedCommands.registerCommand("shootNote", autoCommands.shootNote(shooterSub, transferSub));
    NamedCommands.registerCommand("midArmShot", autoCommands.midArmShot(armSub, shooterSub, transferSub));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverBindings(){
  /* Driver Controls */
  
    swerveBase.setDefaultCommand(
      new TeleopSwerve(
        swerveBase,
        //inverts controls because joysticks are back-right (+) while robot is front-left (+)
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        //REVIEW - Do we need to keep?
        () -> driverController.rightTrigger().getAsBoolean(),//used to drive robot relative
        () -> driverController.leftBumper().getAsBoolean(),//used to decrease speed of chassis
        () -> driverController.rightBumper().getAsBoolean()//used to lock onto speaker
        ));

  //Y Button: Zero Gyro
  //TODO - Swap buttons out with periodic check of alliance color and value[x,y] inversation based on it
  //REVIEW -Relocate button, is useful for showcases
    driverController.y().onTrue(CommandManager.zeroGyro(swerveBase));

    driverController.start().onTrue(CommandManager.redReset(swerveBase, 180));

    /**
     * Climber bindings 
     *  Press and hold up d-pad for as long as you want to keep climbers going up
     *  Press and hold down d-pad for as long as you want to keep climbers going down
     */
    driverController.povUp().whileTrue(new InstantCommand(()-> climberSub.climbUpTest())).whileFalse(new InstantCommand(()-> climberSub.stopClimber()));
    driverController.povDown().whileTrue(new InstantCommand(() -> climberSub.climbDownTest())).whileFalse(new InstantCommand(()-> climberSub.stopClimber()));

    /*Climber setpoints */
    //Auto set's climbers 
    //climbers max height
    //Make sure max value is safe
    driverController.povLeft().onTrue(CommandManager.climberMove(climberSub, ClimberConstants.maxHight));
    //climbers min height
    //Make sure min value is safe
    driverController.povRight().onTrue(CommandManager.climberMove(climberSub, ClimberConstants.minHeight));

  }

  private void configureOperatorBindings(){
  /*add vision to determine speed and angle necessary to score in speaker */

    /**
     * Intake Related Bindings
     * */
    //Left Bumber: Auto intake *Press once and it will run until canceled or overwritten*
    operatorController.leftBumper().onTrue(
      autoCommands.intakeNote(intakeSub, transferSub));

    //Dpad up: manually intake
    operatorController.povUp().whileTrue(
      CommandManager.intakeNote(intakeSub, transferSub)).onFalse(CommandManager.stopIntaking(intakeSub, transferSub));

    //Dpad down: manually spit out
    operatorController.povDown().whileTrue(
      CommandManager.spitNote(intakeSub, transferSub)).onFalse(CommandManager.stopIntaking(intakeSub, transferSub));

    //POV Left: E stop for intake and trasnfer [Added requirements on Instant Commands so should interrupt autoIntaking]
    operatorController.start().onTrue(
      CommandManager.eStop(intakeSub, transferSub, armSub, shooterSub));

  /**
  * Arm Related Bindings
  * */
  //amp state
    operatorController.y()
      .onTrue(
        new ParallelCommandGroup(
          new InstantCommand(()-> armSub.setPosition(0.218), armSub), // pos 0.23
          new InstantCommand(() -> shooterSub.redirect(-0.35), shooterSub)));


    //slighly elevated state
    operatorController.b()
      .onTrue(
        new InstantCommand(()-> armSub.setPosition(0.096), armSub));

    //slighly elevated state
    operatorController.x()
      .onTrue(
        new InstantCommand(()-> armSub.setPosition(0.024), armSub));
    
    //bring arm to default position
    operatorController.a()
      .onTrue(
        new InstantCommand(()-> armSub.setPosition(0.0), armSub));

  /**
  * Shoot Related Bindings
  * */
  //Right Trigger: Manual speaker speed [dutycycle]
    operatorController.leftTrigger().whileTrue(
      new InstantCommand(() -> shooterSub.dutyShot(ShooterConstants.dutySpeakerShot)))
        .onFalse(new InstantCommand(()-> shooterSub.dutyStop()));
    /* 
    operatorController.leftTrigger().whileTrue(
      new InstantCommand(() -> shooterSub.dutyShot(1)))\[]

        .onFalse(new InstantCommand(()-> shooterSub.dutyStop()));
    */

    operatorController.rightTrigger().whileTrue(
      new VisionShot(armSub, shooterSub, swerveBase));

    //Trap shot
    operatorController.rightStick().whileTrue(new InstantCommand(() -> shooterSub.dutyShot(0.38)))
      .onFalse(new InstantCommand(()-> shooterSub.dutyStop()));

  //Right Bumber: Feed Note to shooter [run transfer]
    operatorController.rightBumper().whileTrue(
      CommandManager.feedNote(transferSub))
        .onFalse(new InstantCommand(()-> transferSub.stopTransfer()));

  } 

/* 
  //[OG Saftey Auto]
  private Command preloadAutoAuton(){
    return new InstantCommand(
      () -> shooterSub.setShooterSpeed(.5)).andThen(Commands.waitSeconds(7)).andThen(new InstantCommand(() -> transferSub.setTransferSpeed(.35))).andThen(Commands.waitSeconds(2)).andThen(new InstantCommand(() -> shooterSub.dutyStop())).andThen(new InstantCommand(() -> transferSub.stopTransfer()));
  }
*/

  public Command getAutonomousCommand() {
    //return preloadAutoAuton();
    return autoChooser.getSelected();
}
}