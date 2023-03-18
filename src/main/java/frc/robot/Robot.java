// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

// xbox controler class
import edu.wpi.first.wpilibj.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final Timer m_timer = new Timer();

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    

    //below is 2022 code for displaying automation information to smart dashboard

    //m_chooser.setDefaultOption("Single Cargo", singleCargo);
    //m_chooser.addOption("Dual Cargo", dualCargo);
    //SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //Below is commented out code for select automation command for 2022-2023
    //m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto); //old
    //System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //Commented out code below is 2021-2022 automation code
    //switch (m_autoSelected) {
      //case singleCargo:
      //Robot possitioned in front of hub for single cargo
       /*  if (m_timer.get() < 0.2){
          // makes sure arm is all the way up
          arm.set(0.3);
        } else if (m_timer.get() < 0.6){
          //keeps arm up and shoots cargo
          arm.set(0.02);
          armShoot.set(-1);
        } else if (m_timer.get()< 2.35){
          //stops shooting cargo and moves robot backwards
          armShoot.set(0);
          motor0.set(0.50);
          motor1.set(0.50);
          motor2.set(-0.50);
          motor3.set(-0.50);
        } else{
          //stops robot
          motor0.set(0);
          motor1.set(0);
          motor2.set(0);
          motor3.set(0);
        }*/
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //arm.getEncoder().setPosition(0); // reset the encoder
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

