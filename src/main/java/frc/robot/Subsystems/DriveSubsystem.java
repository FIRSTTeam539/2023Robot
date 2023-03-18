package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.fasterxml.jackson.annotation.Nulls;

//import edu.wpi.first.wpilibj.ADXL345_SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  //gyro
  AHRS ahrs;
  
  boolean autoBalanceXMode;
  boolean autoBalanceYMode;
  
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new WPI_VictorSPX(DriveConstants.kLeftMotor1Port),
          new WPI_VictorSPX(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          new WPI_VictorSPX(DriveConstants.kRightMotor1Port),
          new WPI_VictorSPX(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_leftMotors.setInverted(true); //used before code below
    m_rightMotors.setInverted(true); //attempting to make + forweard and back -
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
  }
  /**
   * Drive Robot Using tank controls
   * 
   *@param left left motor movement; + forward, - backwards
   @param right right motor movement; + forward, - backwards
   *   
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  //gyro code
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(ahrs.getYaw(), 360) *  (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  public void auto_balance(){
      double xAxisRate;
      double yAxisRate;
      double pitchAngleDegrees    = ahrs.getPitch();
      double rollAngleDegrees     = ahrs.getRoll();
      
      if ( !autoBalanceXMode && 
           (Math.abs(pitchAngleDegrees) >= 
            Math.abs(DriveConstants.kOffBalanceAngleThresholdDegrees))) {
          autoBalanceXMode = true;
      }
      else if ( autoBalanceXMode && 
                (Math.abs(pitchAngleDegrees) <= 
                 Math.abs(DriveConstants.kOnBalanceAngleThresholdDegrees))) {
          autoBalanceXMode = false;
                 }
      
      // Control drive system automatically, 
      // driving in reverse direction of pitch/roll angle,
      // with a magnitude based upon the angle
      
      if ( autoBalanceXMode ) {
          double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
          xAxisRate = Math.sin(pitchAngleRadians) * -1;
      }
      
      try {
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);  
        DifferentialDrive.tankDriveIK(pitchAngleDegrees, rollAngleDegrees, autoBalanceXMode);
      } catch( RuntimeException ex ) {
          String err_string = "Drive system error:  " + ex.getMessage();
          DriverStation.reportError(err_string, true);
      }
  }
  /**
   * set a distance for robot to drive to in inches is not command
   * @param distance distance to drive to in inches
   * @param speed is the precent speed of motors to move at
   * 
   */
  private void driveDistance (double distance, double speed){
    double  distance_drive = 0;
    double drive_speed = speed;
    if (distance < 0) drive_speed *= -1;
    m_drive.tankDrive(drive_speed, drive_speed);
    distance_drive += this.getAverageEncoderDistance();
    if (distance_drive == distance){
      m_drive.tankDrive(0, 0);
    }
  }
  /**
   * set a distance for robot to drive to in inches
   * @param distance distance to drive to in inches
   * @param speed is the precent speed of motors to move at
   * @return returns command
   */
  public Command driveTo (double distance, double drive_speed){
    return this.run(() -> this.driveDistance(distance, drive_speed));
  }
  public Command stop() {
    return this.run(() -> this.tankDrive(0, 0));
  }
  public Command autoBalanceCommand (){
    return this.startEnd(() -> this.auto_balance(), () -> this.tankDrive(0, 0));
  }


}