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

  /*private static final int kArmSparkMaxCANID = 8;

  private final Talon motor0 = new Talon(0); // left stick forward  (last year 0) back left wheel
  private final Talon motor1 = new Talon(1); // left stick forward  (last year 3) front left wheel
  private final Talon motor2 = new Talon(2); // right stick forward (last year 2) back right wheel
  private final Talon motor3 = new Talon(3); // right stick forward (last year 1) front right wheel
  //private final Talon climber = new Talon(4);  //year 

  private final Spark armShoot = new Spark(4);

  private final CANSparkMax arm = new CANSparkMax(kArmSparkMaxCANID, MotorType.kBrushless);
  // private final PWMSparkMax centerMotor = new PWMSparkMax(9);
  //private static final int mainDeviceID = 8;
  //private CANSparkMax centerMotor;
  //centerMotor1 = new CANSparkMax(mainDeviceID, MotorType.kBrushless);

 
  private final XboxController controller1 = new XboxController(1);
  private final XboxController controller0 = new XboxController(0);
  boolean directionReversed = false;*/

  private final Timer m_timer = new Timer();

  //private static final String driveForward = "Drive Forward";
  //private static final String lowGoal = "Low Goal";
  //auto
  //private static final String singleCargo = "Single Cargo";
  //private static final String dualCargo = "Dual Cargo";
  //private String m_autoSelected;
  //private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    //centerMotor = new CANSparkMax(mainDeviceID, MotorType.kBrushless);
    //centerMotor.restoreFactoryDefaults();

    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);


    //arm.setIdleMode(IdleMode.kBrake);

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
   /*  //Motor rate
    double Lrate = 1; //left motor rate
    double Rrate = 1; //right motor rate

    //slow down drive speed
    double driveSlow = 1; //slows down drive speed

    // drive motors
    if(!directionReversed) {
      if (controller0.getRawAxis(1) < 0.2 && controller0.getRawAxis(1) > -0.2) {
        // left motors dead zone
        motor0.set(0);
        motor1.set(0);
      } else {
        //left motors move
        motor0.set(controller0.getRawAxis(1) * -1.00 * Lrate * driveSlow);
        motor1.set(controller0.getRawAxis(1) * -1.00* Lrate * driveSlow);
      }
      if (controller0.getRawAxis(5) < 0.2 && controller0.getRawAxis(5) > -0.2) { 
        //Right motors dead zone 
        motor2.set(0);
        motor3.set(0);
      } else {
        //Right motors move
        motor2.set(controller0.getRawAxis(5) * 1.00 * Rrate * driveSlow);
        motor3.set(controller0.getRawAxis(5) * 1.00 * Rrate  * driveSlow);
      }
      // if direction is reversed (by press of A butto)
    } else {
      if (controller0.getRawAxis(1) < 0.2 && controller0.getRawAxis(1) > -0.2) {
        //Left motor dead zone
        motor2.set(0);
        motor3.set(0);
      } else {
        // Left motor move
        motor2.set(controller0.getRawAxis(1) * -1.00 * Lrate * driveSlow);
        motor3.set(controller0.getRawAxis(1) * -1.00 * Lrate * driveSlow);
      }
      if (controller0.getRawAxis(5) < 0.2 && controller0.getRawAxis(5) > -0.2) {
        // Right motor
        motor0.set(0);
        motor1.set(0);
      } else {
        // Right motor move
        motor0.set(controller0.getRawAxis(5) * 1.00 * Rrate * driveSlow);
        motor1.set(controller0.getRawAxis(5) * 1.00 * Rrate * driveSlow);
      }
    }

    //when Y button pressed reverse direction
    //does not work

    SmartDashboard.putBoolean("Direction Reversed", directionReversed);

    if (controller0.getYButtonPressed()) {
      if (!directionReversed) {
        directionReversed = true;
      } else {
        directionReversed = false;
      }
    }
    //when B button slow robot down
    if (controller0.getBButtonPressed()){
      if (driveSlow == 1){
        driveSlow = 0.5;
      } else {
        driveSlow = 1;
      }
    }
    // arm rotation and shooting

    //Modifies speed of arm motor with the Left Stick
    double armRate = 0.40;
    //modifies shooting speed
    double shootRate = 1;
    // modifies intake speed
    double intakeRate = 1;


    double shootArmAxis = -MathUtil.clamp(controller1.getLeftY(), -0.70, 0.70); // Apply axis clamp and invert for driver control

    if (Math.abs(shootArmAxis) > 0.2) arm.set(shootArmAxis * armRate);
    else arm.set(0);

    SmartDashboard.putNumber("arm power", shootArmAxis * armRate); // put arm speed on Smartdash

    SmartDashboard.putNumber("arm enc value", arm.getEncoder().getPosition()); // put encoder value on SmartDash

    // - shoot cargo; + intake cargo
    armShoot.set(controller1.getRightTriggerAxis() * intakeRate- controller1.getLeftTriggerAxis() * shootRate);
*/
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

