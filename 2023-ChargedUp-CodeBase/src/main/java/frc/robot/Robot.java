/* 
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
*/

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Servo;
//import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
  // Field Assignments
  // Motors
  private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(3, MotorType.kBrushless);
  //private RelativeEncoder leftEncoder = leftMotor1.getEncoder();
  //private RelativeEncoder righEncoder = rightMotor1.getEncoder();
  private DifferentialDrive myDrive;

  // Limit Switches
  //private DigitalInput legLocked = new DigitalInput(0);
  //private DigitalInput legUp = new DigitalInput(1);

  // Acceleration Curve
  // Two seconds worth of ticks at 20ms per tick is 100, 
  // Double that for positive and negative acceleration
  // Finally, add one for full stop...that's 201 ticks

  private int maxTicks = 201;
  private int currentIndexLeft = maxTicks / 2;
  private int currentIndexRight = maxTicks / 2;
  private ArrayList<Double> settingsList;
  private double acceleration_midpoint, acceleration_curvature;
  
  // nav-X Micro
  private AHRS navX2;

  // Joysticks
  private Joystick joyDriver = new Joystick(0);
  private Joystick joySpotter = new Joystick(1);

  // Controller Button and Axis Constants because I'm tire of looking them up
      //private final int X_BUTTON = 1;
  private final int A_BUTTON = 2;
  private final int B_BUTTON = 3;
      //private final int Y_BUTTON = 4;
  private final int LB_BUTTON = 5;
  private final int RB_BUTTON = 6;
  private final int LT_BUTTON = 7;
  private final int RT_BUTTON = 8;
      // private final int BACK_BUTTON = 9;
      // private final int START_BUTTON = 10;
      //private final int LS_BUTTON = 11;
      //private final int RS_BUTTON = 12;
      //private final int LR_AXIS_LEFT_STICK = 0;
  private final int UD_AXIS_LEFT_STICK = 1;
      //private final int LR_AXIS_RIGHT_STICK = 2;
  private final int UD_AXIS_RIGHT_STICK = 3;

  // Pipeline Constants
  private final int CONE = 0;
  private final int CUBE = 1;
  private final int RETRO = 2;
  private final int TAGS_SCORE = 3;
  private final int TAGS_LOAD = 4;

  // Servo Angle Constants
  private final double OPEN_ANGLE = 120.0;
  private final double CLOSE_ANGLE = 30.0;
  
  // Robot Hardware
  Compressor myCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Solenoid grabberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
  Solenoid legSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  Servo valveServo = new Servo(0);
  
  // NetworkTables Global Variables for use with Limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry txEntry = table.getEntry("tx");
  NetworkTableEntry tyEntry = table.getEntry("ty");
  NetworkTableEntry taEntry = table.getEntry("ta");

  // Variable that controls autonomous profile selection
  private String autoSelectionName = "Easy";

  // Auto-Aim and Auto-Balance Variables
  private double steering_adjust = 0.0;
  private double distance_adjust = 0.0;

  @Override
  public void robotInit() 
  {
    // Initialize Acceleration Curve settings
    // Logistic Curve values
    acceleration_curvature = -0.1;
    acceleration_midpoint = 25;
    populateArrayList();
    
    // Reset controller settings and set Idle Mode to Coast
    leftMotor1.restoreFactoryDefaults();
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor2.restoreFactoryDefaults();
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.restoreFactoryDefaults();
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor2.restoreFactoryDefaults();
    rightMotor2.setIdleMode(IdleMode.kCoast);

    // Set secondary motors on each side as followers
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    // Set left side motors inverted so that we can use the same 
    // numbers for both sides without worrying about signs
    leftMotor1.setInverted(true);
    
    // Enable the compressor and set up the Drivetrain
    myCompressor.enableDigital();
    myDrive = new DifferentialDrive(leftMotor1, rightMotor1);

    
    // Create and calibrate the navX2 sensor package
    try
    {
      navX2 = new AHRS(SerialPort.Port.kUSB);
    }
    catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    navX2.calibrate();
  }

  @Override
  public void robotPeriodic() 
  {
    // Check Spotter Joystick for input
    joySpotter();

    // Show which Autonomous is Selected
    SmartDashboard.putString("Auto Selected", autoSelectionName);

    // // Acceleration Curve Values
    // SmartDashboard.putNumber("Curvature", acceleration_curvature);
    // SmartDashboard.putNumber("Midpoint", acceleration_midpoint);

    // NavX Info 
    SmartDashboard.putNumber("NavX Yaw", Math.floor(navX2.getYaw()));
    SmartDashboard.putNumber("NavX Pitch", Math.floor(navX2.getPitch()));
    SmartDashboard.putNumber("NavX Roll", Math.floor(navX2.getRoll()));
    
    // // Limelight Info
    //   //read values periodically
    //   double tx = Math.floor(txEntry.getDouble(0.0));
    //   double ty = Math.floor(tyEntry.getDouble(0.0));
    //   double ta = Math.floor(taEntry.getDouble(0.0));
    //   double tagID = Math.floor(table.getEntry("tid").getDouble(-1.0));

    //   //post to smart dashboard periodically
    //   SmartDashboard.putNumber("LimelightX", tx);
    //   SmartDashboard.putNumber("LimelightY", ty);
    //   SmartDashboard.putNumber("LimelightArea", ta);
    //   SmartDashboard.putNumber("AprilTag Value", tagID);  

  }

  @Override
  public void autonomousPeriodic() 
  {
    //Timer timer = new Timer();

    boolean[] stageComplete = {false, false, false, false, false};
    int tagID = (int)table.getEntry("tid").getDouble(-1.0);
    if(autoSelectionName.equals("Easy"))
    {
      
      ///* Very Easy Mode
      // 6-9 Point Auto Based partly on luck
      // Find AprilTag
      // Stage 0 - Drive to Position
      // Stage 1 - Drop Cube off the back of the robot after bumping into the goal area
      // OR: Stage 1 - Release launcher to slap cube up to top position
      // Stage 2 - Back out of Community
      if(!stageComplete[0] && tagID > -1)
      {
        
        stageComplete[0] = true;
      }
      else if(!stageComplete[1])
      { 
        
        stageComplete[1] = true;
      }
      else if(!stageComplete[2])
      {

        stageComplete[2] = true;
      }
      //*/
    }
    else if(autoSelectionName.equals("Medium"))
    {
      /* Medium Mode
      // 9 Point Auto
      // Find AprilTag (1/3 or 6/8 Depending on Alliance)
      //ws Drive to Position
      // Drop Cube (Top Level - 6pts)
      // Back out of Community (3pts)
      */
    }
    else if(autoSelectionName.equals("Hard"))
    {
      /* Hard Mode
      // 21 Point Auto
      // Raise Leg
      // Find AprilTag (2 or 7 Depending on Alliance)
      // Drive to Position
      // Drop Cube (Top Level - 6pts)
      // Back out of community over charge station (3pts)
      // Drive back to charge station and balance (12pts)
      */
    }
    else if(autoSelectionName.equals("Dream"))
    {
      /* DREAM Mode
       * 27 Point Auto
       * 
       */
    }
  }


  @Override
  public void teleopPeriodic() 
  {
    joyDriver();
    double left = 0.0;
    double right = 0.0;
    double desired_setpoint_left = steering_adjust + distance_adjust;
    double desired_setpoint_right = -steering_adjust + distance_adjust;

    // Get Desired Setpoints based on Stick 
    desired_setpoint_left += (double)((int)(joyDriver.getRawAxis(UD_AXIS_LEFT_STICK)*10000))/10000;
    desired_setpoint_right += (double)((int)(joyDriver.getRawAxis(UD_AXIS_RIGHT_STICK)*10000))/10000;

    currentIndexLeft = updateMotorIndex(desired_setpoint_left, currentIndexLeft);
    currentIndexRight = updateMotorIndex(desired_setpoint_right, currentIndexRight);

    left = settingsList.get(currentIndexLeft);
    right = settingsList.get(currentIndexRight);

    // Test Output
    // SmartDashboard.putNumber("Setpoint Left", desired_setpoint_left);
    // SmartDashboard.putNumber("Setpoint Right", desired_setpoint_right);
    // SmartDashboard.putNumber("Current Left", left);
    // SmartDashboard.putNumber("Current Right", right);
    
    // SmartDashboard.putNumber("Output Left", leftMotor1.getAppliedOutput());
    // SmartDashboard.putNumber("Output Right", rightMotor1.getAppliedOutput());

    // Set Motors to new speed and updates tracking variables...Always do this last
    myDrive.tankDrive(left, right);
    steering_adjust = 0.0;
    distance_adjust = 0.0;
  }

  public void autoAim(double tx, double ty)
  {
    double KpAim = 0.02;
    double KpDistance = 0.02;
    double min_aim_command = 0.02;

    double heading_error = -tx;
    double distance_error = -ty;

    if (tx > 1.0)
    {
      steering_adjust = KpAim*heading_error - min_aim_command;
    }
    else if (tx < -1.0)
    {
      steering_adjust = KpAim*heading_error + min_aim_command;
    }

    distance_adjust = KpDistance * distance_error;   
  }

  public void balance()
  {
    double roll_error = navX2.getRoll();

    // Set Adjustment for Yaw Direction
    distance_adjust = roll_error / 90;
  }

  public int updateMotorIndex(double setpoint, int index)
  {
    // Full stop error correction for joystick drift
    if(index < maxTicks/2 + 4 && index > maxTicks/2 - 4){
      if(Math.abs(setpoint) < settingsList.get(maxTicks/2 + 1))
      {
        return maxTicks/2;
      }
    }

    double currentSetting = settingsList.get(index);

    // Update Index
    if(currentSetting < setpoint)
    {
      if(index < settingsList.size() - 1)
      {
        index = index + 1;
      }
      else
      {
        index = settingsList.size() - 1;
      }
    }
    else if(currentSetting > setpoint)
    {
      if(index > 0)
      {
        index = index - 1;
      }
      else
      {
        index = 0;
      }
    }

    // Limit speed during in-place rotations and when leg is unlocked
    if(leftMotor1.getAppliedOutput() * rightMotor1.getAppliedOutput() < 0) // || !legLocked.get())
    {
      if(index < maxTicks/3)
      {
        index = maxTicks/3;
      }
      else if(index > maxTicks*2/3)
      {
        index = maxTicks*2/3;
      }
    }

    return index;

  }

  // The Driver is responsible for robot movement
  // Their joystick will control
  //   Drivetrain, Leg Raising/Lowering, Grabber Open/Close
  //   AutoAim, AutoBalance
  public void joyDriver()
  {
    if(joyDriver.getRawButton(A_BUTTON))
    {
      // Automatically adjust Yaw and Distance to current Limelight pipeline Target
      autoAim(txEntry.getDouble(0.0), tyEntry.getDouble(0.0));
    }
    else if(joyDriver.getRawButton(B_BUTTON))
    {
      // Automatically move robot to try and obtain ~0 Roll
      balance();
    }
    else if(joyDriver.getRawButton(RT_BUTTON))
    {
      // Open Grabber
      grabberSolenoid.set(true);
    }
    else if(joyDriver.getRawButton(LT_BUTTON))
    {
      // Close Grabber
      grabberSolenoid.set(false);
    }
    else if(joyDriver.getRawButton(LB_BUTTON))
    {
      liftLeg();
    }
    else if(joyDriver.getRawButton(RB_BUTTON))
    {
      lowerLeg();
    }
    else
    {
      stopLeg();
    }
  }

  // The Spotter is responsible for Autonomous Setup and Target Aquisition
  // Their joystick will control
  //   Autonomous Profile Selection, Pipeline Selection
  public void joySpotter()
  {
    if(joySpotter.getRawButton(1)) // Trigger Button
    {
      // If Spotter notices low battery or brownout then disable compressor
      if(myCompressor.isEnabled())
      {
        myCompressor.disable();
      }
      else myCompressor.enableDigital();
    }
    else if(joySpotter.getRawButton(2))
    {
      // Easy Auto Selection
      autoSelectionName = "Easy";
    }
    else if(joySpotter.getRawButton(3))
    {
      // Medium Auto Selection
      autoSelectionName = "Medium";
    }
    else if(joySpotter.getRawButton(4))
    {
      // Hard Auto Selection
      autoSelectionName = "Hard";
    }
    else if(joySpotter.getRawButton(5))
    {
      // The Absolute DREAM Auto Selection
      autoSelectionName = "Dream";
    }
    else if(joySpotter.getRawButton(10))
    {
      // AprilTag Crosshair Calibrated for loading zone
      table.getEntry("pipeline").setNumber(TAGS_LOAD);
    }
    else if(joySpotter.getRawButtonPressed(9))
    {
      // AprilTag Crosshair Calibrated for Scoring Pipeline
      table.getEntry("pipeline").setNumber(TAGS_SCORE);
    }
    else if(joySpotter.getRawButton(8))
    {
      // Retro-Tape for Scoring Pipeline
      table.getEntry("pipeline").setNumber(RETRO);
    }
    else if(joySpotter.getRawButton(7))
    {
      // Cone Pipeline
      table.getEntry("pipeline").setNumber(CONE);
    }
    else if(joySpotter.getRawButton(6))
    {
      // Cube Pipeline
      table.getEntry("pipeline").setNumber(CUBE);
    }
    else // No Assigned Button/Axis
    {
      // Do Nothing
    }
  }

  public void liftLeg()
  {
    valveServo.setAngle(OPEN_ANGLE);
    legSolenoid.set(true);

  }

  public void lowerLeg()
  {
    valveServo.setAngle(OPEN_ANGLE);
    legSolenoid.set(false);

  }

  public void stopLeg()
  {
    valveServo.setAngle(CLOSE_ANGLE);
    legSolenoid.set(false);
  }

  public double getSettingFromCurve(double x)
  {
    return (1/(1+Math.pow(Math.E, acceleration_curvature*(x - acceleration_midpoint))));
  }

  public void populateArrayList()
  {
    /*
     * This is a custom acceleration curve based on the Logistic Function
     * because I couldn't get PID tuning to work with REV kSmartVelocity.
     * This will likely seem like much more work to people who know how PID 
     * tuning works with the SparkMax SmartMotion system but this was easier
     * for me at the time of writing.
     * 
     * The refresh rate for periodic functions is defaulted to 20ms which I 
     * will refer to as 1 "tick" and all functions and the curvature coefficient
     * are based on this unit
     * 
     * The idea is that there are discrete motor settings during both positive and
     * negative acceleration that follow the logistics curve.  These values will be calculated 
     * here and recalculated when changes are made to the curvature and midpoint variables
     * in the Dashboard.  They will be stored in the settingsList ArrayList such that:
     *      settingsList.get(0) = -1 (full reverse)
     *      settingsList.get(size/2) = 0 (full stop)
     *      settingsList.get(size - 1) = 1 (full forward)
     *      at other positions 1 through size-2 will follow the output of the function
     * 
     * The standard Logistics Function as I'm using it is:
     *      y = 1/(1+e^(acceleration_curvature)(x-acceleration_midpoint))
     *      
     *      where y = the motor setting between 0 and 1 which can be modified to be + or - based on direction
     *      and x = the index of the ticks array
     */
    
    settingsList  = new ArrayList<Double>();

    // Populate "empty" List to avoid null pointers
    for(int i = 0; i < maxTicks; i++)
    {
      settingsList.add(0.0);
    }

    // Set values for endpoints
    settingsList.set(0, -1.0);
    settingsList.set(settingsList.size() - 1, 1.0);

    // Set remaining values based on Curve Function
    int k = 1;
    for(int i = settingsList.size() / 2 + 1; i < settingsList.size(); i++)
    {
      settingsList.set(i, getSettingFromCurve(k));
      k++;
    }
    
    for(int i = 1; i < settingsList.size() / 2; i++)
    {
      settingsList.set(i, -getSettingFromCurve(k));
      k--;
    }

    // // Testing output
    // double[] temp = new double[settingsList.size()];
    // for(int i = 0; i < settingsList.size(); i++)
    // {
    //   temp[i] = settingsList.get(i).doubleValue();
    // }
    // SmartDashboard.putNumberArray("settingsArray", temp);
  }
  
  
  /*
   * END OF PROGRAM
   * 
   * The remaining methods included for completeness but not currently implemented
   */
  @Override
  public void autonomousInit() 
  {
    // Empty
  }
  
  @Override
  public void teleopInit() 
  {
    // Empty
  }

  @Override
  public void disabledInit() 
  {
    //Empty
  }


  @Override
  public void disabledPeriodic() 
  {
    //Empty
  }


  @Override
  public void testInit() 
  {
    //Empty
  }


  @Override
  public void testPeriodic() 
  {
    //Empty
  }


  @Override
  public void simulationInit() 
  {
    //Empty
  }


  @Override
  public void simulationPeriodic() 
  {
    //Empty
  }

  protected void execute() 
  {
    // Empty
  }
}
