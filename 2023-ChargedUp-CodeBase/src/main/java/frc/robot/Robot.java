package frc.robot;

// import for use with Acceleration Curve system
import java.util.ArrayList;

// import for the navX2-micro 9-axis Gyro
import com.kauailabs.navx.frc.AHRS;

// Standard imports from WPILib for hardware and SmartDashboard
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;

// Rev Robotics imports for SparkMax controllers
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

// WPILib imports for NetworkTable access with Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot
{
  /*
   * The order of these methods is so that the commonly adjusted methods are up top
   * and the methods you should never need to adjust are below them to reduce scrolling
   */
  

   // ******************  AUTONOMOUS CODE  **************************************************************************
  @Override
  public void autonomousInit() 
  {
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);

    // Reset Encoder Positions to zero
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    grabberSolenoid.set(true);
  }

  @Override
  public void autonomousPeriodic() 
  {
    double pos = 0.0;
    double error = 0.0;
    

// Switch Statement based on Autonomous Selected
    switch (autoSelectionName)
    {
      // Each Autonomous Case will use another switch statement to
      // handle the stages of the program
      case "Easy":
        switch (stage)
        {
          case 0: // Release the bungee to slap the cube
            slapTheCube();
            Timer.delay(3);
            stage = 1;
            break;

          case 1: // Drive to position "pos"
            pos = -16; // This is in feet...negative values move the robot forward
            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            break;

          default:
            break;
        }  
        break;
      
      case "Hard-Balance":
        switch (stage)
        {
          case 0: // Release the bungee to slap the cube
            slapTheCube();
            Timer.delay(3);
            stage = 1;
            break;

          case 1: // Drive to position "pos"
            pos = -15; // This is in feet
            error = pos - leftEncoder.getPosition();

            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) > 0.1 ? 1 : 2;
            break;

          case 2:
            if(navX2.getYaw() < 91 && navX2.getYaw() > 89)
            {
              Timer.delay(1);
              if(navX2.getYaw() < 91 && navX2.getYaw() > 89)
              {
                stage = 3;
                isRotating = false;
                leftEncoder.setPosition(0.0);
                rightEncoder.setPosition(0.0);
              }
            }
            else
            {
              targetYaw = 90;
              isRotating = true;
            }
          
          case 3: // Drive back to Charge Station
            pos = 3; // This is in feet
            error = pos - (leftEncoder.getPosition() + rightEncoder.getPosition())/2;

            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) > 0.1 ? 3 : 4;
            if(stage == 3){ Timer.delay(2);}
            break;

          case 4:
            if(navX2.getYaw() < -179 && navX2.getYaw() > 179)
            {
              Timer.delay(1);
              if(navX2.getYaw() < -179 && navX2.getYaw() > 179)
              {
                stage = 5;
                isRotating = false;
                leftEncoder.setPosition(0.0);
                rightEncoder.setPosition(0.0);
              }
            }
            else
            {
              targetYaw = 180;
              isRotating = true;
            }
          
            case 5: // Drive back to Charge Station
            pos = 3; // This is in feet
            error = pos - (leftEncoder.getPosition() + rightEncoder.getPosition())/2;

            leftPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
            rightPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);

            stage = Math.abs(error) > 0.1 ? 5 : 6;
            if(stage == 5){ Timer.delay(2);}
            break;
            
          case 6: // balance
            isBalancing = true;
            break;

          default:
            break;
        }
        break;

      default:
        DriverStation.reportError("Error Selecting Autonomous:  Selected Value is:  " + autoSelectionName, true);
        break;
    }
  }


  // **************  SMARTDASHBOARD CODE  **************************************************************************
  
    public void robotPeriodic() 
  {
    // Check Spotter Joystick for input so SPotter can modify Autonomous Setup
    joySpotter();
    //joyBackup();

    // Update Target Yaw to Current Yaw unless the robot is told to rotate
    if(isRotating)
      autoRotateToAngle(targetYaw);

    // Balance the robot
    if(isBalancing)
      balance();

    // Show which Autonomous is Selected
    SmartDashboard.putString("Auto Selected", autoSelectionName);

    // LegLock Limit Switch State
    SmartDashboard.putBoolean("LegLockSwitch", legLocked.get());

    // Toggle for Compressor
    compressorState = SmartDashboard.getBoolean("Compressor", compressorState);

    // I use the negation so that enabled is the fail state for safety purposes
    if(!compressorState)
    {
      myCompressor.disable();
    }
    else
    {
      myCompressor.enableDigital();
    }
    SmartDashboard.putBoolean("Compressor", compressorState);  

  }

  
  // *********************** JOYSTICK/CONTROLLER CODE ***************************************************************

  // The Driver is responsible for robot movement
  // Their joystick will control
  //   Drivetrain, Leg Raising/Lowering, Grabber Open/Close
  //   AutoAim, AutoBalance
  public void joyDriver()
  {
    if(joyDriver.getRawButton(2))
    {
      // Automatically adjust Yaw and Distance to current Limelight pipeline Target
      autoAim(txEntry.getDouble(0.0), tyEntry.getDouble(0.0));
    }
    else if(joyDriver.getRawButton(3))
    {
      unlockLeg();
      liftLeg();
    }
    else if(joyDriver.getRawButton(4))
    {
      lockLeg();
      lowerLeg();
    }
    else if(joyDriver.getRawButton(5))
    {
      // Rotates Directly to the left spinning Counter-Clockwise while holding button
      targetYaw = -90;
      isRotating = true;
    }
    else if(joyDriver.getRawButton(6))
    {
      // Rotates Directly to the right spinning Clockwise while holding button
      targetYaw = 90;
      isRotating = true;
    }
    else if(joyDriver.getRawButton(7))
    {
      grabberSolenoid.set(false);
    }
    else if(joyDriver.getRawButton(8))
    {
      grabberSolenoid.set(true);
    }
    else if(joyDriver.getRawButton(9))
    {
      slapReleaseServo.setAngle(LOCK_ANGLE);
    }
    else
    {
      lockLeg();
      stopLeg();
      isRotating = false;
      //needBackup = false;
    }
  }

  // The Spotter is responsible for Autonomous Setup and Target Aquisition
  // Their joystick will control
  //   Autonomous Profile Selection, Pipeline Selection
  public void joySpotter()
  {
    if(joySpotter.getRawButton(1)) // Trigger
    {
      // Re-Calibrate NavX2 Gyro
      navX2.reset();
      Timer.delay(5);
    }
    else if(joySpotter.getRawButton(2)) // Thumb button down
    {
      // Easy Auto Selection
      autoSelectionName = "Easy";
    }
    else if(joySpotter.getRawButton(3)) // Thumb button up
    {
      // Hard-Balance Auto Selection
      autoSelectionName = "Hard-Balance";
    }
    else if(joySpotter.getRawButton(4)) // Thumb button left
    {
      // Hard-Multiple Auto Selection
      autoSelectionName = "Hard-Multiple";
    }
    else if(joySpotter.getRawButton(6)) // Base Button Left Side Top
    {
      // Target Cube on High Loading Area Pipeline
      table.getEntry("pipeline").setNumber(CONE);
    }
    else if(joySpotter.getRawButton(7)) // Base Button Left Side Bottom
    {
      // Target Cube on High Loading Area Pipeline
      table.getEntry("pipeline").setNumber(CUBE);
    }
    else if(joySpotter.getRawButtonPressed(8)) // Base Buttons Middle Right
    {
      // AprilTag Crosshair Calibrated for Scoring Cones to LEFT
      table.getEntry("pipeline").setNumber(SCORE_CUBE);      
    }
    else if(joySpotter.getRawButtonPressed(9)) // Base Buttons Right side bottom
    {
      // AprilTag Crosshair Calibrated for Scoring Cubes
      table.getEntry("pipeline").setNumber(LOAD_LOW);
    }
    else // No Assigned Button/Axis
    {
      // Do Nothing
    }
  }
  
  
    @Override
  public void robotInit() 
  {

// Initialize Acceleration Curve settings
    // Logistic Curve values
    // acceleration_curvature = -0.1;
    acceleration_midpoint = 50;
    populateArrayList();

    // Reset controller settings and set Idle Mode to Brake
    // Based on advice from Brandon at Cimarron we also limit
    // the current draw of the Neos to prevent brownout
    leftMotor1.restoreFactoryDefaults();
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor1.setSmartCurrentLimit(80);

    leftMotor2.restoreFactoryDefaults();
    leftMotor2.setIdleMode(IdleMode.kCoast);
    leftMotor2.setSmartCurrentLimit(80);

    rightMotor1.restoreFactoryDefaults();
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor1.setSmartCurrentLimit(80);

    rightMotor2.restoreFactoryDefaults();
    rightMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor2.setSmartCurrentLimit(80);

    // Set secondary motors on each side as followers
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    // Set left side motors inverted so that we can use the same 
    // numbers for both sides without worrying about signs
    leftMotor1.setInverted(true);

    // Set Encoder Conversion such that position is in feet (ft) and 
    // velocity is in feet per second (ft/s)
    // Gear Teeth based on our setup of the Toughbox Mini
    double encoderGearTeeth = 14;
    double gear1OuterTeeth = 50;
    double gear1InnerTeeth = 16;
    double gear2OuterTeeth = 48;
    gearRatio = ((encoderGearTeeth/gear1OuterTeeth)*gear1InnerTeeth)/gear2OuterTeeth; // Number of Output shaft revolutions per One Encoder shaft revolution
    wheelDiameter = 0.5; // feet
    encoderRotationsToFeetConversionFactor = gearRatio * Math.PI * wheelDiameter;
    leftEncoder.setPositionConversionFactor(encoderRotationsToFeetConversionFactor);
    rightEncoder.setPositionConversionFactor(encoderRotationsToFeetConversionFactor);

    // Enable the compressor
    myCompressor.enableDigital();

    // Prep Slap Servo
    slapReleaseServo.setAngle(UNLOCK_ANGLE);
    
    // Setup SmartMotion
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setIZone(kIz);
    leftPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);

    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
    rightPID.setIZone(kIz);
    rightPID.setFF(kFF);
    rightPID.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    leftPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    leftPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    leftPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    leftPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    rightPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    rightPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    rightPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    rightPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

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
    navX2.reset();

   
  }

  @Override
  public void teleopInit() 
  {
    isBalancing = false;
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
  }  
  
  @Override
  public void teleopPeriodic() 
  {
    joyDriver();

    double fowardBackwardAxis = joyDriver.getRawAxis(1);
    double twistAxis = -joyDriver.getRawAxis(2);

    // Set deadbands
    fowardBackwardAxis = fowardBackwardAxis < 0.05 && fowardBackwardAxis > -0.05 ? 0 : fowardBackwardAxis/2;
    twistAxis = twistAxis < 0.2 && twistAxis > -0.2 ? 0 : twistAxis/2;
  
    double left = clamp((fowardBackwardAxis + twistAxis + steering_adjust - distance_adjust), -1, 1);
    double right = clamp((fowardBackwardAxis - twistAxis - steering_adjust - distance_adjust), -1, 1);
  
    updateMotorSetting(left, right);
    
    steering_adjust = 0.0;
    distance_adjust = 0.0;
  }

  /*
   * Method that takes a desired Yaw angle and uses the AutoAim method
   * to rotate the robot to that desired Yaw Angle
   * 
   * NOTE: The angle should be between -180 and 180 since the NavX2-Micro
   *       returns Yaw values in that range
   */
  public void autoRotateToAngle(double angle)
  {
    double currentYaw = navX2.getYaw();
    autoAim(angle - currentYaw, 0.0);
  }

  /*
   * Method that takes a Yaw Error (tx) and Roll Error (ty) and sets adjustment variables
   * to bring robot to a position that will reduce those errors to zero
   */
  public void autoAim(double tx, double ty)
  {
    double KpAim = 0.01;
    double KpDistance = 0.01;
    double min_aim_command = 0.01;

    double heading_error = -tx;
    double distance_error = -ty;

    heading_error = heading_error > 25 ? 25 : heading_error;
    heading_error = heading_error < -25 ? -25 : heading_error;

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

  /* 
   * Method that sets the distance adjustment variable based
   * on the Pitch angle from the NavX2-Micro so that it will 
   * roll forward if it is leaning back and vice versa
   */
  public void balance()
  {
    double roll_error = -navX2.getRoll();

    // Set Adjustment for Pitch Correction
    distance_adjust = Math.abs(roll_error) > 10 ? roll_error / 180 : 0;
  }


  /*
   * This method moves the motor through the indices of the
   * ArrayList that holds the pre-calculated motor settings along
   * the Logistics Curve  The setpoint is the desired setpoint
   * and the index should be the current index of the setting for
   * the motor being updated
   */
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

    return index;

  }
  /*
   * Use this method to actually move the robot based on the current
   * values of steering_adjust and distance_adjust and Controller axis
   */
  public void updateMotorSetting(double joyModX, double joyModY)
  {
    double left = 0.0;
    double right = 0.0;
    double desired_setpoint_left = steering_adjust + distance_adjust;
    double desired_setpoint_right = -steering_adjust + distance_adjust;

    // Get Desired Setpoints based on Stick 
    if(!joyDriver.getRawButton(2))
    {
      desired_setpoint_left += (double)((int)(joyModX*10000))/10000;
      desired_setpoint_right += (double)((int)(joyModY*10000))/10000;
    }

    currentIndexLeft = updateMotorIndex(desired_setpoint_left, currentIndexLeft);
    currentIndexRight = updateMotorIndex(desired_setpoint_right, currentIndexRight);

    left = settingsList.get(currentIndexLeft);
    right = settingsList.get(currentIndexRight);
    
    leftMotor1.set(left);
    rightMotor1.set(right);
    steering_adjust = 0.0;
    distance_adjust = 0.0;
  }

  public void liftLeg()
  {
    valveServo.setAngle(OPEN_ANGLE);
    legSolenoid.set(false);

  }

  public void lowerLeg()
  {
    valveServo.setAngle(OPEN_ANGLE);
    legSolenoid.set(true);

  }

  public void stopLeg()
  {
    valveServo.setAngle(CLOSE_ANGLE);
    legSolenoid.set(true);
  }
  
  public void lockLeg()
  {
    legReleaseServo_1.setAngle(UNLOCK_ANGLE);
    legReleaseServo_2.setAngle(LOCK_ANGLE);
  }

  public void unlockLeg()
  {
    legReleaseServo_1.setAngle(LOCK_ANGLE);
    legReleaseServo_2.setAngle(UNLOCK_ANGLE);
  }

  public void slapTheCube()
  {
    slapReleaseServo.setAngle(LOCK_ANGLE);
  }

  public double getSettingFromCurve(double x)
  {
    return -0.5*Math.cos(Math.PI/acceleration_midpoint*x) + 0.5;
    //return (1/(1+Math.pow(Math.E, acceleration_curvature*(x - acceleration_midpoint))));
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
  
  public double clamp(double val, double min, double max) 
  {
    if(val < min) { return min; }
    if(val > max) { return max; }
    return val;
  }


    
  // Motors and Drive Declaration/Insantiation
  private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private RelativeEncoder leftEncoder = leftMotor1.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor1.getEncoder();
  private SparkMaxPIDController leftPID = leftMotor1.getPIDController();
  private SparkMaxPIDController rightPID = rightMotor1.getPIDController();

  // Limit Switch(es) Declaration/Instantiation
  private DigitalInput legLocked = new DigitalInput(0);
  
  // Acceleration Curve
  // Two seconds worth of ticks at 20ms per tick is 100, 
  // Double that for positive and negative acceleration
  // Finally, add one for full stop...that's 201 ticks
  // Adjustments will impact the acceleration curve so do 
  // so only if you also refactor the Curve function itself

  private int maxTicks = 101;
  private int currentIndexLeft = maxTicks / 2;
  private int currentIndexRight = maxTicks / 2;
  private ArrayList<Double> settingsList;
  private double acceleration_midpoint;//, acceleration_curvature;
  
  // nav-X Micro
  private AHRS navX2;

  // Joysticks
  private Joystick joyDriver = new Joystick(0);
  private Joystick joySpotter = new Joystick(1);

  // Pipeline Constants
  private final int CONE = 0; // Retroreflective Threshold for Cones crosshair at bottom to put object in grabber
  private final int CUBE = 1; // Retroreflective Threshold for Cubes crosshair at bottom to put object in grabber
  private final int LOAD_LOW = 2; // AprilTag crosshair centered at height to represent distance for low loading zone
  private final int SCORE_CUBE = 3; // AprilTag centered at height to represent distance for scoring on middle level cube area


  // Servo Angle Constants
  private final double OPEN_ANGLE = 120.0;
  private final double CLOSE_ANGLE = 30.0;
  private final double UNLOCK_ANGLE = 0;
  private final double LOCK_ANGLE = 180;

  
  // Robot Hardware
  Compressor myCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Solenoid grabberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  Solenoid legSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  Servo valveServo = new Servo(0);
  Servo legReleaseServo_1 = new Servo(1);
  Servo legReleaseServo_2 = new Servo(2);
  Servo slapReleaseServo = new Servo(3);
  
  
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

  // Encoder Variables
  private double gearRatio, wheelDiameter, encoderRotationsToFeetConversionFactor;

  // Rotation with navX2 Variables
  private double targetYaw;
  private boolean isRotating = false;
  private boolean isBalancing = false;


  // Compressor Enable/Disable variable
  private boolean compressorState = true;

  // Autonomous Stage Tracking Array
  private int stage = 0;

  // PID Variables for DriveToPosition Method
  // Setup PID for SmartMotion
  private double kP = 5e-5;
  private double kI = 1e-6;
  private double kD = 0.0;
  private double kIz = 1;
  private double kFF = 0.000256; // Aggressive Acceleration...for more moderate acceleration use 0.000156
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxVel = 5700;
  private double minVel = 0;
  private double maxAcc = 1500;
  private double allowedErr = 0.1; // This is in Feet so our error should be around 1.2 inches +/- but usually -
}





















