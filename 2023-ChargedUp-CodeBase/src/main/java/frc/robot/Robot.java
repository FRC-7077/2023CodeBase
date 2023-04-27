package frc.robot;

// import for use with Acceleration Curve system
import java.util.ArrayList;

// Standard imports from WPILib for hardware and SmartDashboard
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Servo;

// Rev Robotics imports for SparkMax controllers
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Robot extends TimedRobot
{
  /*
   * The order of these methods is so that the commonly adjusted methods are up top
   * and the methods you should never need to adjust are below them to reduce scrolling
   */
  

   // ******************  AUTONOMOUS CODE  **************************************************************************
  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}


  // **************  SMARTDASHBOARD CODE  **************************************************************************
  
  public void robotPeriodic() 
  {
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
    if(joyDriver.getRawButton(3))
    {
      unlockLeg();
      liftLeg();
    }
    else if(joyDriver.getRawButton(4))
    {
      lockLeg();
      lowerLeg();
    }
    else if(joyDriver.getRawButton(7))
    {
      grabberSolenoid.set(false);
    }
    else if(joyDriver.getRawButton(8))
    {
      grabberSolenoid.set(true);
    }
    else
    {
      lockLeg();
      stopLeg();
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

    // Enable the compressor
    myCompressor.enableDigital();
  }

  @Override
  public void teleopInit() 
  {
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
  
    double left = clamp((fowardBackwardAxis + twistAxis), -1, 1);
    double right = clamp((fowardBackwardAxis - twistAxis), -1, 1);
  
    updateMotorSetting(left, right);

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
    double desired_setpoint_left = 0;
    double desired_setpoint_right = 0;

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
  private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushed);
  private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushed);
  private CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushed);
  private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushed);

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
  
  // Joysticks
  private Joystick joyDriver = new Joystick(0);

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
 

  // Compressor Enable/Disable variable
  private boolean compressorState = true;

}





















