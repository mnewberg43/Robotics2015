package org.usfirst.frc.team291.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	DoubleSolenoid	arm_pneumatics ,
					back_pneumatics ;
	RobotDrive myRobot;
	Joystick stick;
	Joystick stick2;
	Talon	armLeft ;
	Talon	armRight ;
	CANTalon 	frontLeftChannel ,
				frontRightChannel ,
				rearLeftChannel ,
				rearRightChannel ;
	int autoLoopCounter;
	double angle = 0 ,
			startAngle = 0 ,
			rotation = 0 ,
			armCommand = 0 ,
			armCommandScale = .8 ;
	double ycomm = 0 ;
	double rotSpeedScale = 1.75 ,
			maxYComm = .7 ,
			maxXComm = .7 ;
	double timeCounter = 0 ,
			timeCounterMax = 12 ;
	double lastRotCommand = 0 ;
	double	xcommAuto = 0 ,
			ycommAuto = 0 ,
			rotcommAuto = 0 ;
	double xcomm = 0;
	int x = 0 , y = 0 ;
	int session;
	int gyroCounter = 0 ;
    Image frame;
    Image temp_frame;
    AxisCamera camera;
    DigitalOutput blinkyLight ;
    Gyro gyro1;
    boolean newblink,
    		oldblink,
    		blinkstatus,
    		gyronogyro ,
    		keepHeading = false ,
    		oldKeepHeading = false;
    BuiltInAccelerometer accel ;
    Timer	accel_time ;
    double	curr_time ;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	stick = new Joystick(0);
    	stick2 = new Joystick(1);
    	frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
    	gyro1 = new Gyro(0);
        // open the camera at the IP address assigned. This is the IP address that the camera
        // can be accessed through the web interface.
        camera = new AxisCamera("10.2.91.11");
        LiveWindow.addSensor("Gyro", 0, gyro1);
        accel_time = new Timer() ;
        frontRightChannel = new CANTalon(4);
        frontLeftChannel = new CANTalon(3);
        rearRightChannel = new CANTalon(1);
        rearLeftChannel = new CANTalon(2);
    	myRobot = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	myRobot.setInvertedMotor(MotorType.kFrontRight, true);	// invert the left side motors
    	myRobot.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
    	arm_pneumatics = new DoubleSolenoid(0,1);
    	back_pneumatics = new DoubleSolenoid(2,3);
    	armLeft = new Talon(1);
    	armRight = new Talon(0);
    	accel = new BuiltInAccelerometer(Accelerometer.Range.k2G);
    	accel_time.reset();
    	curr_time = 0 ;
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 200) //Check if we've completed 100 loops (approximately 2 seconds)
		{ // Auto version 1: move bin to scoring zone
			// drive forwards half speed
			xcommAuto = 0.0 ;
    		ycommAuto = -0.50 ;
    		rotcommAuto = 0.0 ;
			autoLoopCounter++;
		} 
 //    	if (autoLoopCounter < 110)
//    	if (autoLoopCounter < 200) // Autonomous version 2: go to step
//    	{
//    		autoLoopCounter++ ;
//    		xcommAuto = 0.0 ;
//    		ycommAuto = -.45 ;
//    		rotcommAuto = 0.0 ;
//    		if (autoLoopCounter > 25)
//    		{
//    			arm_pneumatics.set(DoubleSolenoid.Value.kReverse); // Close the arms after half a second
//    			
//    		}
//
//    	}
    	else
    	{
    		xcommAuto = 0.0 ;
    		ycommAuto = 0.0 ;
    		rotcommAuto = 0.0 ;
    	}
    	myRobot.mecanumDrive_Cartesian( xcommAuto, ycommAuto, rotcommAuto, 0);
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {

    	while (isOperatorControl() && isEnabled()) {
    		

    		xcomm = stick.getX();
    		if (xcomm > maxXComm)
    			{xcomm = maxXComm ;}
    		if (xcomm < -maxXComm)
    			{xcomm = -maxXComm ;}
    		if ((xcomm < .1) && (xcomm > -.1) )
    		{xcomm = 0 ;}
    		ycomm = stick.getY();
    		if (ycomm > maxYComm )
    			{ycomm = maxYComm ;}
    		if (ycomm < -maxYComm)
    		{ycomm = -maxYComm; }
    		if ((ycomm < .1) && (ycomm > -.1) )
    		{ycomm = 0 ;}
    	
    		rotation = stick.getRawAxis(4);

    		 if ((rotation > -.1) && (rotation <.1))
    		{
    			rotation = 0 ;
    		}
    		rotation = rotation / rotSpeedScale ;
			
    		angle = gyro1.getAngle() ;
    		
    		if (rotation == 0)
    		{
    			if (timeCounter >= timeCounterMax )
    			{
    				rotation = -( (angle-startAngle) / 60 ) ;
    			}
    			else
    			{
    				rotation = 0 ;
    				timeCounter += 1 ;
    				startAngle = angle + 24*lastRotCommand ;
    			}
    			
    		}
    		else
    		{
    			lastRotCommand = rotation; 
    			timeCounter = 0 ;
    		}
    		
    		myRobot.mecanumDrive_Cartesian( xcomm, ycomm, rotation, 0);

    		if ( stick2.getRawButton(1) )
    		{
    			arm_pneumatics.set(DoubleSolenoid.Value.kForward);
    		}
    		else if (stick2.getRawButton(2))
    		{
    			arm_pneumatics.set(DoubleSolenoid.Value.kReverse);
    		}
    		else 
    		{
    			arm_pneumatics.set(DoubleSolenoid.Value.kOff);
    		}
    		
    		if (stick2.getRawButton(5))
    		{
    			back_pneumatics.set(DoubleSolenoid.Value.kForward);
    		}
    		else if (stick2.getRawButton(6))
    		{
    			back_pneumatics.set(DoubleSolenoid.Value.kReverse);
    		}
    		else
    		{
    			back_pneumatics.set(DoubleSolenoid.Value.kOff); 
    		}
    		

    		armCommand = stick2.getY() ;
    		
    		if ( (armCommand < .2) && (armCommand > -.2) )
    		{
    			armCommand = 0 ;

    		}
    		armCommand = armCommand * armCommandScale ;
    		
    		if (armCommand == 0 )
    		{
    			if (stick2.getRawButton(5))
        		{
    				armLeft.set(-.25);
        		}
        		else if (stick2.getRawButton(6))
        		{
        			armRight.set(-.25);
        		}
        		else
        		{
        			armLeft.set(0);
        			armRight.set(0); 
        		}
    		}
    		else
    		{
    			if (armCommand < 0 )
    			{
    				armLeft.set(armCommand*-1.055);
    			}
    			else
    			{
    				armLeft.set(armCommand*-1.02) ;	
    			}
    			
    			armRight.set(armCommand*-1) ;
    		}
    		
	    
	    	LiveWindow.run();

    	}
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	
    	curr_time = accel_time.get();
    	
    	SmartDashboard.putNumber("time Counter", curr_time);
    	if ( (armCommand < .2) && (armCommand > -.2) )
		{
			armCommand = 0 ;

		}
		armCommand = armCommand * armCommandScale ;
		
		if (armCommand == 0 )
		{
			if (stick2.getRawButton(5))
    		{
				armLeft.set(-.25);
    		}
    		else if (stick2.getRawButton(6))
    		{
    			armRight.set(-.25);
    		}
    		else
    		{
    			armLeft.set(0);
    			armRight.set(0); 
    		}
		}
		else
		{
			if (armCommand < 0 )
			{
				armLeft.set(armCommand*-1.055);
			}
			else
			{
				armLeft.set(armCommand*-1.02) ;	
			}
			
			armRight.set(armCommand*-1) ;
		}
    	

    	LiveWindow.run();
    	
    	myRobot.mecanumDrive_Cartesian( 0, 0, 0, 0);
    }
	
	public double joystickTransferFunction(double input, double deadbandMag, double maxOutput) {
	// This is going to take an input from -1.0 to 1.0 and
	// provide an output that will be used by the robot drive.
	
		double	output ,
				slope ,
				offset ;
		slope = maxOutput / (1 - deadbandMag) ;
		offset = slope * deadbandMag ;
		if (input < (-1.0) )
		{
			input = -1.0 ;
		}
		
		if (input > 1.0)
		{
			input = 1.0 ;
		}
		
		if (input < (-deadbandMag))
		{
			output = (input * slope) + offset ;
		}
		else if (input < (deadbandMag) )
		{
			output = 0.0 ;
		}
		else
		{
			output = (input * slope) - offset ;
		}
	
	
	
	}
	
    
}
