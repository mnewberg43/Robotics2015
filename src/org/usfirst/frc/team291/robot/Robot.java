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
	public static final boolean usegyro = true,
								nousegyro = false;
	
	 
	DoubleSolenoid	arm_pneumatics ,
					back_pneumatics ;
	RobotDrive myRobot;
	Joystick stick;
	Joystick stick2;
	Talon	armLeft ;
	Talon	armRight ;
//	final int frontLeftChannel	= 2;
	CANTalon 	frontLeftChannel ,
				frontRightChannel ,
				rearLeftChannel ,
				rearRightChannel ;
//    final int rearLeftChannel	= 3;
//    final int frontRightChannel	= 1;
//    final int rearRightChannel	= 0;
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
    double	xAccelOffset = 0 ,
    		yAccelOffset = 0 ,
    		zAccelOffset = 0 ;
    double	xAccel[] ,
    		yAccel[] ,
    		zAccel[] ;
    double	xSpeed[] ,
    		ySpeed[] ,
    		zSpeed[] ;
    double	xDisp[] ,
    		yDisp[] ,
    		zDisp[] ;
    Timer	accel_time ;
    double	curr_time ;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);
    	stick2 = new Joystick(1);
    	blinkstatus = true;
    	blinkyLight = new DigitalOutput(0);
    	blinkyLight.set(blinkstatus);
//    	myRobot = new RobotDrive(0, 1, 2, 3);
    	frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
    	oldblink = false;
    	newblink = false;
    	gyro1 = new Gyro(0);
        // open the camera at the IP address assigned. This is the IP address that the camera
        // can be accessed through the web interface.
        camera = new AxisCamera("10.2.91.11");
        LiveWindow.addSensor("Gyro", 0, gyro1);
        //LiveWindow.
        accel_time = new Timer() ;
        frontRightChannel = new CANTalon(4);
        frontLeftChannel = new CANTalon(3);
        rearRightChannel = new CANTalon(1);
        rearLeftChannel = new CANTalon(2);
    	myRobot = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	myRobot.setInvertedMotor(MotorType.kFrontRight, true);	// invert the left side motors
    	myRobot.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
    	gyronogyro = nousegyro;
    	arm_pneumatics = new DoubleSolenoid(0,1);
    	back_pneumatics = new DoubleSolenoid(2,3);
    	armLeft = new Talon(1);
    	armRight = new Talon(0);
    	accel = new BuiltInAccelerometer(Accelerometer.Range.k2G);
    	xAccel = yAccel = zAccel = new double[] { 0 , 0 , 0 } ;
    	xSpeed = ySpeed = zSpeed = new double[] { 0 , 0 , 0 } ;
    	xDisp = yDisp = zDisp = new double[] { 0 , 0 , 0 } ;
    	xAccelOffset = accel.getX();
    	yAccelOffset = accel.getY();
    	zAccelOffset = accel.getZ();
    	accel_time.reset();
    	curr_time = 0 ;
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
//    	xAccelOffset = accel.getX();
//    	yAccelOffset = accel.getY();
//    	zAccelOffset = accel.getZ();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
//    	if(autoLoopCounter < 200) //Check if we've completed 100 loops (approximately 2 seconds)
//		{
//			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
//			autoLoopCounter++;
//			} else {
//			myRobot.drive(0.0, 0.0); 	// stop robot
//		}
    	
//    	if (autoLoopCounter < 110)
    	if (autoLoopCounter < 200)
    	{
    		//arm_pneumatics.set(DoubleSolenoid.Value.kForward);
    		autoLoopCounter++ ;
    		xcommAuto = 0.0 ;
    		ycommAuto = -.45 ;
    		rotcommAuto = 0.0 ;
    		if (autoLoopCounter > 25)
    		{
    			arm_pneumatics.set(DoubleSolenoid.Value.kReverse);
    			
    		}
    		else
    		{
//    			arm_pneumatics.set(DoubleSolenoid.Value.kForward);
    		}
    	}
    	else
    	{
    		xcommAuto = 0.0 ;
    		ycommAuto = 0.0 ;
    		rotcommAuto = 0.0 ;
//    		arm_pneumatics.set(DoubleSolenoid.Value.kForward);
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
//        myRobot.arcadeDrive(stick);
//    	accel_time.reset();
//    	startAngle = gyro1.getAngle();
//    	xAccel[0] = (accel.getX() - xAccelOffset) ;
//    	yAccel[0] = (accel.getY() - yAccelOffset) ;
//    	zAccel[0] = (accel.getZ() - zAccelOffset) ;
//    	
//    	xSpeed[0] = xSpeed[0] + ( xAccel[0] * 32 * .05) ; // v = v0 + (a) (t)
//    	ySpeed[0] = ySpeed[0] + ( yAccel[0] * 32 * .05) ;
//    	zSpeed[0] = zSpeed[0] + ( zAccel[0] * 32 * .05) ;	
//    	
//    	xDisp[0] = xDisp[0] + ( xSpeed[0] * .05 ) ; // x = x0 + v*t
//    	yDisp[0] = yDisp[0] + ( ySpeed[0] * .05 ) ;
//    	zDisp[0] = zDisp[0] + ( zSpeed[0] * .05 ) ;
//    	
//    	SmartDashboard.putNumber("xAccel", xAccel[0] );
//    	SmartDashboard.putNumber("yAccel", yAccel[0] );
//    	SmartDashboard.putNumber("zAccel", zAccel[0] );
//    	
//    	SmartDashboard.putNumber("xSpeed", xSpeed[0] );
//    	SmartDashboard.putNumber("ySpeed", ySpeed[0] );
//    	SmartDashboard.putNumber("zSpeed", zSpeed[0] );
//    	
//    	SmartDashboard.putNumber("xDisp", xDisp[0] );
//    	SmartDashboard.putNumber("yDisp", yDisp[0] );
//    	SmartDashboard.putNumber("zDisp", zDisp[0] );
//    	NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
    	while (isOperatorControl() && isEnabled()) {
    		
//    		curr_time = accel_time.get();
//    		xAccel[0] = (accel.getX() - xAccelOffset) ;
//        	yAccel[0] = (accel.getY() - yAccelOffset) ;
//        	zAccel[0] = (accel.getZ() - zAccelOffset) ;
//        	
//        	xSpeed[0] = xSpeed[0] + ( xAccel[0] * 32 * curr_time) ; // v = v0 + (a) (t)
//        	ySpeed[0] = ySpeed[0] + ( yAccel[0] * 32 * curr_time) ; 
//        	zSpeed[0] = zSpeed[0] + ( zAccel[0] * 32 * curr_time) ;	
//        	/* very simplistic integration since I didn't have access to Gabe's
//    			code for the more awesome integration. */
//        	xDisp[0] = xDisp[0] + ( xSpeed[0] * curr_time ) ; // x = x0 + v*t
//        	yDisp[0] = yDisp[0] + ( ySpeed[0] * curr_time ) ;
//        	zDisp[0] = zDisp[0] + ( zSpeed[0] * curr_time ) ;
//        	
//        	SmartDashboard.putNumber("xAccel", xAccel[0] );
//        	SmartDashboard.putNumber("yAccel", yAccel[0] );
//        	SmartDashboard.putNumber("zAccel", zAccel[0] );
//        	
//        	SmartDashboard.putNumber("xSpeed", xSpeed[0] );
//        	SmartDashboard.putNumber("ySpeed", ySpeed[0] );
//        	SmartDashboard.putNumber("zSpeed", zSpeed[0] );
//        	
//        	SmartDashboard.putNumber("xDisp", xDisp[0] );
//        	SmartDashboard.putNumber("yDisp", yDisp[0] );
//        	SmartDashboard.putNumber("zDisp", zDisp[0] );
//        	accel_time.reset();
        	
//    		if (stick.getRawButton(5) )
//    		{
//    			gyronogyro = usegyro ;
//    		}
//    		if (stick.getRawButton(6))
//    		{
//    			gyronogyro = nousegyro ;
//    		}
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
    		
//    		keepHeading = stick.getRawButton(8);
//			angle = gyro1.getAngle();
//			if (keepHeading)
//			{
//				if (keepHeading != oldKeepHeading)
//				{
//					startAngle = angle ;
//				}
//				rotation = (stick.getRawAxis(4)- ( (angle-startAngle) / 45));
////				myRobot.mecanumDrive_Cartesian(-ycomm, -xcomm, (stick.getRawAxis(4)- ( (angle-startAngle) / 360)), -(angle));
//			}
//			else
//			{
//				rotation = stick.getRawAxis(4) ;
//
//			}
//			oldKeepHeading = keepHeading;
//    		
//    		if (gyronogyro != usegyro)
//    		{
//
//    		angle = 0;
//    			
//    		}
//    		angle = 0;
    		rotation = stick.getRawAxis(4);
//    		rotation = stick.getZ();
//    		if (rotation > .8)
//    		{
//    			rotation = .8 ;
//    		}
//    		else if (rotation < -.8)
//    		{
//    			rotation = -.8 ;
//    			
//    		}
    		 if ((rotation > -.1) && (rotation <.1))
    		{
    			rotation = 0 ;
    		}
    		rotation = rotation / rotSpeedScale ;
			
//    		myRobot.mecanumDrive_Cartesian( xcomm, ycomm, rotation, -angle);
//    		if (rotation != 0)
//    		{
//    			startAngle = angle ;
//    		}
//			startAngle = startAngle - rotation ;
//			if (gyroCounter >=5)
//			{
//				startAngle = startAngle + rotation ;
//				gyroCounter = 0 ;
//			}
//			gyroCounter++ ;
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
    			
//    			startAngle = angle + 20*rotation ;
    		}
//    		rotation = ( (angle-startAngle) / 30);

    		
    		myRobot.mecanumDrive_Cartesian( xcomm, ycomm, rotation, 0);
//    		newblink = stick.getRawButton(3);
//    		Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
//    		if (newblink)
//    		{
//    			if (newblink != oldblink)
//    			{
//    				blinkstatus = !blinkstatus ;
//    				blinkyLight.set(blinkstatus);
//    			}
//    			
//    		}
//    		oldblink = newblink;
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
    		
//	        camera.getImage(frame);
//	        temp_frame = frame;
//	        while ((y < 240) && (x < 320))
//	        {
//	        	while (y < 240)
//	        	{
//	        		y = y + 1;	
//	        	}
//	        	x = x + 1;
//	        	y = 300;
//	        	x = 400;
//	        			
//	        }
//	        NIVision.imaqDrawShapeOnImage(frame, frame, rect,
//	                DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
	
//	        CameraServer.getInstance().setImage(frame);
//	        
//	        y = 1000;
//	        x = 1000;
//	        /** robot code here! **/
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
	        LiveWindow.run();
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
//	        Timer.delay(0.005);		// wait for a motor update time
	    
	    	LiveWindow.run();
//	    	timeCounter += 1 ;
//	    	SmartDashboard.putNumber("time Counter", timeCounter);
//	    	if ( (armCommand < .2) && (armCommand > -.2) )
//			{
//				armCommand = 0 ;
	//
//			}
//			armCommand = armCommand * armCommandScale ;
//			
//			if (armCommand == 0 )
//			{
//				if (stick2.getRawButton(5))
//	    		{
//					armLeft.set(-.25);
//	    		}
//	    		else if (stick2.getRawButton(6))
//	    		{
//	    			armRight.set(-.25);
//	    		}
//	    		else
//	    		{
//	    			armLeft.set(0);
//	    			armRight.set(0); 
//	    		}
//			}
//			else
//			{
//				if (armCommand < 0 )
//				{
//					armLeft.set(armCommand*-1.055);
//				}
//				else
//				{
//					armLeft.set(armCommand*-1.02) ;	
//				}
//				
//				armRight.set(armCommand*-1) ;
//			}
//	    	
//	    	xAccel[0] = (accel.getX() - xAccelOffset) ;
//	    	yAccel[0] = (accel.getY() - yAccelOffset) ;
//	    	zAccel[0] = (accel.getZ() - zAccelOffset) ;
//	    	
//	    	xSpeed[0] = xSpeed[0] + ( xAccel[0] * 32 * .05) ; // v = v0 + (a) (t)
//	    	ySpeed[0] = ySpeed[0] + ( yAccel[0] * 32 * .05) ;
//	    	zSpeed[0] = zSpeed[0] + ( zAccel[0] * 32 * .05) ;	
//	    	
//	    	xDisp[0] = xDisp[0] + ( xSpeed[0] * .05 ) ; // x = x0 + v*t
//	    	yDisp[0] = yDisp[0] + ( ySpeed[0] * .05 ) ;
//	    	zDisp[0] = zDisp[0] + ( zSpeed[0] * .05 ) ;
//	    	
//	    	SmartDashboard.putNumber("xAccel", xAccel[0] );
//	    	SmartDashboard.putNumber("yAccel", yAccel[0] );
//	    	SmartDashboard.putNumber("zAccel", zAccel[0] );
//	    	
//	    	SmartDashboard.putNumber("xSpeed", xSpeed[0] );
//	    	SmartDashboard.putNumber("ySpeed", ySpeed[0] );
//	    	SmartDashboard.putNumber("zSpeed", zSpeed[0] );
//	    	
//	    	SmartDashboard.putNumber("xDisp", xDisp[0] );
//	    	SmartDashboard.putNumber("yDisp", yDisp[0] );
//	    	SmartDashboard.putNumber("zDisp", zDisp[0] );
//	    	LiveWindow.run();
    	}
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
//    	LiveWindow.run();
    	
    	curr_time = accel_time.get();
    	
//    	timeCounter += 1 ;
    	SmartDashboard.putNumber("time Counter", curr_time);
//    	if ( (armCommand < .2) && (armCommand > -.2) )
//		{
//			armCommand = 0 ;
//
//		}
//		armCommand = armCommand * armCommandScale ;
//		
//		if (armCommand == 0 )
//		{
//			if (stick2.getRawButton(5))
//    		{
//				armLeft.set(-.25);
//    		}
//    		else if (stick2.getRawButton(6))
//    		{
//    			armRight.set(-.25);
//    		}
//    		else
//    		{
//    			armLeft.set(0);
//    			armRight.set(0); 
//    		}
//		}
//		else
//		{
//			if (armCommand < 0 )
//			{
//				armLeft.set(armCommand*-1.055);
//			}
//			else
//			{
//				armLeft.set(armCommand*-1.02) ;	
//			}
//			
//			armRight.set(armCommand*-1) ;
//		}
//    	
    	xAccel[0] = (accel.getX() - xAccelOffset) ;
    	yAccel[0] = (accel.getY() - yAccelOffset) ;
    	zAccel[0] = (accel.getZ() - zAccelOffset) ;
    	
    	xSpeed[0] = xSpeed[0] + ( xAccel[0] * 32 * .05) ; // v = v0 + (a) (t)
    	ySpeed[0] = ySpeed[0] + ( yAccel[0] * 32 * .05) ;
    	zSpeed[0] = zSpeed[0] + ( zAccel[0] * 32 * .05) ;	
    	
    	xDisp[0] = xDisp[0] + ( xSpeed[0] * .05 ) ; // x = x0 + v*t
    	yDisp[0] = yDisp[0] + ( ySpeed[0] * .05 ) ;
    	zDisp[0] = zDisp[0] + ( zSpeed[0] * .05 ) ;
    	
    	SmartDashboard.putNumber("xAccel", xAccel[0] );
    	SmartDashboard.putNumber("yAccel", yAccel[0] );
    	SmartDashboard.putNumber("zAccel", zAccel[0] );
    	
    	SmartDashboard.putNumber("xSpeed", xSpeed[0] );
    	SmartDashboard.putNumber("ySpeed", ySpeed[0] );
    	SmartDashboard.putNumber("zSpeed", zSpeed[0] );
    	
    	SmartDashboard.putNumber("xDisp", xDisp[0] );
    	SmartDashboard.putNumber("yDisp", yDisp[0] );
    	SmartDashboard.putNumber("zDisp", zDisp[0] );
    	LiveWindow.run();
    	
    	myRobot.mecanumDrive_Cartesian( 0, 0, 0, 0);
    }
    
}
