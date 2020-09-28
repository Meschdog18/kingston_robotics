package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

        import java.util.Locale;

@TeleOp(name="TestColor", group="Linear Opmode")

public class TestColor extends LinearOpMode {

    //map to config file
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor fifthMotor = null;
    DcMotor sixthMotor = null;
    Servo ballPusher = null;
    TouchSensor touchLeft, touchRight;
    ColorSensor colorSensor, bottomSensor;
   // DistanceSensor distanceSensor;
    BNO055IMU imu; //gyroscope
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    double r,robotAngle, rightX, v1, v2, v3, v4;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);
        //link motors, servos, etc
        leftRear   = hardwareMap.get(DcMotor.class, "RearLeft");
        rightRear  = hardwareMap.get(DcMotor.class, "RearRight");
        leftFront  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        fifthMotor = hardwareMap.get(DcMotor.class, "FifthMotor");
       // sixthMotor = hardwareMap.get(DcMotor.class, "SixthMotor");
       // ballPusher  = hardwareMap.get(Servo.class, "BallPusher");
        //can not turn light off, for colors position around 2cm away. Values needs to be
        //adjusted for ambient lighting.
        //proximity is best from 5-25cm
        //check out file SensorREVColorDistance for more info
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        bottomSensor = hardwareMap.colorSensor.get("BottomSensor");
        // get a reference to the distance sensor that shares the same name.
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");
        // IMPORTANT-In robot config use odd numbered ports for each digital device
        touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        touchRight = hardwareMap.touchSensor.get("TouchRight");

        //setup gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port on a
        // Core Device Interface Module, configured to be a sensor of type "AREV Expansion Hub IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }








        //displays before pressing play
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // sleep(500);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();


        // wait for start button.
        waitForStart();
        resetAngle();

        while (opModeIsActive()) {

            telemetry.addData("Mode", "running...");
            telemetry.update();

            //set each motor direction - depends on their mounted orientation
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            fifthMotor.setDirection(DcMotor.Direction.FORWARD);
            //sixthMotor.setDirection(DcMotor.Direction.REVERSE);
           // ballPusher.setDirection(Servo.Direction.FORWARD);

            if(gamepad1.x) {
                fireAway();
            }

            r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            v1 = r * Math.cos(robotAngle) + rightX;
            v2 = r * Math.sin(robotAngle) - rightX;
            v3 = r * Math.sin(robotAngle) + rightX;
            v4 = r * Math.cos(robotAngle) - rightX;

/*
            telemetry.addData("V1: ", v1);
            telemetry.addData("V2: ", v2);
            telemetry.addData("V3: ", v3);
            telemetry.addData("V4: ", v4);
*/
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);




            //display data
            // telemetry.addData("Left Rear Power",leftPower);
            // telemetry.addData("Right Rear Power",rightPower);
            //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

            correction = checkDirection();

            //telemetry.addData("1 imu heading", lastAngles.firstAngle);
  //          telemetry.addData("Global Heading: ", globalAngle);
            //telemetry.addData("3 correction", correction);

           // telemetry.addData("Touched Left: ", touchLeft.isPressed());
            //telemetry.addData("Touched Right: ", touchRight.isPressed());
            //telemetry.addData("Distance:",String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));



            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Green: ", colorSensor.green());
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("Luminosity: ", colorSensor.alpha());
            //telemetry.addData("Argb: ", colorSensor.argb());

            telemetry.addData("Red: ", bottomSensor.red());
            telemetry.addData("Green: ", bottomSensor.green());
            telemetry.addData("Blue: ", bottomSensor.blue());
            telemetry.addData("Luminosity: ", bottomSensor.alpha());
            //telemetry.addData("Argb: ", bottomSensor.argb());
            telemetry.update();

            if(gamepad1.y) {
                //Keep moving until finding blue
                while (!(colorSensor.alpha() > 100 && colorSensor.blue() > 2*colorSensor.red())) {
                    telemetry.addData("Moving.....", "...");
                    telemetry.addData("Red: ", colorSensor.red());
                    telemetry.addData("Green: ", colorSensor.green());
                    telemetry.addData("Blue: ", colorSensor.blue());
                    telemetry.addData("Luminosity: ", colorSensor.alpha());
                    //telemetry.addData("Luminosity: ", colorSensor.alpha());
                    //telemetry.addData("Argb: ", colorSensor.argb());
                    telemetry.update();
                }
                telemetry.addData("Found Color", "BLUE");
                sleep(3000);
            }


        }
    }


    // *******  Action Methods  ***************

    //fires ball
    private void fireAway()
    {
        fifthMotor.setPower(1);
        sixthMotor.setPower(1);
        sleep(300);        // allow wheels to get up to max speed
        ballPusher.setPosition(0.05);  //pushes ball into wheels
        sleep(300);
        ballPusher.setPosition(0.5); //return pusher to initial position
        sleep(1000);
        ballPusher.setPosition(0.05);  //pushes ball into wheels
        sleep(300);
        ballPusher.setPosition(0.5); //return pusher to initial position
        sleep(500);      //keep wheels spinning
        fifthMotor.setPower(0);      //stop motors
        sixthMotor.setPower(0);
    }

    // *******  GYROSCOPE Methods   *********

    // Resets the cumulative angle tracking to zero.
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Get current cumulative angle rotation from last reset.
    // @return Angle in degrees. + = left, - = right.
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }


    // See if we are moving in a straight line and if not return a power correction value.
    // @return Power adjustment, + is adjust left - is adjust right.
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

}
