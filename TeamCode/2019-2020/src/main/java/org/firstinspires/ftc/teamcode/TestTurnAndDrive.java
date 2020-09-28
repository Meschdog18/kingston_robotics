package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="TestTurnAndDrive", group="Unknown")
//@Disabled

public class TestTurnAndDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //map to config file
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor fifthMotor = null;
    DcMotor sixthMotor = null;
    Servo RightLatch = null;
    Servo LeftLatch = null;
    TouchSensor touchLeft, touchRight;

    BNO055IMU imu; //gyroscope
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    ColorSensor colorSensor, bottomSensor;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // sleep(250);

        //link motors, servos, etc
        leftRear = hardwareMap.get(DcMotor.class, "RearLeft");
        rightRear = hardwareMap.get(DcMotor.class, "RearRight");
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        RightLatch  = hardwareMap.get(Servo.class, "RightLatch");
        LeftLatch = hardwareMap.get(Servo.class, "LeftLatch");
        touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        touchRight = hardwareMap.touchSensor.get("TouchRight");

        //setup gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port on a
        // Core Device Interface Module, configured to be a sensor of type "REV Expansion Hub IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        //calibrate gyroscope
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        //check out file SensorREVColorDistance for more info
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        bottomSensor = hardwareMap.colorSensor.get("BottomSensor");

        // get a reference to the distance sensor that shares the same name.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");

        // IMPORTANT-In robot config use odd numbered ports for each digital device

        //set motor encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse direction of motors on left side
        //This insures that all motors will rotate in same direction when setting power
        //with the same number.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Starting left turn");
        telemetry.update();
       // turnLeft(180,1,0.3);
        turnAngle(90,0.5,0.3);

        while(opModeIsActive() && runtime.seconds() < 6) {


        }
        //turnLeft(90,1,0.3);
        //sleep(5000);
    } //runOpMode()



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
        //This assumes the Rev Hub is vertical.
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

    //Operation methods
    public void turnAngle(double angle, double tolerance, double motorPower) {
        //tolerance: how much angle can be off by
        //motorPower: how fast to turn
        double angleDiff;
        int direction = 1; //+ is clockwise turning
        resetAngle();
        if (angle < 0) {
            direction = -1;
        }
        while ((Math.abs(angle) - Math.abs(getAngle()) > tolerance)) {

            angleDiff = Math.abs(angle) - Math.abs(getAngle());
            if (angleDiff < 30 && angleDiff > 20) {
                motorPower = 0.2;
            }
            else if (angleDiff < 20) {
                motorPower = 0.1;
            }


            leftFront.setPower(motorPower * direction);
            rightFront.setPower(-1 * motorPower * direction);
            leftRear.setPower(motorPower * direction);
            rightRear.setPower(-1 * motorPower * direction);

            telemetry.addData("Wanted Angle", angle);
            telemetry.addData("Curr Angle", getAngle());
            telemetry.update();
        }
    } //turnAngle()

    public void turnLeft(double turnAngle, int tolerance, double motorPower) {

        double curTime;
        double angleDiff = 10, targetAngle;
        double angleTolerance = 0.5; // + or - how close to match turnAngle
        double maxTurnTime = 4; //max amount of time (secs) to spend turning
        Orientation angles; //contains gyroscope data

        //get current gyroscope info
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        targetAngle=angles.firstAngle + turnAngle;

        if(targetAngle<-180) {targetAngle+=360;}
        if(targetAngle>180) {targetAngle-=360;}

        curTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - curTime) < maxTurnTime && angleDiff > angleTolerance) {
            //need to work on overshooting the angle
            // curAngle < targetAngle) {

            leftRear.setPower(-1 * motorPower);
            rightRear.setPower(motorPower);
            leftFront.setPower(-1 * motorPower);
            rightFront.setPower(motorPower);

            //get current gyroscope info
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            angleDiff = Math.abs(Math.abs(angles.firstAngle) - targetAngle);

            if (angleDiff < 30 && angleDiff > 20) {
                motorPower = 0.2;
            }
            else if (angleDiff < 20) {
                motorPower = 0.1;
            }

           // telemetry.addData("prev angle", prevAngle);
            telemetry.addData("targetHeading", targetAngle);
            telemetry.addData("curAngle", angles.firstAngle);
            telemetry.addData("motorPower", motorPower);
            telemetry.update();

        }
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("targetHeading", targetAngle);
        telemetry.addData("curAngle", angles.firstAngle);
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
/*
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 10 && degreesLeft > 1 &&
                oldDegreesLeft - degreesLeft >= 0) {
            //check to see if we overshot target
            scaledPower = degreesLeft / (100 + degreesLeft) * motorPower;


            if (scaledPower > 1) {
                scaledPower = 0.1;
            }
            leftRear.setPower(scaledPower);
            rightRear.setPower(-1 * scaledPower);
            leftFront.setPower(scaledPower);
            rightFront.setPower(-1 * scaledPower);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft = degreesLeft;
            degreesLeft = ((int) (Math.signum(angles.firstAngle - targetHeading) + 1) / 2) *
                    (360 - Math.abs(angles.firstAngle - targetHeading)) +
                    (int) (Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
            if (Math.abs(angles.firstAngle - prevAngle) < 1) {
                motorPower *= 1.1;
            } //bump up speed to wheels in case robot stalls before reaching target
            prevAngle = angles.firstAngle;


        } //while
*/

    } // turnLeft()






} //program end
