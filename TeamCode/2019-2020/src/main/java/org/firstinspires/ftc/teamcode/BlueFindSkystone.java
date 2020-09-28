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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


@Autonomous(name="BlueFindSkystone", group="Skyline")

public class BlueFindSkystone extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //map to config file
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor fifthMotor = null;
    Servo StoneGrabber = null;

    TouchSensor touchLeft, touchRight;

    BNO055IMU imu; //gyroscope
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    ColorSensor colorSensor, bottomSensor;
    DistanceSensor distanceSensor;
    DistanceSensor sensorRange;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);
        leftRear = hardwareMap.get(DcMotor.class, "RearLeft");
        rightRear = hardwareMap.get(DcMotor.class, "RearRight");
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        fifthMotor = hardwareMap.get(DcMotor.class, "FifthMotor");
        StoneGrabber = hardwareMap.get(Servo.class, "StoneGrabber");
        touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        touchRight = hardwareMap.touchSensor.get("TouchRight");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        //setup gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        bottomSensor = hardwareMap.colorSensor.get("BottomSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        fifthMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        int slideDistance = 2500; //distance to strafe to the left
        int backUpDistance = 200; //distance to back away from the wall b4 strafing to the left
        int clearWallDistance = 400; //forward distance that determines clearing the wall
        int wallPos = 0, slidePos;
        boolean passedWall = false, leftTouched = false, rightTouched = false;
        boolean wallEdge = false;
        double wallAngle = getAngle();
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        boolean yellow = false;
        double speed = 0;
        //determines left or right movement based on color
        //blue = -1 -> Right      red = 1 -> Left
        int allianceColor = -1;
        Boolean black = false;
        Boolean foundblock = false;

        //intial open grabber
        StoneGrabber.setPosition(1);

        //move along wall
        speed = 0.5;
        leftFront.setPower(-speed * allianceColor);
        rightFront.setPower(speed * allianceColor);
        leftRear.setPower(speed * allianceColor);
        rightRear.setPower(-speed * allianceColor);
        sleep(1200);

        pause4DirectionSwitch(200);

        speed = 0.5;
        //move forward
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);


        while(!foundblock) {
            if(sensorRange.getDistance(DistanceUnit.INCH) < 12){
                speed = 0.1;
                //slow down
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);

                if (colorSensor.alpha() > 200) {

                    telemetry.addData("Stage: ", "Is alpha less than 400");
                    foundblock = true;
                    pause4DirectionSwitch(200);
                }
            }

        }
        leftFront.setPower(-speed * allianceColor * -1);
        rightFront.setPower(speed * allianceColor * -1);
        leftRear.setPower(speed * allianceColor * -1);
        rightRear.setPower(-speed * allianceColor * -1);

        int average;
        double percent = 0.10;
        while (!black) {
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Green: ", colorSensor.green());
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("Luminosity(Alpha): ", colorSensor.alpha());

            average = (colorSensor.red() + colorSensor.green() + colorSensor.blue())/3;average = (colorSensor.red() + colorSensor.green() + colorSensor.blue())/3;

            if((colorSensor.red()-average)<average*percent && (colorSensor.green()-average)<average*percent && (colorSensor.blue()-average)<average*percent){ //the color black is usually where the rbg values are close together
                //this if statement checks that each color value is less than 10% from the average (porribly adjust the precent values
                black = true;
                //wait 3/4 sec to align bot better
                sleep(300);
                speed = -.2;
                //if the alpha is NOT less than 100, and the colorsensor is BIGGER than 300, stop
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);
                sleep(200);
                speed = 0;
                //if the alpha is NOT less than 100, and the color sensor is BIGGER than 300, stop
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);
                telemetry.addData("Stage: ", "Black");
            }
            if (colorSensor.alpha() < 100){ //DO NOT USE THIS
                //if the alpha is NOT less than 100, and the colorSensor is NOT bigger than 300, go right
                speed = 0;
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);
                telemetry.addData("Stage: ", "Not Black");
            }
            telemetry.addData("Black: ", black);
            telemetry.update();
        }

        // movement done now
        speed = 0.0;
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
        //stone grabber
        //lower arm
        fifthMotor.setPower(0.4);
        sleep(500);
        fifthMotor.setPower(0);
        StoneGrabber.setPosition(0);
        sleep(1800);
        //raise arm
        fifthMotor.setPower(-0.3);
        sleep(500);
        //fifthMotor.setPower(0.0);
        //sleep(1000);
        telemetry.update();

        pause4DirectionSwitch(200);

        speed = -0.2;
        //move backwards
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
        sleep(1000);
        pause4DirectionSwitch(300);

        // turn -90 to move stone under sky bridge
        turnAngle(-90, 2, 0.2);

        pause4DirectionSwitch(200);

        // sleep(1000);

        //find blue tape denoting the skybridge
        //sensor doesn't detect blue as well as red. use 2x instead of 1.5x for red
        while (!(bottomSensor.blue() > 1.4*bottomSensor.red())){
            speed = 0.5;
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed);
            rightRear.setPower(speed);

            telemetry.addData("Red: ", bottomSensor.red());
            telemetry.addData("Green: ", bottomSensor.green());
            telemetry.addData("Blue: ", bottomSensor.blue());
            telemetry.addData("Luminosity: ", bottomSensor.alpha());
            telemetry.addData("Argb: ", bottomSensor.argb());
            telemetry.update();
        }

        sleep(800);


        speed = 0.0;
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);

        //lower stone
        fifthMotor.setPower(0.4);
        sleep(500);
        StoneGrabber.setPosition(1);
        fifthMotor.setPower(.0);
        sleep(300);

        //raise arm
        fifthMotor.setPower(-0.2);
        //StoneGrabber.setPosition(0);

        //reverse back to center
        //sensor doesn't detect blue as well as red. use 2x instead of 1.5x for red
        speed = -0.4;
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);

        sleep(200);

        StoneGrabber.setPosition(0);

        while (!(bottomSensor.blue() > 1.4*bottomSensor.red())){

        }
    }

    // *******  GYROSCOPE Methods   *********

    // Resets the cumulative angle tracking to zero.
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Get current cumulative angle rotation from last reset.
    // @return Angle in degrees. + = left, - = right.
    private double getAngle() {
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
    private double checkDirection() {
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

    public void turnAngle(double angle, int tolerance, double motorPower) {
        //tolerance: how much angle can be off by
        //motorPower: how fast to turn
        int direction = 1; //+ is clockwise turning
        resetAngle();
        if (angle < 0) {
            direction = -1;
        }
        while ((Math.abs(angle) - Math.abs(getAngle()) > tolerance)) {
            leftFront.setPower(motorPower * direction);
            rightFront.setPower(-1 * motorPower * direction);
            leftRear.setPower(motorPower * direction);
            rightRear.setPower(-1 * motorPower * direction);
            telemetry.addData("Wanted Angle", angle);
            telemetry.addData("Curr Angle", getAngle());
            telemetry.update();
        }
    }

    public void pause4DirectionSwitch(int delay) {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(delay);
    }

}
