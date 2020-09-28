package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MecanumDrive", group="Linear Opmode")

public class MecanumDrive extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor fifth_motor = null;
    private DcMotor sixth_motor = null;
    public DcMotor extend= null;
    public Servo ball_pusher = null;
    BNO055IMU imu; //gyroscope
    TouchSensor touchLeft, touchRight;
    boolean touchedTriggered;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);
        left_rear = hardwareMap.get(DcMotor.class, "RearLeft");
        right_rear = hardwareMap.get(DcMotor.class, "RearRight");
        left_front = hardwareMap.get(DcMotor.class, "FrontLeft");
        right_front = hardwareMap.get(DcMotor.class, "FrontRight");

        //fifth_motor  = hardwareMap.get(DcMotor.class, "FifthMotor");
        //sixth_motor  = hardwareMap.get(DcMotor.class, "SixthMotor");
        //extend = hardwareMap.get(DcMotor.class, "lifter");
        // ball_pusher = hardwareMap.get(Servo.class, "BallPusher");
        // POV Mode uses left stick to go forward, and right stick to turn.5
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        // get a reference to touch sensor.
        //touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        //touchRight = hardwareMap.touchSensor.get("TouchRight");



        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        final double maxPower = 1;
        // set power to zero to avoid a FTC bug
        right_front.setPower(0);
        left_front.setPower(0);
        left_rear.setPower(0);
        right_rear.setPower(0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            mechanum();
        }

    }




    public void mechanum(){
        final double maxPower = 0.5;

        double joy1Y = gamepad1.left_stick_x;
        joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
        double joy1X = gamepad1.left_stick_y;
        joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
        double joy2X = gamepad1.right_stick_x;
        joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;

        left_front.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        right_front.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        left_rear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        right_rear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));


        telemetry.addData("LF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        telemetry.addData("RF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        telemetry.addData("LR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        telemetry.addData("RR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
        telemetry.update();
    }






}






/*
    public void Mechanum(){
        final double maxPower = 1;

        double joy1Y = -gamepad1.left_stick_x;
        joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
        double joy1X = gamepad1.left_stick_y;
        joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
        double joy2X = gamepad1.right_stick_x;
        joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;

        rf.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        lf.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        rr.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
        lr.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
    }

    */