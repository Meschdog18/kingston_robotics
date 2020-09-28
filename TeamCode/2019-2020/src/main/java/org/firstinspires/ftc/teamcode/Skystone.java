package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import java.util.TimerTask;

import java.util.Base64;

@TeleOp(name="SkyStone", group="Linear Opmode")

public class Skystone extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    Servo RightLatch = null;
    Servo LeftLatch = null;
    Servo StoneGrabber = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor FifthMotor = null;
    //    int WantedArmPos = 0;
    int CurrentArmPos = 0;
    int EncoderOffSet = 0;
    boolean Pull = false;
    double speed = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RightLatch  = hardwareMap.get(Servo.class, "RightLatch");
        LeftLatch = hardwareMap.get(Servo.class, "LeftLatch");
        leftRear = hardwareMap.get(DcMotor.class, "RearLeft");
        rightRear = hardwareMap.get(DcMotor.class, "RearRight");
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        FifthMotor = hardwareMap.get(DcMotor.class, "FifthMotor");
        StoneGrabber = hardwareMap.get(Servo.class, "StoneGrabber");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FifthMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        final double maxPower = 1;
        // set power to zero to avoid a FTC bug
        EncoderOffSet = FifthMotor.getCurrentPosition();
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        waitForStart();
        runtime.reset();
        RightLatch.setPosition(0.5);
        LeftLatch.setPosition(0.5);

        while (opModeIsActive()) {

            grabber();
            mechanum();
            Latch();
            telemetry.update();

        }

    }
    public void grabber() {
        FifthMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CurrentArmPos = FifthMotor.getCurrentPosition() - EncoderOffSet;
        telemetry.addData("Encoder Pos:", CurrentArmPos);
        if (gamepad2.left_bumper) {
            StoneGrabber.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            StoneGrabber.setPosition(0.0);
        }
        //Fifth motor controls
        //get current motor encoder position than have motor goto that position to lock in place
        //limiter

        //if(gamepad2.a && CurrentArmPos <= 370){
        if (gamepad2.a) {
            FifthMotor.setPower(0.15);
        } else {
            if (!FifthMotor.isBusy()) {
                FifthMotor.setPower(0.0);
            }
            //FifthMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // FifthMotor.setPower(0.0);
          /*
            CurrentArmPos = FifthMotor.getCurrentPosition();

            if(WantedArmPos != CurrentArmPos && !FifthMotor.isBusy()){
            FifthMotor.setPower(0.2);
            FifthMotor.setTargetPosition(WantedArmPos);
            FifthMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }else{
            FifthMotor.setPower(0.0);
        }
      */

        }
        //if(gamepad2.y && CurrentArmPos >= 10){

        if (gamepad2.left_stick_y > 0.1) {
            FifthMotor.setPower(0.2);
        }
        else {
            if (gamepad2.left_stick_y < -0.1) {
                FifthMotor.setPower(-0.2);
            } else {
                FifthMotor.setPower(0);
            }
        }




       // if (gamepad2.y) {

        //    FifthMotor.setPower(-0.15);
            //WantedArmPos = FifthMotor.getCurrentPosition();

       // } else {
            // CurrentArmPos = FifthMotor.getCurrentPosition();
        //    if (!FifthMotor.isBusy()) {
        //        FifthMotor.setPower(0.0);
         //   }
/*
            if(WantedArmPos != CurrentArmPos  && !FifthMotor.isBusy()){
                FifthMotor.setPower(0.2);
                FifthMotor.setTargetPosition(WantedArmPos);
                FifthMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                FifthMotor.setPower(0.0);
            }
           */// FifthMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //  FifthMotor.setPower(0.0);
      //  }



        if (gamepad1.left_bumper) {
            //2 secs

/*
            final Timer timer = new Timer();
            final TimerTask task = new TimerTask() {
                private int count = 0;
                double speed = -0.1;

                public void run() {

                    speed = speed - .1;
                    leftFront.setPower(speed);
                    rightFront.setPower(speed);
                    leftRear.setPower(speed);
                    rightRear.setPower(speed);
                    count++;
                    if (count == 4) {
                        double speed = 0;
                        leftFront.setPower(speed);
                        rightFront.setPower(speed);
                        leftRear.setPower(speed);
                        rightRear.setPower(speed);

                        timer.cancel();

                    }
                }


            };


            timer.schedule(task, 500, 1000);//call timer

            //when done stop
*/

/*
            for(int i=1;i<5;i++){
                double speed = i/4;
                telemetry.addData("speed", speed);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                telemetry.update();
            rightRear.setPower(speed);

                sleep(1000);
            }

 */
            while (speed < .4) {
                speed += .01;
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(-speed);
                sleep(35);
            }
            while (!gamepad1.right_bumper) {} //wait until button is pressed
            while (speed > 0) {
                speed -= .01;
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(-speed);
                sleep(35);

            }
            RightLatch.setPosition(1.0);
            LeftLatch.setPosition(1.0);

        }
    }
    public void Latch(){
        if(gamepad2.dpad_down){
            RightLatch.setPosition(0.0);
            LeftLatch.setPosition(0.0);
        }
        if(gamepad2.dpad_up){
            RightLatch.setPosition(1);
            LeftLatch.setPosition(1);
        }
    }
    public void mechanum(){
        final double maxPower = 1.0;

        double joy1Y = gamepad1.left_stick_x;
        joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
        double joy1X = gamepad1.left_stick_y;
        joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
        double joy2X = gamepad1.right_stick_x;
        joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;

        leftFront.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        rightFront.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        leftRear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        rightRear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));


        telemetry.addData("LF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        telemetry.addData("RF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        telemetry.addData("LR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        telemetry.addData("RR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
    }






}





