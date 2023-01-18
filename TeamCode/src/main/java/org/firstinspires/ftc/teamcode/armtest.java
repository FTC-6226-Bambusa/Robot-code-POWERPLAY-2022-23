package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.easyopencv.OpenCvCamera;


    @Autonomous(name="armtest", group="Auto")
    public class armtest extends LinearOpMode {
        sleevedetection detector;
        private OpenCvCamera webcam;

        @Override
        public void runOpMode() throws InterruptedException {
            DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor1");
            DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor0");
            DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");
            DcMotor motorBackRight = hardwareMap.dcMotor.get("motor3");
            DcMotor mmotor0 = hardwareMap.dcMotor.get("mmotor0");
            DcMotor mmotor1 = hardwareMap.dcMotor.get("mmotor1");
            CRServo servo = hardwareMap.crservo.get("intake");

            mmotor0.setDirection(DcMotorSimple.Direction.REVERSE);

            mmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            waitForStart();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            mmotor0.setTargetPosition(-1750);
            mmotor1.setTargetPosition(-1750);
            mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            mmotor0.setPower(0.97);
            mmotor1.setPower(0.97);

            while (opModeIsActive() && mmotor0.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {
                telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                telemetry.update();
                idle();
            }
            mmotor0.setPower(0.0);
            mmotor1.setPower(0.0);

            //this is where the next one begins
           /* sleep(1500);
            mmotor0.setTargetPosition(-2050);
            mmotor1.setTargetPosition(-2050);
            mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            mmotor0.setPower(0.97);
            mmotor1.setPower(0.97);

            while (opModeIsActive() && mmotor0.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {
                telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                telemetry.update();
                idle();
            }
            mmotor0.setPower(0.0);
            mmotor1.setPower(0.0);*/
        }
    }
