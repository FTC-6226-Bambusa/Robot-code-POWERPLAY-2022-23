package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor1");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor0");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor3");
        DcMotor mmotor0 = hardwareMap.dcMotor.get("mmotor0");
        DcMotor mmotor1 = hardwareMap.dcMotor.get("mmotor1");
        CRServo servo = hardwareMap.crservo.get("intake");

        ElapsedTime h = new ElapsedTime();

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mmotor0.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        mmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double e = 1; //left trigger/right trigger will change the speed from default

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;



            //low & slow mode
            if (gamepad1.left_trigger > 0.2){
                e = 0.6;
            }
            //book it mode!
            if (gamepad1.right_trigger > 0.2){
                e = 2;
            }

            //THIS MOVES IT UP
            if (gamepad2.y){
                mmotor0.setTargetPosition(-2200);
                mmotor1.setTargetPosition(-2200);

                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mmotor0.setPower(0.97);
                mmotor1.setPower(0.97);
            }
            else if (gamepad2.a)/*THIS MOVES IT BACK DOWN*/ {
                mmotor0.setTargetPosition(0);
                mmotor1.setTargetPosition(0);
                servo.setPower(-0.2);
                sleep(200);
                servo.setPower(0);

                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mmotor0.setPower(0.97);
                mmotor1.setPower(0.97);

            }
            else if (mmotor0.isBusy()) {
                telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                telemetry.update();
            }

            else {

                //manual control of the lift via the gamepad//
                mmotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mmotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mmotor0.setPower(0.3*gamepad2.left_stick_y);
                mmotor1.setPower(0.3*gamepad2.left_stick_y);
            }


            if (gamepad2.x){
                servo.setPower(0.2);
            }

            //speed changing time and motor power setting
            motorFrontLeft.setPower(e*0.5*frontLeftPower);
            motorBackLeft.setPower(e*0.5*backLeftPower);
            motorFrontRight.setPower(e*0.5*frontRightPower);
            motorBackRight.setPower(e*0.5*backRightPower);
        }
    }
}