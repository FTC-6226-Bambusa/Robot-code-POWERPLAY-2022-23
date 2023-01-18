package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.sleevedetection.ParkingPosition.CENTER;
import static org.firstinspires.ftc.teamcode.sleevedetection.ParkingPosition.LEFT;
import static org.firstinspires.ftc.teamcode.sleevedetection.ParkingPosition.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.channels.Pipe;
import java.util.Arrays;

@Autonomous(name="Sleeve Detector", group="Auto")
public class auto extends LinearOpMode {
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

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        sleevedetection detector = new sleevedetection(telemetry);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        waitForStart();

        double botHeading = -imu.getAngularOrientation().firstAngle;

        switch (detector.getPosition()) {
            case LEFT:
                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.55);
                motorFrontRight.setPower(-0.55);
                motorFrontLeft.setPower(0.5);
                sleep(700);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                sleep(200);
                //rotate a lil bit
                motorBackLeft.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                sleep(120);
                motorBackLeft.setPower(0);
                motorBackLeft.setPower(0);
                //strafe left
                motorBackLeft.setPower(0.55);
                motorBackRight.setPower(0.55);
                motorFrontRight.setPower(-0.55);
                motorFrontLeft.setPower(-0.5);
                sleep(1100);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                break;

            case RIGHT: //make sure its a bit to the corner so that it has room to spare :p
                    motorBackLeft.setPower(0.5);
                    motorBackRight.setPower(-0.55);
                    motorFrontRight.setPower(-0.55);
                    motorFrontLeft.setPower(0.5);
                    sleep(700);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    sleep(200);
                    //rotate a lil bit
                    motorBackLeft.setPower(0.5);
                    motorBackLeft.setPower(0.5);
                    sleep(200);
                    motorBackLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    //strafe right*/
                    motorBackLeft.setPower(-0.55);
                    motorBackRight.setPower(-0.55);
                    motorFrontRight.setPower(0.55);
                    motorFrontLeft.setPower(0.5);
                    sleep(1100);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    break;
            case CENTER:

                 // 1+5 auto begins

                // 1) grip claw
                servo.setPower(0.3);
                sleep(1300);

                //set target positions for the drivetrain motors & mast motors
                mmotor0.setTargetPosition(-1750);
                mmotor1.setTargetPosition(-1750);
                motorBackLeft.setTargetPosition(2225);
                motorBackRight.setTargetPosition(-2225);
                motorFrontLeft.setTargetPosition(2225);
                motorFrontRight.setTargetPosition(-2225);

                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set motor power
                mmotor0.setPower(0.97);
                mmotor1.setPower(0.97);
                sleep(200);//lets the claw get a little off the ground before the rest of the robot starts moving
                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorFrontRight.setPower(-0.5);
                //let motors run
                while (opModeIsActive() && motorFrontRight.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but

                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                mmotor0.setPower(0.0);
                mmotor1.setPower(0.0);

                sleep(200);
                //TIME TO TURN 45 Deg
               //Note how the mmotors arent touched here, keep them an independent system
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //make imu variable the thing that determines rotation
                //rotate to make cone align with high junction

                while (opModeIsActive() && botHeading > -127){
                    motorBackLeft.setPower(0.6);
                    motorBackRight.setPower(0.6);
                    motorFrontLeft.setPower(0.6);
                    motorFrontRight.setPower(0.6);
                    botHeading = -imu.getAngularOrientation().firstAngle;
                }
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);

                //fully extend arm
                mmotor0.setTargetPosition(-2050);
                mmotor1.setTargetPosition(-2050);

                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                mmotor0.setPower(0.97);
                mmotor1.setPower(0.97);
                //let motors run
                while (opModeIsActive() && mmotor0.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                mmotor0.setPower(0.0);
                mmotor1.setPower(0.0);




                //move back 15 inches to get ready to place cone

                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setTargetPosition(-420);
                motorBackRight.setTargetPosition(420);
                motorFrontLeft.setTargetPosition(-420);
                motorFrontRight.setTargetPosition(420);

                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorFrontRight.setPower(-0.5);
                //let motors run
                while (opModeIsActive() && motorFrontRight.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but
                // power is still applied so we turn off the power.

                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                sleep(1500);
                servo.setPower(-0.3);
                //servo opens again
                sleep(250);
                servo.setPower(0.0);


                //TIME TO MOVE BACK NOW ><

                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setTargetPosition(350);
                motorBackRight.setTargetPosition(-350);
                motorFrontLeft.setTargetPosition(350);
                motorFrontRight.setTargetPosition(-350);

                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                motorBackLeft.setPower(0.4);
                motorBackRight.setPower(-0.4);
                motorFrontLeft.setPower(0.4);
                motorFrontRight.setPower(-0.4);
                //let motors run
                while (opModeIsActive() && motorFrontRight.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but
                // power is still applied so we turn off the power.

                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);

                //ROTATE 135 degrees CCW to face the cone stack
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                while (opModeIsActive() && botHeading < -96){
                    motorBackLeft.setPower(-0.4);
                    motorBackRight.setPower(-0.4);
                    motorFrontLeft.setPower(-0.4);
                    motorFrontRight.setPower(-0.4);
                    botHeading = -imu.getAngularOrientation().firstAngle;
                }
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);





                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    //move to cone stack
                    mmotor0.setTargetPosition(-290);
                    mmotor1.setTargetPosition(-290);
                    motorBackLeft.setTargetPosition(750);
                    motorBackRight.setTargetPosition(-750);
                    motorFrontLeft.setTargetPosition(750);
                    motorFrontRight.setTargetPosition(-750);


                    motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //set motor power
                    mmotor0.setPower(0.75);
                    mmotor1.setPower(0.75);
                    motorBackLeft.setPower(0.5);
                    motorBackRight.setPower(-0.5);
                    motorFrontLeft.setPower(0.5);
                    motorFrontRight.setPower(-0.5);
                    //let motors run
                    while (opModeIsActive() && (motorFrontRight.isBusy() || mmotor0.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                    {
                        telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                        telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                        telemetry.update();
                        idle();
                    }

                    // set motor power to zero to turn off motors. The motors stop on their own but
                    mmotor0.setPower(0.0);
                    mmotor1.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);

                    sleep(100);

                    //grab first cone stack cone
                    servo.setPower(0.3);
                    sleep(300);
                    //lift arm to position
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    motorBackLeft.setTargetPosition(-720);
                    motorBackRight.setTargetPosition(720);
                    motorFrontLeft.setTargetPosition(-720);
                    motorFrontRight.setTargetPosition(720);
                    mmotor0.setTargetPosition(-2050);
                    mmotor1.setTargetPosition(-2050);

                    mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    motorBackLeft.setPower(0.65);
                    motorBackRight.setPower(-0.65);
                    motorFrontLeft.setPower(0.65);
                    motorFrontRight.setPower(-0.65);
                    sleep(100);
                    mmotor0.setPower(0.97);
                    mmotor1.setPower(0.97);

                    while (opModeIsActive() && mmotor0.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                    {
                        telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                        telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                        telemetry.update();
                        idle();
                    }

                    // set motor power to zero to turn off motors. The motors stop on their own but
                    // power is still applied so we turn off the power.

                    mmotor0.setPower(0.0);
                    mmotor1.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);

                    //turn to face the pole
                    motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    while (opModeIsActive() && botHeading > -133) {
                        motorBackLeft.setPower(0.4);
                        motorBackRight.setPower(0.4);
                        motorFrontLeft.setPower(0.4);
                        motorFrontRight.setPower(0.4);
                        botHeading = -imu.getAngularOrientation().firstAngle;
                    }

                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);


                    //move back 420 ticks to get ready to place cone
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    //move to High Junction
                    motorBackLeft.setTargetPosition(-420);
                    motorBackRight.setTargetPosition(420);
                    motorFrontLeft.setTargetPosition(-420);
                    motorFrontRight.setTargetPosition(420);


                    motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //set motor power
                    motorBackLeft.setPower(0.5);
                    motorBackRight.setPower(-0.5);
                    motorFrontLeft.setPower(0.5);
                    motorFrontRight.setPower(-0.5);
                    //let motors run
                    while (opModeIsActive() && (motorFrontRight.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                    {
                        telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                        telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                        telemetry.update();
                        idle();
                    }
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);
                    //wait a bit and release claw
                    sleep(1000);
                    servo.setPower(-0.3);
                    //servo opens again
                    sleep(250);
                    servo.setPower(0.0);

                    //move to center of square
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    motorBackLeft.setTargetPosition(420);
                    motorBackRight.setTargetPosition(-420);
                    motorFrontLeft.setTargetPosition(420);
                    motorFrontRight.setTargetPosition(-420);


                    motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //set motor power
                    motorBackLeft.setPower(0.5);
                    motorBackRight.setPower(-0.5);
                    motorFrontLeft.setPower(0.5);
                    motorFrontRight.setPower(-0.5);
                    //let motors run
                    while (opModeIsActive() && (motorFrontRight.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                    {
                        telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                        telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                        telemetry.update();
                        idle();
                    }

                    // set motor power to zero to turn off motors. The motors stop on their own but
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);


                    //rotate to face cone stack

                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (opModeIsActive() && botHeading < -95) {
                    motorBackLeft.setPower(-0.4);
                    motorBackRight.setPower(-0.4);
                    motorFrontLeft.setPower(-0.4);
                    motorFrontRight.setPower(-0.4);
                    botHeading = -imu.getAngularOrientation().firstAngle;
                }

                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);

//---------------------------------------------------------------------------------------------next cone bby--------------------------------------------------------

                //move to cone stack

                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                mmotor0.setTargetPosition(-230);
                mmotor1.setTargetPosition(-230);
                motorBackLeft.setTargetPosition(720);
                motorBackRight.setTargetPosition(-720);
                motorFrontLeft.setTargetPosition(720);
                motorFrontRight.setTargetPosition(-720);


                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set motor power
                mmotor0.setPower(0.75);
                mmotor1.setPower(0.75);
                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorFrontRight.setPower(-0.5);
                //let motors run
                while (opModeIsActive() && (motorFrontRight.isBusy() || mmotor0.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but
                mmotor0.setPower(0.0);
                mmotor1.setPower(0.0);
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);

                sleep(100);

                //grab first cone stack cone
                servo.setPower(0.3);
                sleep(300);
                //lift arm to position
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorBackLeft.setTargetPosition(-750);
                motorBackRight.setTargetPosition(750);
                motorFrontLeft.setTargetPosition(-750);
                motorFrontRight.setTargetPosition(750);
                mmotor0.setTargetPosition(-2050);
                mmotor1.setTargetPosition(-2050);

                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                motorBackLeft.setPower(0.65);
                motorBackRight.setPower(-0.65);
                motorFrontLeft.setPower(0.65);
                motorFrontRight.setPower(-0.65);
                sleep(100);
                mmotor0.setPower(0.97);
                mmotor1.setPower(0.97);

                while (opModeIsActive() && mmotor0.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but
                // power is still applied so we turn off the power.

                mmotor0.setPower(0.0);
                mmotor1.setPower(0.0);
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);

                //turn to face the pole
                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (opModeIsActive() && botHeading > -130) {
                    motorBackLeft.setPower(0.4);
                    motorBackRight.setPower(0.4);
                    motorFrontLeft.setPower(0.4);
                    motorFrontRight.setPower(0.4);
                    botHeading = -imu.getAngularOrientation().firstAngle;
                }

                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);


                //move back 150 ticks to get ready to place cone
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //move to High Junction
                motorBackLeft.setTargetPosition(-420);
                motorBackRight.setTargetPosition(420);
                motorFrontLeft.setTargetPosition(-420);
                motorFrontRight.setTargetPosition(420);


                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set motor power
                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorFrontRight.setPower(-0.5);
                //let motors run
                while (opModeIsActive() && (motorFrontRight.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                //wait a bit and release claw
                sleep(1000);
                servo.setPower(-0.3);
                //servo opens again
                sleep(250);
                servo.setPower(0.0);

                //move to center of square
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                motorBackLeft.setTargetPosition(420);
                motorBackRight.setTargetPosition(-420);
                motorFrontLeft.setTargetPosition(420);
                motorFrontRight.setTargetPosition(-420);


                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set motor power
                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorFrontRight.setPower(-0.5);
                //let motors run
                while (opModeIsActive() && (motorFrontRight.isBusy()))   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
                {
                    telemetry.addData("encoder-fwd-left", mmotor0.getCurrentPosition() + "  busy=" + mmotor0.isBusy());
                    telemetry.addData("encoder-fwd-right", mmotor1.getCurrentPosition() + "  busy=" + mmotor1.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but
                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);


                //rotate to face cone stack

                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (opModeIsActive() && botHeading < -95) {
                    motorBackLeft.setPower(-0.4);
                    motorBackRight.setPower(-0.4);
                    motorFrontLeft.setPower(-0.4);
                    motorFrontRight.setPower(-0.4);
                    botHeading = -imu.getAngularOrientation().firstAngle;
                }

                motorBackLeft.setPower(0.0);
                motorBackRight.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                break;
        }
        webcam.stopStreaming();
    }

}