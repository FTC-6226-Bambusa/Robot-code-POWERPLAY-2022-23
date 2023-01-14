package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.sleevedetection.ParkingPosition.CENTER;
import static org.firstinspires.ftc.teamcode.sleevedetection.ParkingPosition.LEFT;
import static org.firstinspires.ftc.teamcode.sleevedetection.ParkingPosition.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
     //   motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
      //  motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //set encoder mode for mast motors (mmotor1 and mmotor0)
       /* mmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);*/
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
        sleep(1000);

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
                motorBackLeft.setPower(0.5);
                motorBackRight.setPower(-0.55);
                motorFrontRight.setPower(-0.55);
                motorFrontLeft.setPower(0.5);
                sleep(800);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                 // delete after here

                break;
        }
        webcam.stopStreaming();
    }

}