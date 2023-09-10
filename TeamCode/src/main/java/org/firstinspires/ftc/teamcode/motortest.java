package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class motortest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Declare our AXON drive servos (in cont. rotation mode)
        CRServo axon1 = hardwareMap.crservo.get("axon1");
        CRServo axon2 = hardwareMap.crservo.get("axon2");
        CRServo axon3 = hardwareMap.crservo.get("axon3");
        CRServo axon4 = hardwareMap.crservo.get("axon4");

        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor4");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declare the analog input from the encoders on these fantastic servos. 0v = 0 degrees, 3.3v = 360.
        AnalogInput axon1enc = hardwareMap.get(AnalogInput.class, "axon1enc");
        AnalogInput axon2enc = hardwareMap.get(AnalogInput.class, "axon2enc");
        AnalogInput axon3enc = hardwareMap.get(AnalogInput.class, "axon3enc");
        AnalogInput axon4enc = hardwareMap.get(AnalogInput.class, "axon4enc");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);
            motorFrontRight.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.left_stick_y);
        }
    }
}