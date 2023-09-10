package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ServoHomeFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Declare our AXON drive servos (in cont. rotation mode)
        CRServo axon1 = hardwareMap.crservo.get("axon1");
        CRServo axon2 = hardwareMap.crservo.get("axon2");
        CRServo axon3 = hardwareMap.crservo.get("axon3");
        CRServo axon4 = hardwareMap.crservo.get("axon4");

        //Declare the analog input from the encoders on these fantastic servos. 0v = 0 degrees, 3.3v = 360.
        AnalogInput axon1enc = hardwareMap.get(AnalogInput.class, "axon1enc");
        AnalogInput axon2enc = hardwareMap.get(AnalogInput.class, "axon2enc");
        AnalogInput axon3enc = hardwareMap.get(AnalogInput.class, "axon3enc");
        AnalogInput axon4enc = hardwareMap.get(AnalogInput.class, "axon4enc");



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double conversionFactor = ((2 * Math.PI) / 3.3);
            double axon1Heading = conversionFactor*axon1enc.getVoltage();
            double axon2Heading = conversionFactor*axon2enc.getVoltage();
            double axon3Heading = conversionFactor*axon3enc.getVoltage();
            double axon4Heading = conversionFactor*axon4enc.getVoltage();
            int servoSelector = 1;
            //switch between servos

            telemetry.addData("axon 1 heading:", axon1Heading);
            telemetry.addData("axon 2 heading:", axon2Heading);
            telemetry.addData("axon 3 heading:", axon3Heading);
            telemetry.addData("axon 4 heading:", axon4Heading);
            telemetry.update();

            }
        }
    }