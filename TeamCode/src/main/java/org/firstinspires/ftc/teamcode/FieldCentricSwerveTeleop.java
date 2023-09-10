package org.firstinspires.ftc.teamcode;

import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class FieldCentricSwerveTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor4");

        double conversionFactor = ((2 * Math.PI) / 3.3); //the conversion factor of analog encoder input to servo position

        //declare axons
        CRServo axon1 = hardwareMap.crservo.get("axon1");
        CRServo axon2 = hardwareMap.crservo.get("axon2");
        CRServo axon3 = hardwareMap.crservo.get("axon3");
        CRServo axon4 = hardwareMap.crservo.get("axon4");

        //Declare our AXON drive servos (in cont. rotation mode)

        //Declare the analog input from the encoders on these fantastic servos. 0v = 0 degrees, 3.3v = 360.
        AnalogInput axon1enc = hardwareMap.get(AnalogInput.class, "axon1enc");
        AnalogInput axon2enc = hardwareMap.get(AnalogInput.class, "axon2enc");
        AnalogInput axon3enc = hardwareMap.get(AnalogInput.class, "axon3enc");
        AnalogInput axon4enc = hardwareMap.get(AnalogInput.class, "axon4enc");


        //Defensive play needs the "no power mode" to still use power to hold its pos.
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set the OG servo home positions
        double axon1Heading;
        double axon2Heading;
        double axon3Heading;
        double axon4Heading;

        double direcpod1;
        double direcpod2;
        double direcpod3;
        double direcpod4;

        double offsetpod1 = axon1enc.getVoltage()*conversionFactor;
        double offsetpod2 = axon2enc.getVoltage()*conversionFactor;
        double offsetpod3 = axon3enc.getVoltage()*conversionFactor;
        double offsetpod4 = axon4enc.getVoltage()*conversionFactor;

        double dist1;
        double dist2;
        double dist3;
        double dist4;

        double pi = Math.PI;

        double acceptable_error = 0.07;//the acceptable error of the pointing of the pods, in radians

        double reverso1 = 1;
        double reverso2 = 1;
        double reverso3 = 1;
        double reverso4 = 1;
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        boolean OneIsReversed = false;
        boolean TwoIsReversed= false;
        boolean ThreeIsReversed= false;
        boolean FourIsReversed= false;

        int q2 = 0;
        double Q2 = 1;

        double Q1 = 0;// how much to add/subtract from the direction to be relative to 0* reference angle
        int q1 = 1;

        double state = 1;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

                double botHeading = -imu.getAngularOrientation().firstAngle; //the direction the bot is facing, used for field oriented drive

                double e = 1; //left trigger/right trigger will change the speed from default
                //low & slow mode (automatically in book it mode because SWORVE)
                if (gamepad1.left_trigger > 0.2) {
                    e = 0.5;
                }
                //The pods are in order of the quadrants of the cartesian grid :)
                //set the vectors for x, y, and θ based on the gamepad inputs
                double xvec = gamepad1.left_stick_x;
                double yvec = gamepad1.left_stick_y;
                double rotvec = gamepad1.right_stick_x; //this is the rotation vector, with the singular input of the right joystick. It is interpreted differently
                //each pod, as seen with the sqrt(2) shenanigans.

//Vector calculations that find the values which will later be fed to the motor and servo PIDs
                //vector components for pod 1
                double pod1_x_comp = xvec + (sqrt(2) * rotvec);
                double pod1_y_comp = yvec + (-sqrt(2) * rotvec);


                if (pod1_x_comp >= 0 && pod1_y_comp >= 0){//Quad. 1
                    Q1 = 0;
                }
                else if (pod1_x_comp < 0){// Quad2&3
                    Q1 = Math.PI;
                }
                else if (pod1_x_comp >= 0 && pod1_y_comp < 0){//Quad4
                    Q1 = 2*Math.PI;
                }


                if (pod1_x_comp >= 0 && pod1_y_comp >= 0){//Quad 1
                    q1 = 1;
                }
                else if(pod1_x_comp < 0 && pod1_y_comp < 0){
                    q1 = 1;
                }
                else if(pod1_x_comp < 0 && pod1_y_comp >= 0){
                    q1 = -1;
                }
                else if(pod1_x_comp >= 0 && pod1_y_comp < 0){
                    q1 = -1;
                }

                //pod1 overall vector
                double magpod1 = sqrt(pow(pod1_x_comp, 2) + (pow(pod1_y_comp, 2))); // magnitude of the overall vector for pod 1
                direcpod1 = (Q1 + q1*Math.abs((asin(pod1_y_comp/magpod1))));

//----------------------------------------------------------------------------------------------------------------
                //vector components for pod 2
                double pod2_x_comp = xvec + (sqrt(2) * rotvec);
                double pod2_y_comp = yvec + (sqrt(2) * rotvec);

                // how much to add/subtract from the direction to be relative to 0* reference angle
                if (pod2_x_comp >= 0 && pod2_y_comp >= 0){//Quad. 1
                    Q2 = 0;
                }
                else if (pod2_x_comp < 0){// Quad2&3
                    Q2 = Math.PI;
                }
                else if (pod2_x_comp >= 0 && pod2_y_comp < 0){//Quad4
                    Q2 = 2*Math.PI;
                }


                if (pod2_x_comp >= 0 && pod2_y_comp >= 0){//Quad 1
                    q2 = 1;
                }
                else if(pod2_x_comp < 0 && pod2_y_comp < 0){
                    q2 = 1;
                }
                else if(pod2_x_comp < 0 && pod2_y_comp >= 0){
                    q2 = -1;
                }
                else if(pod2_x_comp >= 0 && pod2_y_comp < 0){
                    q2 = -1;
                }

                //pod1 overall vector
                double magpod2 = sqrt(pow(pod2_x_comp, 2) + (pow(pod2_y_comp, 2))); // magnitude of the overall vector for pod 2
                direcpod2 = (Q2 + q2*Math.abs((asin(pod2_y_comp/magpod2))));
//----------------------------------------------------------------------------------------------------------------
                //vector components for pod 3
                double pod3_x_comp = xvec + (-sqrt(2) * rotvec);
                double pod3_y_comp = yvec + (sqrt(2) * rotvec);

                double Q3 = 0;// how much to add/subtract from the direction to be relative to 0* reference angle
                if (pod3_x_comp >= 0 && pod3_y_comp >= 0){//Quad. 1
                    Q3 = 0;
                }
                else if (pod3_x_comp < 0){// Quad2&3
                    Q3 = Math.PI;
                }
                else if (pod3_x_comp >= 0 && pod3_y_comp < 0){//Quad4
                    Q3 = 2*Math.PI;
                }

                int q3 = 1;
                if (pod3_x_comp >= 0 && pod3_y_comp >= 0){//Quad 1
                    q3 = 1;
                }
                else if(pod3_x_comp < 0 && pod3_y_comp < 0){
                    q3 = 1;
                }
                else if(pod3_x_comp < 0 && pod3_y_comp >= 0){
                    q3 = -1;
                }
                else if(pod3_x_comp >= 0 && pod3_y_comp < 0){
                    q3 = -1;
                }

                //pod1 overall vector
                double magpod3 = sqrt(pow(pod3_x_comp, 2) + (pow(pod3_y_comp, 2))); // magnitude of the overall vector for pod 3
                direcpod3 = (Q3 + q3*Math.abs((asin(pod3_y_comp/magpod3))));
//----------------------------------------------------------------------------------------------------------------
                //vector components for pod 4
                double pod4_x_comp = xvec + (-sqrt(2) * rotvec);
                double pod4_y_comp = yvec + (-sqrt(2) * rotvec);

                double Q4 = 0;// how much to add/subtract from the direction to be relative to 0* reference angle
                if (pod4_x_comp >= 0 && pod4_y_comp >= 0){//Quad. 1
                    Q4 = 0;
                }
                else if (pod4_x_comp < 0){// Quad2&3
                    Q4 = Math.PI;
                }
                else if (pod4_x_comp >= 0 && pod4_y_comp < 0){//Quad4
                    Q4 = 2*Math.PI;
                }

                int q4 = 1;
                if (pod4_x_comp >= 0 && pod4_y_comp >= 0){//Quad 1
                    q4 = 1;
                }
                else if(pod4_x_comp < 0 && pod4_y_comp < 0){
                    q4 = 1;
                }
                else if(pod4_x_comp < 0 && pod4_y_comp >= 0){
                    q4 = -1;
                }
                else if(pod4_x_comp >= 0 && pod4_y_comp < 0){
                    q4 = -1;
                }

                //pod1 overall vector
                double magpod4 = sqrt(pow(pod4_x_comp, 2) + (pow(pod4_y_comp, 2))); // magnitude of the overall vector for pod 4
                direcpod4 = (Q4 + q4*Math.abs((asin(pod4_y_comp/magpod4))));

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                //This next block of code will take the input from the analog encoders on the axon servos
                //and convert it into a variable that is measuring the axon position in radians



                axon1Heading = ((conversionFactor * axon1enc.getVoltage()));
                axon1Heading = axon1Heading - 6.167;
                if (axon1Heading<0){ axon1Heading += 2*pi;}


                axon2Heading = ((conversionFactor * axon2enc.getVoltage()));
                axon2Heading = axon2Heading - 3.297;
                if (axon2Heading<0){ axon2Heading += 2*pi;}


                axon3Heading = ((conversionFactor * axon3enc.getVoltage()));
                axon3Heading = axon3Heading - 2.951;
                if (axon3Heading<0){ axon3Heading += 2*pi;}


                axon4Heading = ((conversionFactor * axon4enc.getVoltage()));
                axon4Heading = axon4Heading - 6.018;
                if (axon4Heading<0){ axon4Heading += 2*pi;}


                //Time to assign each pod's hardware to the values calculated in the
                //above math. Servos will be assigned to the directions and motors will
                //get their motor.setpower() values from the magnitude

                //detecting if pods should go to the angle or angle+180 to get there faster
                //pod1


                //number wrapping
                if (direcpod1<0){
                    direcpod1 = direcpod1+(2*Math.PI);
                }
                if (direcpod1>2*Math.PI){
                    direcpod1 = direcpod1-(2*Math.PI);
                }

                if (direcpod2<0){
                    direcpod2 = direcpod2+(2*Math.PI);
                }
                if (direcpod2>2*Math.PI){
                    direcpod2 = direcpod2-(2*Math.PI);
                }

                if (direcpod3<0){
                    direcpod3 = direcpod3+(2*Math.PI);
                }
                if (direcpod3>2*Math.PI){
                    direcpod3 = direcpod3-(2*Math.PI);
                }

                if (direcpod4<0){
                    direcpod4 = direcpod4+(2*Math.PI);
                }
                if (direcpod4>2*Math.PI){
                    direcpod4 = direcpod4-(2*Math.PI);
                }
                ///////////////////   ///////////////////   ///////////////////   ///////////////////   ///////////////////   ///////////////////


                //LOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLOLO

                dist1 = Math.abs(axon1Heading - direcpod1);
                dist2 = Math.abs(axon2Heading - direcpod2);
                dist3 = Math.abs(axon3Heading - direcpod3);
                dist4 = Math.abs(axon4Heading - direcpod4);

           //servo moves on pod1
                if (direcpod1 == 0 && Math.abs(pi - axon1Heading) > Math.abs(0-axon1Heading) && axon1Heading <= pi*0.5 && axon1Heading >= acceptable_error){
                    axon1.setPower(0.3*Math.abs(axon1Heading));
                    state = 1;
                }
                else if (direcpod1 == 0 && Math.abs(pi - axon1Heading) > Math.abs(0-axon1Heading) && axon1Heading >= pi*1.5 && (Math.abs(2*pi - axon1Heading) >= acceptable_error)){
                    axon1.setPower(-0.3*Math.abs(2*pi - axon1Heading));
                    state = 2;
                }
                else if (direcpod1 == 0 && Math.abs(pi - axon1Heading) <= Math.abs(0-axon1Heading) && axon1Heading <= pi && axon1Heading > 0.5*pi && dist1 >= acceptable_error){
                    axon1.setPower(-0.3*Math.abs(pi - axon1Heading));
                    state = 3;
                }
                else if (direcpod1 == 0 && Math.abs(pi - axon1Heading) <= Math.abs(0-axon1Heading) && axon1Heading > pi && axon1Heading < 1.5*pi && dist1 >= acceptable_error){
                    axon1.setPower(0.3*Math.abs(pi - axon1Heading));
                    state = 4;
                }//                                                                                                                                                     ===========

                else if (direcpod1 == pi && Math.abs(pi - axon1Heading) > Math.abs(0-axon1Heading) && axon1Heading <= pi*0.5 && axon1Heading >= acceptable_error){
                    axon1.setPower(0.3*Math.abs(axon1Heading));
                    state = 5;
                }
                else if (direcpod1 == pi && Math.abs(pi - axon1Heading) > Math.abs(0-axon1Heading) && axon1Heading >= pi*1.5 && (Math.abs(2*pi - axon1Heading) >= acceptable_error)){
                    axon1.setPower(-0.3*Math.abs(2*pi - axon1Heading));
                    state = 6;
                }
                else if (direcpod1 == pi && Math.abs(pi - axon1Heading) <= Math.abs(0-axon1Heading) && axon1Heading <= pi && axon1Heading > 0.5*pi && dist1 >= acceptable_error){
                    axon1.setPower(-0.3*Math.abs(pi - axon1Heading));
                    state = 7;
                }
                else if (direcpod1 == pi && Math.abs(pi - axon1Heading) <= Math.abs(0-axon1Heading) && axon1Heading > pi && axon1Heading < 1.5*pi && dist1 >= acceptable_error){
                    axon1.setPower(0.3*Math.abs(pi - axon1Heading));
                    state = 8;
                }


                else if (direcpod1 != 0 && direcpod1 != pi && dist1 < 0.5*pi && dist1 > acceptable_error && axon1Heading > direcpod1) { // if distance is less than 90, heading > target
                    axon1.setPower(0.3 * Math.abs(axon1Heading - direcpod1));
                    state = 9;
                }
                else if (direcpod1 != 0 && direcpod1 != pi && dist1 < 0.5*pi && dist1 > acceptable_error && axon1Heading < direcpod1) { // if distance is less than 90, heading < target
                    axon1.setPower(-0.3 * Math.abs(axon1Heading - direcpod1));
                    state = 10;
                }

                else if (direcpod1 != 0 && direcpod1 != pi && dist1 > 1.5*pi && direcpod1 < (2*pi - acceptable_error) && axon1Heading > direcpod1){// if distance is greater than 270, heading >
                    axon1.setPower(-0.3*(2*pi - Math.abs(axon1Heading-direcpod1)));
                    state = 11;
                }
                else if (direcpod1 != 0 && direcpod1 != pi && dist1 > 1.5*pi && direcpod1 < (2*pi - acceptable_error) && axon1Heading < direcpod1) {// if distance is greater than 270 heading <
                    axon1.setPower(0.3*(2*pi - Math.abs(axon1Heading-direcpod1)));
                    state = 12;
                }

                else if (direcpod1 != 0 && direcpod1 != pi && dist1 >= 0.5*pi && dist1 < pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod1 && direcpod1 <= pi) && axon1Heading > ((direcpod1 + pi) % (2*pi)) && axon1Heading > direcpod1){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                    axon1.setPower(-0.3*(Math.abs(axon1Heading-((direcpod1 + pi) % (2*pi)))));
                    state = 19;
                }
                else if (direcpod1 != 0 && direcpod1 != pi && dist1 >= 0.5*pi && dist1 < pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod1 && direcpod1 <= pi) && axon1Heading > ((direcpod1 + pi) % (2*pi)) && axon1Heading < direcpod1){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                    axon1.setPower(0.3*(2*pi-(Math.abs(axon1Heading-((direcpod1 + pi) % (2*pi))))));
                    state = 19.5;
                }
                else if (direcpod1 != 0 && direcpod1 != pi && dist1 >= 0.5*pi && dist1 < pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) > acceptable_error && (pi < direcpod1 && direcpod1 < 2*pi) && axon1Heading > ((direcpod1 + pi) % (2*pi)) && axon1Heading < direcpod1){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                    axon1.setPower(-0.3*(2*pi-(Math.abs(axon1Heading-((direcpod1 + pi) % (2*pi))))));
                    state = 20;
                }
                else if (direcpod1 != 0 && direcpod1 != pi && dist1 >= 0.5*pi && dist1 < pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) > acceptable_error && (pi < direcpod1 && direcpod1 < 2*pi) && axon1Heading > ((direcpod1 + pi) % (2*pi)) && axon1Heading > direcpod1){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                    axon1.setPower(-0.3*(Math.abs(axon1Heading-((direcpod1 + pi) % (2*pi)))));
                    state = 20.5;
                }

                else if (direcpod1 != 0 && direcpod1 != pi && dist1 >= pi && dist1 <= 1.5*pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) > acceptable_error && axon1Heading < direcpod1){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                    axon1.setPower(-0.3*Math.abs(axon1Heading-((direcpod1 + pi) % (2*pi))));
                    state = 14;
                }
                else if (direcpod1 != 0 && direcpod1 != pi && dist1 >= pi && dist1 <= 1.5*pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) > acceptable_error && axon1Heading > direcpod1){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                    axon1.setPower(0.3*(Math.abs(axon1Heading-((direcpod1 + pi) % (2*pi)))));
                    state = 14.5;
                }


                else if (dist1 >= 0.5*pi && dist1 <= 1.5*pi && Math.abs(axon1Heading - ((direcpod1 + pi) % (2*pi))) <= acceptable_error){ //if at antitarget, chill
                    axon1.setPower(0); state = 15;
                    OneIsReversed = true;
                }
                else if (dist1 < 0.5*pi && dist1 <= acceptable_error) { // if less than 90 and at target, chill
                    axon1.setPower(0); state = 16;
                    OneIsReversed = false;
                }
                else if (direcpod1 > 1.5*pi && direcpod1 >= (2*pi - acceptable_error)){// if distance is greater than 270 buut it needs to be at meridian, chill
                    axon1.setPower(0); state = 17;
                    OneIsReversed = false;
                }
                else if (magpod1 == 0){
                    axon1.setPower(0); state = 18;
                }//pod1 code
/////////////////////////////////////////////////////////////////////
           if (direcpod2 == 0 && Math.abs(pi - axon2Heading) > Math.abs(0-axon2Heading) && axon2Heading <= pi*0.5 && axon2Heading >= acceptable_error){
                axon2.setPower(0.3*Math.abs(axon2Heading));
                state = 1;
            }
            else if (direcpod2 == 0 && Math.abs(pi - axon2Heading) > Math.abs(0-axon2Heading) && axon2Heading >= pi*1.5 && (Math.abs(2*pi - axon2Heading) >= acceptable_error)){
                axon2.setPower(-0.3*Math.abs(2*pi - axon2Heading));
                state = 2;
            }
            else if (direcpod2 == 0 && Math.abs(pi - axon2Heading) <= Math.abs(0-axon2Heading) && axon2Heading <= pi && axon2Heading > 0.5*pi && dist2 >= acceptable_error){
                axon2.setPower(-0.3*Math.abs(pi - axon2Heading));
                state = 3;
            }
            else if (direcpod2 == 0 && Math.abs(pi - axon2Heading) <= Math.abs(0-axon2Heading) && axon2Heading > pi && axon2Heading < 1.5*pi && dist2 >= acceptable_error){
                axon2.setPower(0.3*Math.abs(pi - axon2Heading));
                state = 4;
            }//                                                                                                                                                     ===========

            else if (direcpod2 == pi && Math.abs(pi - axon2Heading) > Math.abs(0-axon2Heading) && axon2Heading <= pi*0.5 && axon2Heading >= acceptable_error){
                axon2.setPower(0.3*Math.abs(axon2Heading));
                state = 5;
            }
            else if (direcpod2 == pi && Math.abs(pi - axon2Heading) > Math.abs(0-axon2Heading) && axon2Heading >= pi*1.5 && (Math.abs(2*pi - axon2Heading) >= acceptable_error)){
                axon2.setPower(-0.3*Math.abs(2*pi - axon2Heading));
                state = 6;
            }
            else if (direcpod2 == pi && Math.abs(pi - axon2Heading) <= Math.abs(0-axon2Heading) && axon2Heading <= pi && axon2Heading > 0.5*pi && dist2 >= acceptable_error){
                axon2.setPower(-0.3*Math.abs(pi - axon2Heading));
                state = 7;
            }
            else if (direcpod2 == pi && Math.abs(pi - axon2Heading) <= Math.abs(0-axon2Heading) && axon2Heading > pi && axon2Heading < 1.5*pi && dist2 >= acceptable_error){
                axon2.setPower(0.3*Math.abs(pi - axon2Heading));
                state = 8;
            }


            else if (direcpod2 != 0 && direcpod2 != pi && dist2 < 0.5*pi && dist2 > acceptable_error && axon2Heading > direcpod2) { // if distance is less than 90, heading > target
                axon2.setPower(0.3 * Math.abs(axon2Heading - direcpod2));
                state = 9;
            }
            else if (direcpod2 != 0 && direcpod2 != pi && dist2 < 0.5*pi && dist2 > acceptable_error && axon2Heading < direcpod2) { // if distance is less than 90, heading < target
                axon2.setPower(-0.3 * Math.abs(axon2Heading - direcpod2));
                state = 10;
            }

            else if (direcpod2 != 0 && direcpod2 != pi && dist2 > 1.5*pi && direcpod2 < (2*pi - acceptable_error) && axon2Heading > direcpod2){// if distance is greater than 270, heading >
                axon2.setPower(-0.3*(2*pi - Math.abs(axon2Heading-direcpod2)));
                state = 11;
            }
            else if (direcpod2 != 0 && direcpod2 != pi && dist2 > 1.5*pi && direcpod2 < (2*pi - acceptable_error) && axon2Heading < direcpod2) {// if distance is greater than 270 heading <
                axon2.setPower(0.3*(2*pi - Math.abs(axon2Heading-direcpod2)));
                state = 12;
            }

            else if (direcpod2 != 0 && direcpod2 != pi && dist2 >= 0.5*pi && dist2 < pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod2 && direcpod2 <= pi) && axon2Heading > ((direcpod2 + pi) % (2*pi)) && axon2Heading > direcpod2){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon2.setPower(-0.3*(Math.abs(axon2Heading-((direcpod2 + pi) % (2*pi)))));
                state = 19;
            }
            else if (direcpod2 != 0 && direcpod2 != pi && dist2 >= 0.5*pi && dist2 < pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod2 && direcpod2 <= pi) && axon2Heading > ((direcpod2 + pi) % (2*pi)) && axon2Heading < direcpod2){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon2.setPower(0.3*(2*pi-(Math.abs(axon2Heading-((direcpod2 + pi) % (2*pi))))));
                state = 19.5;
            }
            else if (direcpod2 != 0 && direcpod2 != pi && dist2 >= 0.5*pi && dist2 < pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) > acceptable_error && (pi < direcpod2 && direcpod2 < 2*pi) && axon2Heading > ((direcpod2 + pi) % (2*pi)) && axon2Heading < direcpod2){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon2.setPower(-0.3*(2*pi-(Math.abs(axon2Heading-((direcpod2 + pi) % (2*pi))))));
                state = 20;
            }
            else if (direcpod2 != 0 && direcpod2 != pi && dist2 >= 0.5*pi && dist2 < pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) > acceptable_error && (pi < direcpod2 && direcpod2 < 2*pi) && axon2Heading > ((direcpod2 + pi) % (2*pi)) && axon2Heading > direcpod2){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon2.setPower(-0.3*(Math.abs(axon2Heading-((direcpod2 + pi) % (2*pi)))));
                state = 20.5;
            }

            else if (direcpod2 != 0 && direcpod2 != pi && dist2 >= pi && dist2 <= 1.5*pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) > acceptable_error && axon2Heading < direcpod2){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon2.setPower(-0.3*Math.abs(axon2Heading-((direcpod2 + pi) % (2*pi))));
                state = 14;
            }
            else if (direcpod2 != 0 && direcpod2 != pi && dist2 >= pi && dist2 <= 1.5*pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) > acceptable_error && axon2Heading > direcpod2){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon2.setPower(0.3*(Math.abs(axon2Heading-((direcpod2 + pi) % (2*pi)))));
                state = 14.5;
            }


            else if (dist2 >= 0.5*pi && dist2 <= 1.5*pi && Math.abs(axon2Heading - ((direcpod2 + pi) % (2*pi))) <= acceptable_error){ //if at antitarget, chill
                axon2.setPower(0); state = 15;
                OneIsReversed = true;
            }
            else if (dist2 < 0.5*pi && dist2 <= acceptable_error) { // if less than 90 and at target, chill
                axon2.setPower(0); state = 16;
                OneIsReversed = false;
            }
            else if (direcpod2 > 1.5*pi && direcpod2 >= (2*pi - acceptable_error)){// if distance is greater than 270 buut it needs to be at meridian, chill
                axon2.setPower(0); state = 27;
                OneIsReversed = false;
            }
            else if (magpod2 == 0){
                axon2.setPower(0); state = 18;
            }//pod2 code
/////////////////////////////////////////////////////////////////////
           if (direcpod3 == 0 && Math.abs(pi - axon3Heading) > Math.abs(0-axon3Heading) && axon3Heading <= pi*0.5 && axon3Heading >= acceptable_error){
                axon3.setPower(0.3*Math.abs(axon3Heading));
                state = 1;
            }
            else if (direcpod3 == 0 && Math.abs(pi - axon3Heading) > Math.abs(0-axon3Heading) && axon3Heading >= pi*1.5 && (Math.abs(2*pi - axon3Heading) >= acceptable_error)){
                axon3.setPower(-0.3*Math.abs(2*pi - axon3Heading));
                state = 2;
            }
            else if (direcpod3 == 0 && Math.abs(pi - axon3Heading) <= Math.abs(0-axon3Heading) && axon3Heading <= pi && axon3Heading > 0.5*pi && dist3 >= acceptable_error){
                axon3.setPower(-0.3*Math.abs(pi - axon3Heading));
                state = 3;
            }
            else if (direcpod3 == 0 && Math.abs(pi - axon3Heading) <= Math.abs(0-axon3Heading) && axon3Heading > pi && axon3Heading < 1.5*pi && dist3 >= acceptable_error){
                axon3.setPower(0.3*Math.abs(pi - axon3Heading));
                state = 4;
            }//                                                                                                                                                     ===========

            else if (direcpod3 == pi && Math.abs(pi - axon3Heading) > Math.abs(0-axon3Heading) && axon3Heading <= pi*0.5 && axon3Heading >= acceptable_error){
                axon3.setPower(0.3*Math.abs(axon3Heading));
                state = 5;
            }
            else if (direcpod3 == pi && Math.abs(pi - axon3Heading) > Math.abs(0-axon3Heading) && axon3Heading >= pi*1.5 && (Math.abs(2*pi - axon3Heading) >= acceptable_error)){
                axon3.setPower(-0.3*Math.abs(2*pi - axon3Heading));
                state = 6;
            }
            else if (direcpod3 == pi && Math.abs(pi - axon3Heading) <= Math.abs(0-axon3Heading) && axon3Heading <= pi && axon3Heading > 0.5*pi && dist3 >= acceptable_error){
                axon3.setPower(-0.3*Math.abs(pi - axon3Heading));
                state = 7;
            }
            else if (direcpod3 == pi && Math.abs(pi - axon3Heading) <= Math.abs(0-axon3Heading) && axon3Heading > pi && axon3Heading < 1.5*pi && dist3 >= acceptable_error){
                axon3.setPower(0.3*Math.abs(pi - axon3Heading));
                state = 8;
            }


            else if (direcpod3 != 0 && direcpod3 != pi && dist3 < 0.5*pi && dist3 > acceptable_error && axon3Heading > direcpod3) { // if distance is less than 90, heading > target
                axon3.setPower(0.3 * Math.abs(axon3Heading - direcpod3));
                state = 9;
            }
            else if (direcpod3 != 0 && direcpod3 != pi && dist3 < 0.5*pi && dist3 > acceptable_error && axon3Heading < direcpod3) { // if distance is less than 90, heading < target
                axon3.setPower(-0.3 * Math.abs(axon3Heading - direcpod3));
                state = 10;
            }

            else if (direcpod3 != 0 && direcpod3 != pi && dist3 > 1.5*pi && direcpod3 < (2*pi - acceptable_error) && axon3Heading > direcpod3){// if distance is greater than 270, heading >
                axon3.setPower(-0.3*(2*pi - Math.abs(axon3Heading-direcpod3)));
                state = 11;
            }
            else if (direcpod3 != 0 && direcpod3 != pi && dist3 > 1.5*pi && direcpod3 < (2*pi - acceptable_error) && axon3Heading < direcpod3) {// if distance is greater than 270 heading <
                axon3.setPower(0.3*(2*pi - Math.abs(axon3Heading-direcpod3)));
                state = 12;
            }

            else if (direcpod3 != 0 && direcpod3 != pi && dist3 >= 0.5*pi && dist3 < pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod3 && direcpod3 <= pi) && axon3Heading > ((direcpod3 + pi) % (2*pi)) && axon3Heading > direcpod3){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon3.setPower(-0.3*(Math.abs(axon3Heading-((direcpod3 + pi) % (2*pi)))));
                state = 19;
            }
            else if (direcpod3 != 0 && direcpod3 != pi && dist3 >= 0.5*pi && dist3 < pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod3 && direcpod3 <= pi) && axon3Heading > ((direcpod3 + pi) % (2*pi)) && axon3Heading < direcpod3){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon3.setPower(0.3*(2*pi-(Math.abs(axon3Heading-((direcpod3 + pi) % (2*pi))))));
                state = 19.5;
            }
            else if (direcpod3 != 0 && direcpod3 != pi && dist3 >= 0.5*pi && dist3 < pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) > acceptable_error && (pi < direcpod3 && direcpod3 < 2*pi) && axon3Heading > ((direcpod3 + pi) % (2*pi)) && axon3Heading < direcpod3){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon3.setPower(-0.3*(2*pi-(Math.abs(axon3Heading-((direcpod3 + pi) % (2*pi))))));
                state = 20;
            }
            else if (direcpod3 != 0 && direcpod3 != pi && dist3 >= 0.5*pi && dist3 < pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) > acceptable_error && (pi < direcpod3 && direcpod3 < 2*pi) && axon3Heading > ((direcpod3 + pi) % (2*pi)) && axon3Heading > direcpod3){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon3.setPower(-0.3*(Math.abs(axon3Heading-((direcpod3 + pi) % (2*pi)))));
                state = 20.5;
            }

            else if (direcpod3 != 0 && direcpod3 != pi && dist3 >= pi && dist3 <= 1.5*pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) > acceptable_error && axon3Heading < direcpod3){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon3.setPower(-0.3*Math.abs(axon3Heading-((direcpod3 + pi) % (2*pi))));
                state = 14;
            }
            else if (direcpod3 != 0 && direcpod3 != pi && dist3 >= pi && dist3 <= 1.5*pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) > acceptable_error && axon3Heading > direcpod3){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon3.setPower(0.3*(Math.abs(axon3Heading-((direcpod3 + pi) % (2*pi)))));
                state = 14.5;
            }


            else if (dist3 >= 0.5*pi && dist3 <= 1.5*pi && Math.abs(axon3Heading - ((direcpod3 + pi) % (2*pi))) <= acceptable_error){ //if at antitarget, chill
                axon3.setPower(0); state = 15;
                OneIsReversed = true;
            }
            else if (dist3 < 0.5*pi && dist3 <= acceptable_error) { // if less than 90 and at target, chill
                axon3.setPower(0); state = 16;
                OneIsReversed = false;
            }
            else if (direcpod3 > 1.5*pi && direcpod3 >= (2*pi - acceptable_error)){// if distance is greater than 270 buut it needs to be at meridian, chill
                axon3.setPower(0); state = 27;
                OneIsReversed = false;
            }
            else if (magpod3 == 0){
                axon3.setPower(0); state = 18;
            }//pod3 code
/////////////////////////////////////////////////////////////////////
            //pod4 code
            if (direcpod4 == 0 && Math.abs(pi - axon4Heading) > Math.abs(0-axon4Heading) && axon4Heading <= pi*0.5 && axon4Heading >= acceptable_error){
                axon4.setPower(0.3*Math.abs(axon4Heading));
                state = 1;
            }
            else if (direcpod4 == 0 && Math.abs(pi - axon4Heading) > Math.abs(0-axon4Heading) && axon4Heading >= pi*1.5 && (Math.abs(2*pi - axon4Heading) >= acceptable_error)){
                axon4.setPower(-0.3*Math.abs(2*pi - axon4Heading));
                state = 2;
            }
            else if (direcpod4 == 0 && Math.abs(pi - axon4Heading) <= Math.abs(0-axon4Heading) && axon4Heading <= pi && axon4Heading > 0.5*pi && dist4 >= acceptable_error){
                axon4.setPower(-0.3*Math.abs(pi - axon4Heading));
                state = 3;
            }
            else if (direcpod4 == 0 && Math.abs(pi - axon4Heading) <= Math.abs(0-axon4Heading) && axon4Heading > pi && axon4Heading < 1.5*pi && dist4 >= acceptable_error){
                axon4.setPower(0.3*Math.abs(pi - axon4Heading));
                state = 4;
            }//                                                                                                                                                     ===========

            else if (direcpod4 == pi && Math.abs(pi - axon4Heading) > Math.abs(0-axon4Heading) && axon4Heading <= pi*0.5 && axon4Heading >= acceptable_error){
                axon4.setPower(0.3*Math.abs(axon4Heading));
                state = 5;
            }
            else if (direcpod4 == pi && Math.abs(pi - axon4Heading) > Math.abs(0-axon4Heading) && axon4Heading >= pi*1.5 && (Math.abs(2*pi - axon4Heading) >= acceptable_error)){
                axon4.setPower(-0.3*Math.abs(2*pi - axon4Heading));
                state = 6;
            }
            else if (direcpod4 == pi && Math.abs(pi - axon4Heading) <= Math.abs(0-axon4Heading) && axon4Heading <= pi && axon4Heading > 0.5*pi && dist4 >= acceptable_error){
                axon4.setPower(-0.3*Math.abs(pi - axon4Heading));
                state = 7;
            }
            else if (direcpod4 == pi && Math.abs(pi - axon4Heading) <= Math.abs(0-axon4Heading) && axon4Heading > pi && axon4Heading < 1.5*pi && dist4 >= acceptable_error){
                axon4.setPower(0.3*Math.abs(pi - axon4Heading));
                state = 8;
            }


            else if (direcpod4 != 0 && direcpod4 != pi && dist4 < 0.5*pi && dist4 > acceptable_error && axon4Heading > direcpod4) { // if distance is less than 90, heading > target
                axon4.setPower(0.3 * Math.abs(axon4Heading - direcpod4));
                state = 9;
            }
            else if (direcpod4 != 0 && direcpod4 != pi && dist4 < 0.5*pi && dist4 > acceptable_error && axon4Heading < direcpod4) { // if distance is less than 90, heading < target
                axon4.setPower(-0.3 * Math.abs(axon4Heading - direcpod4));
                state = 10;
            }

            else if (direcpod4 != 0 && direcpod4 != pi && dist4 > 1.5*pi && direcpod4 < (2*pi - acceptable_error) && axon4Heading > direcpod4){// if distance is greater than 270, heading >
                axon4.setPower(-0.3*(2*pi - Math.abs(axon4Heading-direcpod4)));
                state = 11;
            }
            else if (direcpod4 != 0 && direcpod4 != pi && dist4 > 1.5*pi && direcpod4 < (2*pi - acceptable_error) && axon4Heading < direcpod4) {// if distance is greater than 270 heading <
                axon4.setPower(0.3*(2*pi - Math.abs(axon4Heading-direcpod4)));
                state = 12;
            }

            else if (direcpod4 != 0 && direcpod4 != pi && dist4 >= 0.5*pi && dist4 < pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod4 && direcpod4 <= pi) && axon4Heading > ((direcpod4 + pi) % (2*pi)) && axon4Heading > direcpod4){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon4.setPower(-0.3*(Math.abs(axon4Heading-((direcpod4 + pi) % (2*pi)))));
                state = 19;
            }
            else if (direcpod4 != 0 && direcpod4 != pi && dist4 >= 0.5*pi && dist4 < pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) > acceptable_error && (0.5*pi <= direcpod4 && direcpod4 <= pi) && axon4Heading > ((direcpod4 + pi) % (2*pi)) && axon4Heading < direcpod4){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon4.setPower(0.3*(2*pi-(Math.abs(axon4Heading-((direcpod4 + pi) % (2*pi))))));
                state = 19.5;
            }
            else if (direcpod4 != 0 && direcpod4 != pi && dist4 >= 0.5*pi && dist4 < pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) > acceptable_error && (pi < direcpod4 && direcpod4 < 2*pi) && axon4Heading > ((direcpod4 + pi) % (2*pi)) && axon4Heading < direcpod4){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon4.setPower(-0.3*(2*pi-(Math.abs(axon4Heading-((direcpod4 + pi) % (2*pi))))));
                state = 20;
            }
            else if (direcpod4 != 0 && direcpod4 != pi && dist4 >= 0.5*pi && dist4 < pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) > acceptable_error && (pi < direcpod4 && direcpod4 < 2*pi) && axon4Heading > ((direcpod4 + pi) % (2*pi)) && axon4Heading > direcpod4){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon4.setPower(-0.3*(Math.abs(axon4Heading-((direcpod4 + pi) % (2*pi)))));
                state = 20.5;
            }

            else if (direcpod4 != 0 && direcpod4 != pi && dist4 >= pi && dist4 <= 1.5*pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) > acceptable_error && axon4Heading < direcpod4){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon4.setPower(-0.3*Math.abs(axon4Heading-((direcpod4 + pi) % (2*pi))));
                state = 14;
            }
            else if (direcpod4 != 0 && direcpod4 != pi && dist4 >= pi && dist4 <= 1.5*pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) > acceptable_error && axon4Heading > direcpod4){ // if angle between 90 and 270, AKA tryna go to anti target, and distance to antitarget is greater than acceptable error
                axon4.setPower(0.3*(Math.abs(axon4Heading-((direcpod4 + pi) % (2*pi)))));
                state = 14.5;
            }


            else if (dist4 >= 0.5*pi && dist4 <= 1.5*pi && Math.abs(axon4Heading - ((direcpod4 + pi) % (2*pi))) <= acceptable_error){ //if at antitarget, chill
                axon4.setPower(0); state = 15;
                OneIsReversed = true;
            }
            else if (dist4 < 0.5*pi && dist4 <= acceptable_error) { // if less than 90 and at target, chill
                axon4.setPower(0); state = 16;
                OneIsReversed = false;
            }
            else if (direcpod4 > 1.5*pi && direcpod4 >= (2*pi - acceptable_error)){// if distance is greater than 270 buut it needs to be at meridian, chill
                axon4.setPower(0); state = 27;
                OneIsReversed = false;
            }
            else if (magpod4 == 0){
                axon4.setPower(0); state = 18;
            }
            //==================================================================================================================================================================
            if (axon1Heading <= direcpod1 + (((0.5*pi) % (2*pi)))){
                OneIsReversed = false;
            }
            else if (axon1Heading >= direcpod1 - (((0.5*pi) % (2*pi)))){
                OneIsReversed = true;
            }
            //pod2
            if (axon2Heading <= direcpod2 + (((0.5*pi) % (2*pi)))){
                TwoIsReversed = false;
            }
            else if (axon1Heading >= direcpod1 - (((0.5*pi) % (2*pi)))){
                TwoIsReversed = true;
            }
            //pod3
            if (axon3Heading <= direcpod3 + (((0.5*pi) % (2*pi)))){
                ThreeIsReversed = false;
            }
            else if (axon3Heading >= direcpod3 - (((0.5*pi) % (2*pi)))){
                ThreeIsReversed = true;
            }
            //pod4
            if (axon4Heading <= direcpod4 + (((0.5*pi) % (2*pi)))){
                FourIsReversed = false;
            }
            else if (axon4Heading >= direcpod4 - (((0.5*pi) % (2*pi)))){
                FourIsReversed = true;
            }

                if (OneIsReversed) {
                    reverso1 = -1;
                } else {
                    reverso1 = 1;
                }

                if (TwoIsReversed) {
                    reverso2 = -1;
                } else {
                    reverso2 = 1;
                }

                if (ThreeIsReversed) {
                    reverso3 = -1;
                } else {
                    reverso3 = 1;
                }

                if (FourIsReversed) {
                    reverso4 = -1;
                } else {
                    reverso4 = 1;
                }
                //the actual magnitude control: setting the motor power at the end of each loop
                motorFrontLeft.setPower(reverso2 * e * magpod2);
                motorBackLeft.setPower(reverso3 * e * magpod3);
                motorFrontRight.setPower(reverso1 * e * magpod1);
                motorBackRight.setPower(reverso4 * e * magpod4);
                telemetry.addData("john", direcpod1);
                telemetry.addData("axon1heading", axon1Heading);
                telemetry.addData("axon2heading", axon2Heading);
                telemetry.addData("axon3heading", axon3Heading);
                telemetry.addData("axon4heading", axon4Heading);
                telemetry.addData("direcpod1", direcpod1);
                telemetry.addData("direcpod2", direcpod2);
                telemetry.addData("direcpod3", direcpod3);
                telemetry.addData("direcpod4", direcpod4);
                telemetry.addData("Is reversed?", OneIsReversed);
                telemetry.addData("magpod1", magpod1);
                telemetry.addData("state", state);
                telemetry.update();
            }
        }
    }
