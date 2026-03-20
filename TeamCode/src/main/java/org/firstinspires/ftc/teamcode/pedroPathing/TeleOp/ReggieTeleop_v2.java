package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;

import org.firstinspires.ftc.teamcode.pedroPathing.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@TeleOp(name = "Reggie_TeleOP2", group = "LM3/Tournament Reggie")
public class ReggieTeleop_v2 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
//    private ColorSensor colorSensorLeft;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    public HuskyLens huskyLens;
//    private ColorSensor colorSensorRight;
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose = Reggie.poseFromAuto;
    public static Pose scoringPose = Reggie.scoringPose;
    public PathChain toPickup, pickupToShoot;

    public enum ScoringState {
        IDLE,
        ACCELERATING_FAR,
        ACCELERATING_NEAR,
        SHOOTING_RIGHT,
        SHOOTING_LEFT,
        COOLING_DOWN
    }

    ScoringState artifactScoringState = ScoringState.IDLE;
    int longDistanceVelocity = 1400;
    int midDistanceVelocity = 1300;
    int closeDistanceVelocity = 1150;

    double velocityMultiplier = 1150;
    int longDistancePower = 90;
    int midDistancePower = 68;
    int closeDistancePower = 60;

    int shootingState = 0;
    public boolean inEndgame;
    public ElapsedTime shootingTime, playTime;
    double sortPositionMiddle =0.5 ;
    double sortPositionRight = 0.05;
    double sortPositionLeft = 0.9;
    int Side = 0;
    int rRed;
    int rGreen;
    int rBlue;
    int lRed;
    int lGreen;
    int lBlue;

    public double range, bearing;
    public void setPathState(int pState) {
        farState = pState;
    }

    public Pose intake;
    public Pose shoot;


public boolean shooting;
    private int farState;
    public int currentState = 0;
    public boolean indexerStop;


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        //Pedro Pathing
//        Constants.setConstants(FConstants.class, LConstants.class);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(DataStorage.currentPose);
        Miller = new Reggie();
        Miller.init(hardwareMap);
        Miller.usingPIDF = true;
        shootingTime = new ElapsedTime();
        playTime = new ElapsedTime();
        inEndgame = false;
        shooting = false;

//        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
//        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        aprilTagWebcam.init(hardwareMap, telemetry);





        // playTime.reset();
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    public void buildPaths() {

        /* Drive to read the obelisk */


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        playTime.reset();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        /**Pedro Pathing Driving
         *
         */
        if (!DataStorage.Blue && !DataStorage.FAR){
             intake = DataStorage.RedIntake;
            shoot = DataStorage.RedShoot;
        }else if(DataStorage.FAR){
            intake = DataStorage.RedIntakeFAR;
            shoot = DataStorage.RedShootFAR;
        }
        else{
            intake = DataStorage.BlueIntake;
            shoot = DataStorage.BlueShoot;
        }

        follower.update();
        Pose currentPose1 = new Pose(follower.getPose().getX(),follower.getPose().getY(),Math.toRadians(follower.getPose().getHeading()));

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        toPickup = follower.pathBuilder()
                .addPath(new BezierLine(currentPose1, intake))
                .setConstantHeadingInterpolation(intake.getHeading())
                .build();
        pickupToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake, shoot))
                .setConstantHeadingInterpolation(shoot.getHeading())
                .build();

//        lBlue = colorSensorLeft.blue();
//        lGreen = colorSensorLeft.green();
//        lRed = colorSensorLeft.red();
//        rBlue = colorSensorRight.blue();
//        rGreen = colorSensorRight.green();
//        rRed = colorSensorRight.red();




        /*
        switch (artifactScoringState){
            case IDLE:
                        if(gamepad1.left_bumper && shootingState==0){
                            shootingState=1;
                            Miller.setShooterPower(90,Miller.leftShooterMotor);
                            Miller.setShooterPower(90,Miller.rightShooterMotor);
                            setScoringState(ScoringState.ACCELERATING_FAR);


                        } else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                        if(gamepad1.right_bumper && shootingState==0){
                            shootingState=1;
                            Miller.setShooterPower(80,Miller.leftShooterMotor);
                            Miller.setShooterPower(80,Miller.rightShooterMotor);
                            setScoringState(ScoringState.ACCELERATING_NEAR);


                        } else if (!gamepad1.right_bumper) {
                            shootingState = 0;
                        }
                        break;
            case ACCELERATING_FAR:
                        if(gamepad1.left_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(90,Miller.leftIndexerServo);
                            setScoringState(ScoringState.SHOOTING_LEFT);
                        }else if(gamepad1.right_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(90,Miller.rightIndexerServo);
                            setScoringState(ScoringState.SHOOTING_RIGHT);
                        }else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                            shootingState = 0;
                        }
                        break;
            case ACCELERATING_NEAR:
                        if(gamepad1.left_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(80,Miller.leftIndexerServo);
                            setScoringState(ScoringState.SHOOTING_LEFT);
                        }else if(gamepad1.right_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(80,Miller.rightIndexerServo);
                            setScoringState(ScoringState.SHOOTING_RIGHT);
                        }else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                            shootingState = 0;
                        }
                        break;
            case SHOOTING_LEFT:
                        if(shootingTime.seconds()>1.0 && gamepad1.left_bumper && shootingState==0){
                            Miller.indexerPower(0.0,Miller.leftIndexerServo);
                            Miller.setShooterPower(0.0,Miller.leftShooterMotor);
                            Miller.setShooterPower(0.0,Miller.rightShooterMotor);
                            shootingState=0;
                            setScoringState(ScoringState.IDLE);
                        }else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                break;

        }
        */

        //Intake
        if (gamepad1.left_trigger > 0.125) {
            Miller.setIntakePower(-gamepad1.left_trigger);


        } else if (gamepad1.right_trigger > 0.125) {
            Miller.setIntakePower(gamepad1.right_trigger);
            Miller.setStoppers(true,true);
            //Miller.setSorterServoPosition(.7);
        } else {
            Miller.setIntakePower(0.0);
        }




        //Flywheels
        if (currentState == 1 && !indexerStop) {
            Miller.indexerPower(30, Miller.leftIndexerServo);
            Miller.indexerPower(30, Miller.rightIndexerServo);
        }else if (currentState == 2 && indexerStop) {
            Miller.indexerPower(0, Miller.leftIndexerServo);
            Miller.indexerPower(0, Miller.rightIndexerServo);
        }else{
            //Indexers Code
            if (gamepad1.right_bumper) {
                Miller.indexerPower(80, Miller.rightIndexerServo);
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.setSorterServoPosition(Miller.sortPositionRight);
                Miller.setStoppers(false,true);
                Miller.setIntakePower(0.4);
            } else if (gamepad1.left_bumper) {
                Miller.indexerPower(80, Miller.leftIndexerServo);
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.setSorterServoPosition(Miller.sortPositionLeft);
                Miller.setStoppers(true,false);
                Miller.setIntakePower(0.4);
            } else {
                Miller.indexerPower(0, Miller.leftIndexerServo);
                Miller.indexerPower(0, Miller.rightIndexerServo);
                Miller.setSorterServoPosition(sortPositionMiddle);
            }


        }
        if (gamepad1.squareWasPressed()) {
            if(currentState == 0){
                indexerStop = false;
                Miller.leftIndexerServo.setDirection(DcMotorSimple.Direction.FORWARD);
                Miller.rightIndexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
                follower.followPath(toPickup);
                Miller.setStoppers(false,false);
                Miller.setShooterPower(-20);
                currentState = 1;
            }else if (currentState == 1){

                indexerStop = true;
                currentState = 2;
            }
            else if (currentState == 2){
                Miller.leftIndexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
                Miller.rightIndexerServo.setDirection(DcMotorSimple.Direction.FORWARD);
                Miller.setShooterPower(0);
                Miller.setStoppers(true,true);
                follower.followPath(pickupToShoot);
                Miller.TARGET_MIN_VELOCITY_LEFT = longDistanceVelocity - 100;
                Miller.TARGET_VELOCITY_LEFT = longDistanceVelocity + 10 ;
                Miller.setShooterVelocity(Reggie.SIDES.LEFT);

                currentState = 3;
            }
            else{
                Miller.setShooterPower(0);
                follower.breakFollowing();
                follower.startTeleopDrive();
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                currentState = 0;
            }

        } else if (gamepad1.triangle) {
            Miller.TARGET_MIN_VELOCITY_LEFT = longDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = longDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
        } else if (gamepad1.circle) {
            Miller.TARGET_MIN_VELOCITY_LEFT = closeDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = closeDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
           /* if(range>0){
                Miller.TARGET_MIN_VELOCITY_LEFT = (Math.sqrt(range) * velocityMultiplier) - 50;
            Miller.TARGET_VELOCITY_LEFT = (Math.sqrt(range) * velocityMultiplier);
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                shooting = true;
            }*/

//            Miller.setShooterVelocity(closeDistanceVelocity);
        } else if (gamepad1.cross) {
            Miller.setShooterPower(0);
            //shooting = false;

        }
        /*
        if(range>0 && shooting){
            Miller.TARGET_MIN_VELOCITY_LEFT = (Math.sqrt(range) * velocityMultiplier) - 50;
            Miller.TARGET_VELOCITY_LEFT = (Math.sqrt(range) * velocityMultiplier);
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
        }*/

        //Sorter
        if (gamepad1.dpad_left) {
            Miller.setSorterServoPosition(Miller.sortPositionLeft);
        } else if (gamepad1.dpad_right) {
            Miller.setSorterServoPosition(Miller.sortPositionRight);
        } else if (gamepad1.dpad_down) {
            Miller.setSorterServoPosition(sortPositionMiddle);
        }

        //General code for both options (1 or 2 Players)
        if (playTime.seconds() > 40 && !inEndgame) {
            //gamepad1.rumble(3000);
            //gamepad2.rumble(3000);
            endgameLED();
            inEndgame = true;
        }

       /* if(gamepad1.psWasPressed()){
            Pose currentPose = follower.getPose();
             PathChain ScoringPath;
            ScoringPath = follower.pathBuilder().
                            addPath(new BezierLine(currentPose, scoringPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(),scoringPose.getHeading())
                                            .build();
            follower.followPath(ScoringPath, 0.85,true);
        }*/

        /*
        if(gamepad1.dpad_down){
            Miller.leftStopper.setPosition(Miller.leftStopper.getPosition()-0.01);
            Miller.rightStopper.setPosition(Miller.rightStopper.getPosition()-0.01);
        }else if(gamepad1.dpad_up){
            Miller.leftStopper.setPosition(Miller.leftStopper.getPosition()+0.01);
            Miller.rightStopper.setPosition(Miller.rightStopper.getPosition()+0.01);
        }
        */

        /*else if(gamepad1.dpad_up){
            Miller.setStoppers(false, true);
        }else if(gamepad1.dpad_left){
            Miller.setStoppers(true, false);
        }else if(gamepad1.dpad_right){
            Miller.setStoppers(true, true);
        }*/

        if(gamepad1.psWasPressed()){
            //Miller.usingPIDF = !Miller.usingPIDF;
            Miller.parking();
        }

        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        if (id20 != null) {
             bearing = id20.ftcPose.bearing;

             range = id20.ftcPose.range;
        }else{
            range=0;
        }

        telemetry.addData("Blue",DataStorage.Blue);
        telemetry.addData("indexerStop", indexerStop);

        telemetry.addData("Play Time: ", playTime.seconds());
        if (artifactScoringState == ScoringState.ACCELERATING_FAR) {
            telemetry.addData("Its Miller Time - ", "Shooting from Far Away!");
        }
        telemetry.addData("Current State:", artifactScoringState.toString());
        telemetry.addData("Using PIDF:", Miller.usingPIDF);
        telemetry.addData("bearing:", bearing);
        telemetry.addData("range:", range);
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", follower.getPose().getHeading());


        telemetry.addData("Left Blue:",lBlue);
        telemetry.addData("Left Green:",lGreen);
        telemetry.addData("Left Red:",lRed);
        telemetry.addData("Right Blue:",rBlue);
        telemetry.addData("Right Green:",rGreen);
        telemetry.addData("Right Red:",rRed);
        telemetry.addData("Left Stopper Position:",Miller.leftStopper.getPosition());
        telemetry.addData("Right Stopper Position:",Miller.rightStopper.getPosition());
        telemetry.addData("CurrentState",currentState);
        telemetry.addData("current X", currentPose1.getPose().getX());
        telemetry.addData("current Y", currentPose1.getPose().getY());
        telemetry.addData("current Heading", currentPose1.getPose().getHeading());

    }


    public void endgameLED () {
        gamepad1.setLedColor(255, 0, 0, 500);
        gamepad2.setLedColor(0, 0, 255, 500);
    }
    public void setScoringState (ScoringState pState){
        artifactScoringState = pState;
        shootingTime.reset();

    }

}
