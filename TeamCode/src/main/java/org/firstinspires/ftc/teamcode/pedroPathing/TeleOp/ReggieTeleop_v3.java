package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import android.provider.ContactsContract;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.DataStorage;
@Disabled
/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@TeleOp(name = "Reggie_TeleOp_Regionals", group = "Regionals Reggie")
public class ReggieTeleop_v3 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
//    private ColorSensor colorSensorLeft;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    public HuskyLens huskyLens;
//    private ColorSensor colorSensorRight;
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose = DataStorage.currentPose;
    public static Pose scoringPose;

    public enum ScoringState {
        IDLE,
        ACCELERATING_FAR,
        ACCELERATING_NEAR,
        SHOOTING_RIGHT,
        SHOOTING_LEFT,
        COOLING_DOWN
    }

    ScoringState artifactScoringState = ScoringState.IDLE;
    int longDistanceVelocity = 1600;
    int midDistanceVelocity = 1300;
    int closeDistanceVelocity = 1150;

    double velocityMultiplier = 8.6;
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
public boolean shooting;
    public static double shootingAngleFirst = 66;

    private final Pose scorePose = new Pose(83, 28, Math.toRadians(shootingAngleFirst)); // Highest (First Set) of Artifacts from the Spike Mark.

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        //Pedro Pathing
//        Constants.setConstants(FConstants.class, LConstants.class);

        Miller = new Reggie();
        Miller.init(hardwareMap);
        Miller.usingPIDF = true;
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(DataStorage.currentPose ==null ? new Pose() : DataStorage.currentPose);
        follower.update();
        shootingTime = new ElapsedTime();
        playTime = new ElapsedTime();
        inEndgame = false;
        shooting = false;

//        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
//        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addData("Starting X:",DataStorage.currentPose);

        telemetry.addData("Starting X:",follower.getPose().getX());
        telemetry.addData("Starting Y:",follower.getPose().getY());
        telemetry.addData("Starting Heading:",follower.getPose().getHeading());
        telemetry.update();

        // playTime.reset();
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
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
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);



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

        //Flywheels
        /*if (gamepad1.square) {
            Miller.TARGET_MIN_VELOCITY_LEFT = longDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = longDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
//            Miller.setShooterPower(longDistancePower);
        } else if (gamepad1.triangle) {

            Miller.TARGET_MIN_VELOCITY_LEFT = midDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = midDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
//            Miller.setShooterPower(midDistancePower);
        } else if (gamepad1.circle) {
            Miller.TARGET_MIN_VELOCITY_LEFT = closeDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = closeDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
            /*if(range>0){
                Miller.TARGET_MIN_VELOCITY_LEFT = (range * velocityMultiplier) - 100;
                Miller.TARGET_VELOCITY_LEFT = range*velocityMultiplier;
                Miller.setShooterVelocity(Reggie.SIDES.LEFT);
                shooting = true;


//            Miller.setShooterVelocity(closeDistanceVelocity);
        } else if (gamepad1.cross) {
            Miller.setShooterPower(0);
            shooting = false;

        }

        if(range>0 && shooting){
            Miller.TARGET_MIN_VELOCITY_LEFT = (range * velocityMultiplier) - 100;
            Miller.TARGET_VELOCITY_LEFT = range*velocityMultiplier;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
        }
        */
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

        if(gamepad1.psWasPressed()){
            Miller.usingPIDF = !Miller.usingPIDF;
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

        telemetry.addData("Side", Side);

        telemetry.addData("Play Time: ", playTime.seconds());
        if (artifactScoringState == ScoringState.ACCELERATING_FAR) {
            telemetry.addData("Its Miller Time - ", "Shooting from Far Away!");
        }
        telemetry.addData("Current State:", artifactScoringState.toString());
        telemetry.addData("Using PIDF:", Miller.usingPIDF);
        telemetry.addData("bearing:", bearing);
        telemetry.addData("range:", range);


        telemetry.addData("Left Blue:",lBlue);
        telemetry.addData("Left Green:",lGreen);
        telemetry.addData("Left Red:",lRed);
        telemetry.addData("Right Blue:",rBlue);
        telemetry.addData("Right Green:",rGreen);
        telemetry.addData("Right Red:",rRed);
        telemetry.addData("Left Stopper Position:",Miller.leftStopper.getPosition());
        telemetry.addData("Right Stopper Position:",Miller.rightStopper.getPosition());
        telemetry.update();
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
