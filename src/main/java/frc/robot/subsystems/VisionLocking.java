// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmPathFactory;
import frc.robot.constants.FieldConstants;

import java.util.Arrays;
import java.util.Map;

public class VisionLocking extends SubsystemBase implements SubsystemLogging{

    public enum Team {
        RED, BLUE
    }
    public enum Level {
        HIGH, MID, LOW
    }
    public enum Side {
        LEFT, RIGHT
    }

    public enum PieceType {
        CONES, CUBES
    }
    private Team m_team;
    private Level m_level;
    private Side m_side;
    private int m_grid;
    private PieceType m_pieceType;
    private static final int[] blueTags = {8,7,6};
    private static final int[] redTags = {3,2,1};
    private final ShuffleboardTab driverTab;
    private final ShuffleboardLayout gridLocationLayout;
    private final ShuffleboardLayout gridSelectionLayout;
    private boolean[][] gridLocation;
    private final boolean[] gridSelection;
    private boolean coneCube;

    /** Creates a new VisionLocking. */
    public VisionLocking() {
        m_team = Team.BLUE;
        m_pieceType = PieceType.CONES;
        m_grid = 1;
        m_side = Side.LEFT;
        m_level = Level.HIGH;

        // Shuffleboard Initialization
        driverTab = Shuffleboard.getTab("DriverView");
        gridLocationLayout = driverTab.getLayout("Location in Grid", BuiltInLayouts.kGrid).withSize(3, 3).withPosition(0, 0);
        gridSelectionLayout = driverTab.getLayout("Grid Selection", BuiltInLayouts.kGrid).withSize(3, 1).withPosition(0, 3);
        gridLocation = new boolean[3][3];
        gridSelection = new boolean[3];
        coneCube = false;

        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            m_team = VisionLocking.Team.BLUE;
        } else {
            m_team = VisionLocking.Team.RED;
        }

        configureDashboard();
    }

    public void configureDashboard(){
        // Grid selection layout config
        Map selectProperties = Map.of("colorWhenFalse", "#000000", "colorWhenTrue", "#7cfc00");
        gridSelectionLayout.addBoolean("Left", () -> gridSelection[0]).withProperties(selectProperties).withPosition(0, 0);
        gridSelectionLayout.addBoolean("Center", () -> gridSelection[1]).withProperties(selectProperties).withPosition(1, 0);
        gridSelectionLayout.addBoolean("Right", () -> gridSelection[2]).withProperties(selectProperties).withPosition(2, 0);

        // Grid location layout config
        Map coneSlotProperties = Map.of("colorWhenFalse", "#000000", "colorWhenTrue", "#5D3FD3");
        Map cubeSlotProperties = Map.of("colorWhenFalse", "#000000", "colorWhenTrue", "#FDDA0D");
        // Cube slots
        gridLocationLayout.addBoolean("TL", () -> gridLocation[0][0]).withProperties(cubeSlotProperties).withPosition(0, 0);
        gridLocationLayout.addBoolean("ML", () -> gridLocation[0][1]).withProperties(cubeSlotProperties).withPosition(0, 1);
        gridLocationLayout.addBoolean("BL", () -> gridLocation[0][2]).withProperties(cubeSlotProperties).withPosition(0, 2);

        gridLocationLayout.addBoolean("TR", () -> gridLocation[2][0]).withProperties(cubeSlotProperties).withPosition(2, 0);
        gridLocationLayout.addBoolean("MR", () -> gridLocation[2][1]).withProperties(cubeSlotProperties).withPosition(2, 1);
        gridLocationLayout.addBoolean("BR", () -> gridLocation[2][2]).withProperties(cubeSlotProperties).withPosition(2, 2);
        // Cone slots
        gridLocationLayout.addBoolean("TM", () -> gridLocation[1][0]).withProperties(coneSlotProperties).withPosition(1, 0);
        gridLocationLayout.addBoolean("MM", () -> gridLocation[1][1]).withProperties(coneSlotProperties).withPosition(1, 1);
        gridLocationLayout.addBoolean("BM", () -> gridLocation[1][2]).withProperties(coneSlotProperties).withPosition(1, 2);

        // Cone/cube selector
        driverTab.addBoolean("Cone-Cube", () -> coneCube).withProperties(Map.of("colorWhenFalse", "#FDDA0D", "colorWhenTrue", "#5D3FD3")).withPosition(3, 0).withSize(1, 4);

        // Initialize the two boolean arrays to have base values.
        updateGridArray();
        updateLocationArray();
    }

    public void setSide(Side setTo){
        m_side = setTo;
        updateLocationArray();
    }
    public void setGrid(int setTo){
        m_grid = setTo;
    }
    public void setPieceType(PieceType setTo){
        m_pieceType = setTo;
    }

    public Team getTeam(){
        return m_team;
    }
    public PieceType getPieceType(){
        return m_pieceType;
    }

    public void gridRight(){
        if (m_grid < 2) {
            m_grid += 1;
        }
        updateGridArray();
    }
    public void gridLeft(){
        if (m_grid > 0) {
            m_grid -= 1;
        }
        updateGridArray();
    }

    public void levelUp(){
        if (m_level.equals(Level.LOW)){
            m_level = Level.MID;
        } else if(m_level.equals(Level.MID)) {
            m_level = Level.HIGH;
        }
        updateLocationArray();
    }

    public void levelDown(){
        if (m_level.equals(Level.HIGH)){
            m_level = Level.MID;
        } else if(m_level.equals(Level.MID)) {
            m_level = Level.LOW;
        }
        updateLocationArray();
    }

    public void togglePiece(){
        if (m_pieceType.equals(PieceType.CONES)) {
            m_pieceType = PieceType.CUBES;
        } else {
            m_pieceType = PieceType.CONES;
        }
        updatePieceType();
        updateLocationArray();
    }

    public static Pose2d exit(Team team, int grid){
        Pose2d position;
        if (team.equals(Team.BLUE)){
            position = FieldConstants.aprilTags.get(blueTags[grid - 1]).toPose2d();
            position.transformBy(new Transform2d(new Translation2d(2.52, 0), new Rotation2d())); //.28
        } else {
            position = FieldConstants.aprilTags.get(redTags[grid - 1]).toPose2d();
            position.transformBy(new Transform2d(new Translation2d(-2.52, 0), new Rotation2d()));
        }

        return position;
    }


    public static Pose2d getPosition(Team team, int grid, PieceType piece, Side side){
        Pose2d position = FieldConstants.aprilTags.get(1).toPose2d();
        double xVal = 0.95;
        double yVal = 0.6;

        if(piece.equals(PieceType.CUBES)){
            yVal = 0;
        }
        if (team.equals(Team.BLUE)) {
            //position = FieldConstants.aprilTags.get(blueTags[grid - 1]).toPose2d();
            xVal *= -1;
            if(piece.equals(PieceType.CONES)){
                 if(side.equals(Side.LEFT)) yVal *= -1;
            }
        } else {
            //position = FieldConstants.aprilTags.get(redTags[grid - 1]).toPose2d();
            if(piece.equals(PieceType.CONES)){
                if(!side.equals(Side.LEFT)) yVal *= -1;
            }
        }

        return position.transformBy(new Transform2d(new Translation2d(xVal, yVal), new Rotation2d()));
    }

    /**
     * @return position
     */

    public Pose2d getLockedPosition() {
        return getPosition(m_team, m_grid, m_pieceType, m_side);
    }

    public Command getArmCommand(SwerveDrive m_swerveDrive, GravityClawSubsystem gravityClawSubsystem, Arm m_Arm, Intake intake, Indexer m_indexer) {
        if (m_level.equals(Level.HIGH)) {
            return ArmPathFactory.getScoreHighPath(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer);
        } else if (m_level.equals(Level.MID)){
            return ArmPathFactory.getScoreMidPath(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer);
        }
        return new WaitCommand(0);
    }


    public void updateLocationArray(){
        gridLocation = new boolean[3][3];
        int x = 0;
        if(m_pieceType.equals(PieceType.CUBES)){
            x = 1;
        } else if(m_side.equals(Side.RIGHT)){
            x = 2;
        }
        gridLocation[x][m_level.ordinal()] = true;
    }

    public void updateGridArray(){
        Arrays.fill(gridSelection, false);
        gridSelection[m_grid] = true;
    }

    public void updatePieceType(){
        switch(m_pieceType){
            case CONES:
                coneCube = false;
                break;
            case CUBES:
                coneCube = true;
                break;
        }
    }

    @Override
    public void periodic() {




        updateLogging();
    }

    @Override
    public void updateLogging() {
        log("Locked Pose", getLockedPosition());
    }
}