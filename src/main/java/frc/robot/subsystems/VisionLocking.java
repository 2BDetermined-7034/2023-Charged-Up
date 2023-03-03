// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;

import java.util.Arrays;
import java.util.Map;

public class VisionLocking extends SubsystemBase implements SubsystemLogging {

    private final int[] blueTags = {8, 7, 6};
    private final int[] redTags = {3, 2, 1};
    private final ShuffleboardTab driverTab;
    private final ShuffleboardLayout gridLocationLayout;
    private final ShuffleboardLayout gridSelectionLayout;
    private Team m_team;
    private Level m_level;
    private Side m_side;
    private int m_grid;
    private PieceType m_pieceType;
    private boolean[][] gridLocation;
    private final boolean[] gridSelection;
    private boolean coneCube;
    /**
     * Creates a new VisionLocking.
     */
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

        configureDashboard();
        updateLogging();
    }

    public void configureDashboard() {
        // Grid selection layout config
        Map selecProperties = Map.of("colorWhenFalse", "#000000", "colorWhenTrue", "#7cfc00");
        gridSelectionLayout.addBoolean("Left", () -> gridSelection[0]).withProperties(selecProperties).withPosition(0, 0);
        gridSelectionLayout.addBoolean("Center", () -> gridSelection[1]).withProperties(selecProperties).withPosition(1, 0);
        gridSelectionLayout.addBoolean("Right", () -> gridSelection[2]).withProperties(selecProperties).withPosition(2, 0);

        // Grid location layout config
        Map coneSlotProperties = Map.of("colorWhenFalse", "#000000", "colorWhenTrue", "#FDDA0D");
        Map cubeSlotProperties = Map.of("colorWhenFalse", "#000000", "colorWhenTrue", "#5D3FD3");

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

    public void setSide(Side setTo) {
        m_side = setTo;
        updateLocationArray();
    }

    public void setGrid(int setTo) {
        m_grid = setTo;
    }

    public Team getTeam() {
        return m_team;
    }

    public void setTeam(Team setTo) {
        m_team = setTo;
    }

    public PieceType getPieceType() {
        return m_pieceType;
    }

    public void setPieceType(PieceType setTo) {
        m_pieceType = setTo;
    }

    public int setGrid() {
        return m_grid;
    }

    public Side setSide() {
        return m_side;
    }

    public Level getLevel() {
        return m_level;
    }

    public void setLevel(Level setTo) {
        m_level = setTo;
    }

    public void gridRight() {
        if (m_grid < 2) {
            m_grid += 1;
        }
        updateGridArray();
    }

    public void gridLeft() {
        if (m_grid > 0) {
            m_grid -= 1;
        }
        updateGridArray();
    }

    public void levelUp() {
        if (m_level.equals(Level.LOW)) {
            m_level = Level.MID;
        } else if (m_level.equals(Level.MID)) {
            m_level = Level.HIGH;
        }
        updateLocationArray();
    }

    public void levelDown() {
        if (m_level.equals(Level.HIGH)) {
            m_level = Level.MID;
        } else if (m_level.equals(Level.MID)) {
            m_level = Level.LOW;
        }
        updateLocationArray();
    }

    public void toggleSide() {
        if (m_side.equals(Side.LEFT)) {
            m_side = Side.RIGHT;
        } else {
            m_side = Side.LEFT;
        }
        updateLocationArray();
    }

    public void togglePiece() {
        if (m_pieceType.equals(PieceType.CONES)) {
            m_pieceType = PieceType.CUBES;
        } else {
            m_pieceType = PieceType.CONES;
        }
        updatePieceType();
        updateLocationArray();
    }

    /**
     * Returns the position the robot must be within a certain degree of error of to score on the Grid
     * <p>
     * TODO fix to apply for both alliance colors
     * <p>
     * TODO Add more shit for substation loading
     *
     * @return position
     */

    public Pose2d getLockedPosition() {
        Pose2d position;

        if (m_team.equals(Team.BLUE)) {
            position = FieldConstants.aprilTags.get(blueTags[m_grid - 1]).toPose2d();
            position.transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(0)), new Rotation2d()));
        } else {
            position = FieldConstants.aprilTags.get(redTags[m_grid - 1]).toPose2d();
            position.transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(0), 0), new Rotation2d()));
        }
        //return position;
        return new Pose2d(new Translation2d(2.1, 1), new Rotation2d(179));
    }

    public void updateLocationArray() {
        gridLocation = new boolean[3][3];
        int x = 0;
        if (m_pieceType.equals(PieceType.CUBES)) {
            x = 1;
        } else if (m_side.equals(Side.RIGHT)) {
            x = 2;
        }
        gridLocation[x][m_level.ordinal()] = true;
    }

    public void updateGridArray() {
        Arrays.fill(gridSelection, false);
        gridSelection[m_grid] = true;
    }

    public void updatePieceType() {
        switch (m_pieceType) {
            case CONES:
                coneCube = false;
                break;
            case CUBES:
                coneCube = true;
                break;
        }
    }

    @Override
    public void updateLogging() {
        log("Grid", m_grid);
        log("Piece", m_pieceType.getModeVal() == 0 ? "Cube" : "Cone");
        log("Side", m_side.getModeVal() == 0 ? "Left" : "Right");
        log("Level", m_level.getModeVal() == 0 ? "High" : (m_level.getModeVal() == 1 ? "Mid" : "Low"));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateLogging();
    }

    public enum Team {
        RED, BLUE
    }

    public enum Level {
        HIGH(0), MID(1), LOW(2);

        private final int modeVal;

        Level(int val) {
            modeVal = val;
        }

        public int getModeVal() {
            return modeVal;
        }
    }

    public enum Side {
        LEFT(0), RIGHT(1);
        private final int modeVal;

        Side(int val) {
            modeVal = val;
        }

        public int getModeVal() {
            return modeVal;
        }
    }


    public enum PieceType {
        CONES(0), CUBES(1);

        private final int modeVal;

        PieceType(int val) {
            modeVal = val;
        }

        public int getModeVal() {
            return modeVal;
        }
    }
}