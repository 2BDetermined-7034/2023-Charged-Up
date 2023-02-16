// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;

public class VisionLocking extends SubsystemBase {

    private static final int[] blueTags = {8, 7, 6};
    private static final int[] redTags = {3, 2, 1};
    private Team m_team;
    private Level m_level;
    private Side m_side;
    private int m_grid;
    private PieceType m_pieceType;
    /**
     * Creates a new VisionLocking.
     */
    public VisionLocking() {
        m_team = Team.BLUE;
        m_pieceType = PieceType.CONES;
        m_grid = 1;
        m_side = Side.LEFT;
        m_level = Level.HIGH;
    }

    public void setSide(Side setTo) {
        m_side = setTo;
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

    public void toggleGrid() {
        if (m_grid < 3) {
            m_grid += 1;
        } else {
            m_grid = 1;
        }
    }

    public void toggleLevel() {
        if (m_level.equals(Level.LOW)) {
            m_level = Level.MID;
        } else if (m_level.equals(Level.MID)) {
            m_level = Level.HIGH;
        } else {
            m_level = Level.LOW;
        }
    }

    public void toggleSide() {
        if (m_side.equals(Side.LEFT)) {
            m_side = Side.RIGHT;
        } else {
            m_side = Side.LEFT;
        }
    }

    public void togglePiece() {
        if (m_pieceType.equals(PieceType.CONES)) {
            m_pieceType = PieceType.CUBES;
        } else {
            m_pieceType = PieceType.CONES;
        }
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
        Pose2d position;

        if (team.equals(Team.BLUE)) {
            position = FieldConstants.aprilTags.get(blueTags[grid - 1]).toPose2d();
            if (piece.equals(PieceType.CUBES)){
                position.transformBy(new Transform2d(new Translation2d(1.1, 0), new Rotation2d()));
            } else if (piece.equals(PieceType.CONES)&&side.equals(Side.LEFT)) {
                position.transformBy(new Transform2d(new Translation2d(1.1 , -.6), new Rotation2d()));
            } else {
                position.transformBy(new Transform2d(new Translation2d(1.1 , .6), new Rotation2d()));
            }

        } else {
            position = FieldConstants.aprilTags.get(redTags[grid - 1]).toPose2d();
            if (piece.equals(PieceType.CUBES)){
                position.transformBy(new Transform2d(new Translation2d(-1.1, 0), new Rotation2d()));
            } else if (piece.equals(PieceType.CONES)&&side.equals(Side.LEFT)) {
                position.transformBy(new Transform2d(new Translation2d(-1.1 , -.6), new Rotation2d()));
            } else {
                position.transformBy(new Transform2d(new Translation2d(-1.1 , .6), new Rotation2d()));
            }

        }
        return position;
    }

    /**
     * @return position
     */

    public Pose2d getLockedPosition() {
        return getPosition(m_team, m_grid, m_pieceType, m_side);
    }

    @Override
    public void periodic() {
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            m_team = VisionLocking.Team.BLUE;
        } else {
            m_team = VisionLocking.Team.RED;
        }
    }

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
}
