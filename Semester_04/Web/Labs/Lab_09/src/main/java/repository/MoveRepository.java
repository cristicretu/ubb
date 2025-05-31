package repository;

import model.Move;
import util.DatabaseUtil;

import java.sql.*;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

public class MoveRepository {

  public Move save(Move move) throws SQLException {
    String sql = "INSERT INTO moves (game_state_id, user_id, direction, from_x, from_y, to_x, to_y, ate_apple, score_after_move, move_time) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, move.getGameStateId());
      stmt.setLong(2, move.getUserId());
      stmt.setString(3, move.getDirection());
      stmt.setInt(4, move.getFromX());
      stmt.setInt(5, move.getFromY());
      stmt.setInt(6, move.getToX());
      stmt.setInt(7, move.getToY());
      stmt.setBoolean(8, move.isAteApple());
      stmt.setInt(9, move.getScoreAfterMove());
      stmt.setTimestamp(10, Timestamp.valueOf(move.getMoveTime()));

      int affectedRows = stmt.executeUpdate();

      if (affectedRows == 0) {
        throw new SQLException("Creating move failed, no rows affected.");
      }

      try (Statement idStmt = conn.createStatement();
          ResultSet rs = idStmt.executeQuery("SELECT last_insert_rowid()")) {
        if (rs.next()) {
          move.setId(rs.getLong(1));
        } else {
          throw new SQLException("Creating move failed, no ID obtained.");
        }
      }

      return move;
    }
  }

  public List<Move> findByGameStateId(Long gameStateId) throws SQLException {
    String sql = "SELECT * FROM moves WHERE game_state_id = ? ORDER BY move_time ASC";
    List<Move> moves = new ArrayList<>();

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, gameStateId);

      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          moves.add(mapResultSetToMove(rs));
        }
      }
    }

    return moves;
  }

  public List<Move> findByUserId(Long userId) throws SQLException {
    String sql = "SELECT * FROM moves WHERE user_id = ? ORDER BY move_time DESC";
    List<Move> moves = new ArrayList<>();

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, userId);

      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          moves.add(mapResultSetToMove(rs));
        }
      }
    }

    return moves;
  }

  public boolean deleteByUserId(Long userId) throws SQLException {
    String sql = "DELETE FROM moves WHERE user_id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, userId);

      int affectedRows = stmt.executeUpdate();
      return affectedRows > 0;
    }
  }

  private Move mapResultSetToMove(ResultSet rs) throws SQLException {
    Move move = new Move();
    move.setId(rs.getLong("id"));
    move.setGameStateId(rs.getLong("game_state_id"));
    move.setUserId(rs.getLong("user_id"));
    move.setDirection(rs.getString("direction"));
    move.setFromX(rs.getInt("from_x"));
    move.setFromY(rs.getInt("from_y"));
    move.setToX(rs.getInt("to_x"));
    move.setToY(rs.getInt("to_y"));
    move.setAteApple(rs.getBoolean("ate_apple"));
    move.setScoreAfterMove(rs.getInt("score_after_move"));

    Timestamp moveTime = rs.getTimestamp("move_time");
    if (moveTime != null) {
      move.setMoveTime(moveTime.toLocalDateTime());
    }

    return move;
  }
}