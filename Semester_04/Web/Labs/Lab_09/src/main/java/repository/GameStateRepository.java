package repository;

import model.GameState;
import model.Position;
import util.DatabaseUtil;

import java.sql.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class GameStateRepository {

  public GameState save(GameState gameState) throws SQLException {
    String sql = "INSERT INTO game_states (user_id, score, obstacles, apple, snake, current_direction) VALUES (?, ?, ?, ?, ?, ?)";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, gameState.getUserId());
      stmt.setInt(2, gameState.getScore());
      stmt.setString(3, serializePositions(gameState.getObstacles()));
      stmt.setString(4, serializePosition(gameState.getApple()));
      stmt.setString(5, serializePositions(gameState.getSnake()));
      stmt.setString(6, gameState.getCurrentDirection());

      int affectedRows = stmt.executeUpdate();

      if (affectedRows == 0) {
        throw new SQLException("Creating game state failed, no rows affected.");
      }

      try (Statement idStmt = conn.createStatement();
          ResultSet rs = idStmt.executeQuery("SELECT last_insert_rowid()")) {
        if (rs.next()) {
          gameState.setId(rs.getLong(1));
        } else {
          throw new SQLException("Creating game state failed, no ID obtained.");
        }
      }

      return gameState;
    }
  }

  public Optional<GameState> findById(Long id) throws SQLException {
    String sql = "SELECT * FROM game_states WHERE id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, id);

      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          return Optional.of(mapResultSetToGameState(rs));
        }
      }
    }

    return Optional.empty();
  }

  public List<GameState> findByUserId(Long userId) throws SQLException {
    String sql = "SELECT * FROM game_states WHERE user_id = ? ORDER BY id DESC";
    List<GameState> gameStates = new ArrayList<>();

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, userId);

      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          gameStates.add(mapResultSetToGameState(rs));
        }
      }
    }

    return gameStates;
  }

  public Optional<GameState> findLatestByUserId(Long userId) throws SQLException {
    String sql = "SELECT * FROM game_states WHERE user_id = ? ORDER BY id DESC LIMIT 1";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, userId);

      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          return Optional.of(mapResultSetToGameState(rs));
        }
      }
    }

    return Optional.empty();
  }

  public GameState update(GameState gameState) throws SQLException {
    String sql = "UPDATE game_states SET user_id = ?, score = ?, obstacles = ?, apple = ?, snake = ?, current_direction = ? WHERE id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, gameState.getUserId());
      stmt.setInt(2, gameState.getScore());
      stmt.setString(3, serializePositions(gameState.getObstacles()));
      stmt.setString(4, serializePosition(gameState.getApple()));
      stmt.setString(5, serializePositions(gameState.getSnake()));
      stmt.setString(6, gameState.getCurrentDirection());
      stmt.setLong(7, gameState.getId());

      int affectedRows = stmt.executeUpdate();

      if (affectedRows == 0) {
        throw new SQLException("Updating game state failed, no rows affected.");
      }

      return gameState;
    }
  }

  public boolean deleteById(Long id) throws SQLException {
    String sql = "DELETE FROM game_states WHERE id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, id);

      int affectedRows = stmt.executeUpdate();
      return affectedRows > 0;
    }
  }

  public boolean deleteByUserId(Long userId) throws SQLException {
    String sql = "DELETE FROM game_states WHERE user_id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, userId);

      int affectedRows = stmt.executeUpdate();
      return affectedRows > 0;
    }
  }

  private GameState mapResultSetToGameState(ResultSet rs) throws SQLException {
    GameState gameState = new GameState();
    gameState.setId(rs.getLong("id"));
    gameState.setUserId(rs.getLong("user_id"));
    gameState.setScore(rs.getInt("score"));
    gameState.setObstacles(deserializePositions(rs.getString("obstacles")));
    gameState.setApple(deserializePosition(rs.getString("apple")));
    gameState.setSnake(deserializePositions(rs.getString("snake")));
    gameState.setCurrentDirection(rs.getString("current_direction"));
    return gameState;
  }

  private String serializePosition(Position position) {
    if (position == null)
      return null;
    return position.getX() + "," + position.getY();
  }

  private String serializePositions(Position[] positions) {
    if (positions == null || positions.length == 0)
      return "";

    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < positions.length; i++) {
      if (i > 0)
        sb.append(";");
      sb.append(serializePosition(positions[i]));
    }
    return sb.toString();
  }

  private Position deserializePosition(String positionStr) {
    if (positionStr == null || positionStr.trim().isEmpty())
      return null;

    String[] parts = positionStr.split(",");
    if (parts.length != 2)
      return null;

    try {
      int x = Integer.parseInt(parts[0].trim());
      int y = Integer.parseInt(parts[1].trim());
      return new Position(x, y);
    } catch (NumberFormatException e) {
      return null;
    }
  }

  private Position[] deserializePositions(String positionsStr) {
    if (positionsStr == null || positionsStr.trim().isEmpty())
      return new Position[0];

    String[] positionStrings = positionsStr.split(";");
    List<Position> positions = new ArrayList<>();

    for (String posStr : positionStrings) {
      Position pos = deserializePosition(posStr);
      if (pos != null) {
        positions.add(pos);
      }
    }

    return positions.toArray(new Position[0]);
  }
}
