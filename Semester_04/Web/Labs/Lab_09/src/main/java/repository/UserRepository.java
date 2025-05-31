package repository;

import model.User;
import util.DatabaseUtil;

import java.sql.*;
import java.time.LocalDateTime;
import java.util.Optional;

public class UserRepository {

  public User save(User user) throws SQLException {
    String sql = "INSERT INTO users (username, password, high_score, created_at) VALUES (?, ?, ?, ?)";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setString(1, user.getUsername());
      stmt.setString(2, user.getHashedPassword());
      stmt.setInt(3, user.getHighScore());
      stmt.setTimestamp(4, Timestamp.valueOf(user.getCreatedAt()));

      int affectedRows = stmt.executeUpdate();

      if (affectedRows == 0) {
        throw new SQLException("Creating user failed, no rows affected.");
      }

      try (Statement idStmt = conn.createStatement();
          ResultSet rs = idStmt.executeQuery("SELECT last_insert_rowid()")) {
        if (rs.next()) {
          user.setId(rs.getLong(1));
        } else {
          throw new SQLException("Creating user failed, no ID obtained.");
        }
      }

      return user;
    }
  }

  public Optional<User> findById(Long id) throws SQLException {
    String sql = "SELECT * FROM users WHERE id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, id);

      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          return Optional.of(mapResultSetToUser(rs));
        }
      }
    }

    return Optional.empty();
  }

  public Optional<User> findByUsername(String username) throws SQLException {
    String sql = "SELECT * FROM users WHERE username = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setString(1, username);

      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          return Optional.of(mapResultSetToUser(rs));
        }
      }
    }

    return Optional.empty();
  }

  public User update(User user) throws SQLException {
    String sql = "UPDATE users SET username = ?, password = ?, high_score = ?, last_login = ? WHERE id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setString(1, user.getUsername());
      stmt.setString(2, user.getHashedPassword());
      stmt.setInt(3, user.getHighScore());
      stmt.setTimestamp(4, user.getLastLogin() != null ? Timestamp.valueOf(user.getLastLogin()) : null);
      stmt.setLong(5, user.getId());

      int affectedRows = stmt.executeUpdate();

      if (affectedRows == 0) {
        throw new SQLException("Updating user failed, no rows affected.");
      }

      return user;
    }
  }

  public boolean deleteById(Long id) throws SQLException {
    String sql = "DELETE FROM users WHERE id = ?";

    try (Connection conn = DatabaseUtil.getConnection();
        PreparedStatement stmt = conn.prepareStatement(sql)) {

      stmt.setLong(1, id);

      int affectedRows = stmt.executeUpdate();
      return affectedRows > 0;
    }
  }

  public boolean existsByUsername(String username) throws SQLException {
    return findByUsername(username).isPresent();
  }

  private User mapResultSetToUser(ResultSet rs) throws SQLException {
    User user = new User();
    user.setId(rs.getLong("id"));
    user.setUsername(rs.getString("username"));
    user.setHashedPassword(rs.getString("password"));
    user.setHighScore(rs.getInt("high_score"));

    Timestamp createdAt = rs.getTimestamp("created_at");
    if (createdAt != null) {
      user.setCreatedAt(createdAt.toLocalDateTime());
    }

    Timestamp lastLogin = rs.getTimestamp("last_login");
    if (lastLogin != null) {
      user.setLastLogin(lastLogin.toLocalDateTime());
    }

    return user;
  }
}