package util;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.sql.Statement;

public class DatabaseUtil {
  private static final String DB_URL = "jdbc:sqlite:snake.db";

  static {
    try {
      Class.forName("org.sqlite.JDBC");
      initializeDatabase();
    } catch (ClassNotFoundException e) {
      throw new RuntimeException("SQLite JDBC driver not found", e);
    }
  }

  public static Connection getConnection() throws SQLException {
    return DriverManager.getConnection(DB_URL);
  }

  private static void initializeDatabase() {
    try (Connection conn = getConnection();
        Statement stmt = conn.createStatement()) {

      String createUsersTable = "CREATE TABLE IF NOT EXISTS users (" +
          "id INTEGER PRIMARY KEY AUTOINCREMENT, " +
          "username VARCHAR(50) UNIQUE NOT NULL, " +
          "password VARCHAR(255) NOT NULL, " +
          "high_score INTEGER DEFAULT 0, " +
          "created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP, " +
          "last_login TIMESTAMP" +
          ")";

      String createGameStatesTable = "CREATE TABLE IF NOT EXISTS game_states (" +
          "id INTEGER PRIMARY KEY AUTOINCREMENT, " +
          "user_id INTEGER NOT NULL, " +
          "score INTEGER DEFAULT 0, " +
          "obstacles TEXT, " +
          "apple TEXT, " +
          "snake TEXT NOT NULL, " +
          "current_direction VARCHAR(10) DEFAULT 'RIGHT', " +
          "created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP, " +
          "FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE" +
          ")";

      String createMovesTable = "CREATE TABLE IF NOT EXISTS moves (" +
          "id INTEGER PRIMARY KEY AUTOINCREMENT, " +
          "game_state_id INTEGER NOT NULL, " +
          "user_id INTEGER NOT NULL, " +
          "direction VARCHAR(10) NOT NULL, " +
          "from_x INTEGER NOT NULL, " +
          "from_y INTEGER NOT NULL, " +
          "to_x INTEGER NOT NULL, " +
          "to_y INTEGER NOT NULL, " +
          "ate_apple BOOLEAN DEFAULT FALSE, " +
          "score_after_move INTEGER DEFAULT 0, " +
          "move_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, " +
          "FOREIGN KEY (game_state_id) REFERENCES game_states(id) ON DELETE CASCADE, " +
          "FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE" +
          ")";

      stmt.execute(createUsersTable);
      stmt.execute(createGameStatesTable);
      stmt.execute(createMovesTable);

      System.out.println("Database initialized successfully!");

    } catch (SQLException e) {
      throw new RuntimeException("Failed to initialize database", e);
    }
  }

  public static void closeConnection(Connection conn) {
    if (conn != null) {
      try {
        conn.close();
      } catch (SQLException e) {
        System.err.println("Error closing database connection: " + e.getMessage());
      }
    }
  }
}