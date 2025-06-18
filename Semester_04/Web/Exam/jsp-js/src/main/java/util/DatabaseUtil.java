package util;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.sql.Statement;

public class DatabaseUtil {
  private static final String DB_URL = "jdbc:sqlite:tezt";

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