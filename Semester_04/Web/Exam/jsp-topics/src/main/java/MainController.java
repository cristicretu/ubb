import java.io.IOException;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.stream.Collectors;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;

@WebServlet("/main")
public class MainController extends HttpServlet {
  private static final long serialVersionUID = 1L;
  private static final String DB_URL = "jdbc:sqlite:tezt";

  private Connection getConnection() throws SQLException {
    try {
      Class.forName("org.sqlite.JDBC");
      return DriverManager.getConnection(DB_URL);
    } catch (ClassNotFoundException e) {
      throw new SQLException("SQLite Driver not found", e);
    }
  }

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    HttpSession session = request.getSession();
    String currentUser = (String) session.getAttribute("currentUser");
    if (currentUser == null) {
      response.sendRedirect("login.jsp");
      return;
    }

    try (Connection conn = getConnection()) {
      // ArrayList<Object> documentsAndMovies = getDocumentsAndMovies(conn,
      // currentUser);
      // request.setAttribute("documentsAndMovies", documentsAndMovies);

      // String documentsWithMostAuthors = getDocumentsWithMostAuthors(conn);
      // request.setAttribute("documentsWithMostAuthors", documentsWithMostAuthors);

    } catch (SQLException e) {
      request.setAttribute("error_message", "Database error: " + e.getMessage());
    } catch (Exception e) {
      request.setAttribute("error_message", "Unexpected error: " + e.getMessage());
    }

    request.getRequestDispatcher("main.jsp").forward(request, response);
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    HttpSession session = request.getSession();
    String currentUser = (String) session.getAttribute("currentUser");

    if (currentUser == null) {
      response.sendRedirect("login.jsp");
      return;
    }

    String action = request.getParameter("action");
    String successMessage = "";
    String errorMessage = "";

    try (Connection conn = getConnection()) {
      if ("add_post".equals(action)) {
        String postText = request.getParameter("post_text");
        String topicText = request.getParameter("topic_text");
        Integer topicExists = 0;

        if (postText.isEmpty() || topicText.isEmpty()) {
          errorMessage = "Post and topic cannot be empty";
          return;
        }

        String query = "SELECT * FROM Topics WHERE tipicname = ?";
        try (PreparedStatement stmt = conn.prepareStatement(query)) {
          stmt.setString(1, topicText);
          try (ResultSet rs = stmt.executeQuery()) {
            if (rs.next()) {
              topicExists = rs.getInt("id");
            }
          }
        }

        if (topicExists == 0) {
          String query1 = "INSERT INTO Topics (tipicname) VALUES (?)";
          try (PreparedStatement stmt1 = conn.prepareStatement(query1)) {
            stmt1.setString(1, topicText);
            stmt1.executeUpdate();
          }
        }

        int topicId = 0;
        String query2 = "select last_insert_rowid()";
        try (PreparedStatement stmt2 = conn.prepareStatement(query2)) {
          try (ResultSet rs = stmt2.executeQuery()) {
            if (rs.next()) {
              topicId = rs.getInt("last_insert_rowid()");
            }
          }
        }
        String query3 = "INSERT INTO Posts (user, topicid, text, date) VALUES (?, ?, ?, ?)";
        try (PreparedStatement stmt3 = conn.prepareStatement(query3)) {
          stmt3.setString(1, currentUser);
          stmt3.setInt(2, topicId);
          stmt3.setString(3, postText);
          int dateInt = (int) (System.currentTimeMillis() / 1000L);
          stmt3.setInt(4, dateInt);
          stmt3.executeUpdate();
          successMessage = "Post added successfully!";
        }
      }
    } catch (SQLException e) {
      errorMessage = "Database error: " + e.getMessage();
    }

    if (!successMessage.isEmpty()) {
      session.setAttribute("success_message", successMessage);
    }
    if (!errorMessage.isEmpty()) {
      session.setAttribute("error_message", errorMessage);
    }

    response.sendRedirect("main");
  }
}
