import java.io.IOException;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;

@WebServlet("/login")
public class LoginController extends HttpServlet {
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

    request.getRequestDispatcher("login.jsp").forward(request, response);
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String username = request.getParameter("name");
    String searchParam = request.getParameter("searchParam");
    String errorMessage = "";
    HttpSession session = request.getSession();
    Boolean verifiedSearchParam = false;

    int userId = -1;

    if (username == null || username.trim().isEmpty() || searchParam == null || searchParam.trim().isEmpty()) {
      errorMessage = "Please enter a valid username or document or movie name or id";
      request.setAttribute("error_message", errorMessage);
      request.getRequestDispatcher("login.jsp").forward(request, response);
      return;
    } else {
      try (Connection conn = getConnection()) {
        // find if the user exists
        String query = "SELECT id FROM Authors WHERE name = ?";
        try (PreparedStatement stmt = conn.prepareStatement(query)) {
          stmt.setString(1, username.trim());
          try (ResultSet rs = stmt.executeQuery()) {
            if (rs.next()) {
              userId = rs.getInt("id");
            }
          }
        }

        // Check if user was found
        if (userId == -1) {
          errorMessage = "User '" + username.trim() + "' not found. Valid users: alice, bob, carol";
          request.setAttribute("error_message", errorMessage);
          request.getRequestDispatcher("login.jsp").forward(request, response);
          return;
        }

        // Check if searchParam exists in Documents or Movies tables by name or id
        String searchedDocumentId = null;
        String searchedMovieId = null;

        String query1 = "SELECT id FROM Documents WHERE name = ? OR id = ?";
        try (PreparedStatement stmt = conn.prepareStatement(query1)) {
          stmt.setString(1, searchParam.trim());
          try {
            stmt.setInt(2, Integer.parseInt(searchParam.trim()));
          } catch (NumberFormatException e) {
            stmt.setInt(2, -1); // Invalid ID if not a number
          }
          try (ResultSet rs = stmt.executeQuery()) {
            if (rs.next()) {
              searchedDocumentId = String.valueOf(rs.getInt("id"));
              verifiedSearchParam = true;
            }
          }
        }

        if (!verifiedSearchParam) {
          String query2 = "SELECT id FROM Movies WHERE title = ? OR id = ?";
          try (PreparedStatement stmt = conn.prepareStatement(query2)) {
            stmt.setString(1, searchParam.trim());
            try {
              stmt.setInt(2, Integer.parseInt(searchParam.trim()));
            } catch (NumberFormatException e) {
              stmt.setInt(2, -1); // Invalid ID if not a number
            }
            try (ResultSet rs = stmt.executeQuery()) {
              if (rs.next()) {
                searchedMovieId = String.valueOf(rs.getInt("id"));
                verifiedSearchParam = true;
              }
            }
          }
        }

        if (!verifiedSearchParam) {
          errorMessage = "Could not verify document or movie. Valid options: doc1, doc2, doc3, movie1, movie2, movie3, or their IDs (1, 2, 3)";
          request.setAttribute("error_message", errorMessage);
          request.getRequestDispatcher("login.jsp").forward(request, response);
          return;
        }

        // Check if the user actually authored the found document or movie
        if (!isUserAuthorOfSearchParam(conn, userId, searchedDocumentId, searchedMovieId)) {
          System.out.println("DEBUG: User is not author of the document/movie");
          errorMessage = "You are not the author of the specified document or movie. Please enter a document or movie you have authored.";
          request.setAttribute("error_message", errorMessage);
          request.getRequestDispatcher("login.jsp").forward(request, response);
          return;
        }

        System.out.println("DEBUG: All validations passed - setting session and redirecting");
        // Set session attributes - using "currentUser" to match MainController
        // expectations
        session.setAttribute("userId", Integer.toString(userId));
        session.setAttribute("currentUser", username.trim()); // Changed from "username" to "currentUser"
        session.setAttribute("searchParam", searchParam.trim());

        // Redirect to main page
        System.out.println("DEBUG: Redirecting to main page");
        response.sendRedirect("main");
      } catch (Exception e) {
        System.out.println("DEBUG: Exception occurred: " + e.getMessage());
        e.printStackTrace();
        errorMessage = "Database error: " + e.getMessage();
        request.setAttribute("error_message", errorMessage);
        request.getRequestDispatcher("login.jsp").forward(request, response);
      }
    }
  }

  private boolean isUserAuthorOfSearchParam(Connection conn, int userId, String searchedDocumentId,
      String searchedMovieId) throws SQLException {

    if (searchedDocumentId != null) {
      String query = "SELECT * FROM Authors WHERE id = ? AND documentList LIKE ?";
      try (PreparedStatement stmt = conn.prepareStatement(query)) {
        stmt.setInt(1, userId);
        stmt.setString(2, "%" + searchedDocumentId + "%");
        try (ResultSet rs = stmt.executeQuery()) {
          if (rs.next()) {
            return true;
          } else {
          }
        }
      }
    }

    if (searchedMovieId != null) {
      String query2 = "SELECT * FROM Authors WHERE id = ? AND movieList LIKE ?";
      try (PreparedStatement stmt = conn.prepareStatement(query2)) {
        stmt.setInt(1, userId);
        stmt.setString(2, "%" + searchedMovieId + "%");
        try (ResultSet rs = stmt.executeQuery()) {
          if (rs.next()) {
            return true;
          } else {
          }
        }
      }
    }

    return false;
  }
}

@WebServlet("/logout")
class LogoutController extends HttpServlet {
  private static final long serialVersionUID = 1L;

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    HttpSession session = request.getSession();
    session.invalidate();

    response.sendRedirect("login.jsp");
  }
}