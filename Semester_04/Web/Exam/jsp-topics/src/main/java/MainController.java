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

  private ArrayList<Object> getPosts(Connection conn, String currentUser) throws SQLException {
    String query = "SELECT * FROM Posts WHERE user = ?";
    ArrayList<Object> posts = new ArrayList<>();
    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setString(1, currentUser);
      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          int postId = rs.getInt("id");
          String user = rs.getString("user");
          posts.add(new Object[] { postId, user });
        }
      }
    }

    return posts;
  }

  // The original logic is flawed: it only checks for new posts for the *current
  // user*,
  // so if you add a post as a different user, it won't show up for this user.
  // If you want to detect *any* new post (from any user), you need to fetch all
  // posts, not just the current user's.
  // Here is a rewrite that checks for new posts globally (across all users):

  // Helper to get all posts (not just for current user)
  private ArrayList<Object> getAllPosts(Connection conn) throws SQLException {
    String query = "SELECT * FROM Posts";
    ArrayList<Object> posts = new ArrayList<>();
    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          int postId = rs.getInt("id");
          String user = rs.getString("user");
          posts.add(new Object[] { postId, user });
        }
      }
    }
    return posts;
  }

  private boolean isNewPost(Connection conn, String currentUser, HttpSession session) throws SQLException {
    var oldPosts = (ArrayList<Object>) session.getAttribute("allPosts");
    if (oldPosts == null)
      oldPosts = new ArrayList<>();

    var newPosts = getAllPosts(conn);
    session.setAttribute("allPosts", newPosts);

    // Get list of old post IDs for comparison
    List<Integer> oldIds = oldPosts.stream()
        .map(o -> ((Object[]) o)[0])
        .map(id -> (Integer) id)
        .collect(Collectors.toList());

    // Check for new posts that are NOT from the current user
    for (Object obj : newPosts) {
      Object[] arr = (Object[]) obj;
      Integer id = (Integer) arr[0];
      String postUser = (String) arr[1];

      // Only consider it a "new post" if:
      // 1. It's a new post (not in oldIds)
      // 2. It's NOT from the current user
      if (!oldIds.contains(id) && !currentUser.equals(postUser)) {
        return true;
      }
    }
    return false;
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
      if (isNewPost(conn, currentUser, session)) {
        session.setAttribute("success_message", "New posts available!");
      }

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
      } else if ("update_post".equals(action)) {
        String postId = request.getParameter("post_id");
        String postText = request.getParameter("post_text");
        String topicText = request.getParameter("topic_text");
        Integer topicExists = 0;
        Integer postExists = 0;

        if (postId.isEmpty() || postText.isEmpty() || topicText.isEmpty()) {
          errorMessage = "Post ID, post text, and topic text cannot be empty";
        }

        // Validate that postId is a valid integer
        if (errorMessage.isEmpty()) {
          try {
            Integer.parseInt(postId);
          } catch (NumberFormatException e) {
            errorMessage = "Post ID must be a valid number";
          }
        }

        String query = "SELECT * FROM Topics WHERE tipicname = ?";
        try (PreparedStatement stmt = conn.prepareStatement(query)) {
          stmt.setString(1, topicText);
          try (ResultSet rs = stmt.executeQuery()) {
            if (rs.next()) {
              topicExists = rs.getInt("id");
            } else {
              errorMessage = "Topic does not exist";
            }
          }
        }

        if (topicExists == 0) {
          errorMessage = "Topic does not exist";
        }

        String query2 = "SELECT * FROM Posts WHERE id = ?";
        try (PreparedStatement stmt2 = conn.prepareStatement(query2)) {
          stmt2.setInt(1, Integer.parseInt(postId));
          try (ResultSet rs = stmt2.executeQuery()) {
            if (rs.next()) {
              postExists = rs.getInt("id");
            }
          }
        }

        if (postExists == 0) {
          errorMessage = "Post does not exist";
        }

        if (errorMessage.isEmpty()) {
          String query3 = "UPDATE Posts SET text = ?, topicid = ? WHERE id = ?";
          try (PreparedStatement stmt3 = conn.prepareStatement(query3)) {
            stmt3.setString(1, postText);
            stmt3.setInt(2, topicExists);
            stmt3.setInt(3, postExists);
            stmt3.executeUpdate();
            successMessage = "Post updated successfully!";
          }
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
