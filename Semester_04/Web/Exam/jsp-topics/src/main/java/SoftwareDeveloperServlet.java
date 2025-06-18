import java.io.IOException;
import java.io.PrintWriter;
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
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import model.SoftwareDeveloper;

@WebServlet("/softwareDeveloper")
public class SoftwareDeveloperServlet extends HttpServlet {
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

    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");
    PrintWriter out = response.getWriter();

    String action = request.getParameter("action");
    String name = request.getParameter("name");

    try (Connection conn = getConnection()) {
      if ("readAll".equals(action)) {
        JsonArray developers = readAll(conn);
        out.print(developers.toString());
      } else if ("findOne".equals(action) && name != null) {
        JsonObject developer = findOne(conn, name);
        if (developer != null) {
          out.print(developer.toString());
        } else {
          response.setStatus(HttpServletResponse.SC_NOT_FOUND);
          out.print("{\"error\":\"Developer not found\"}");
        }
      } else {
        JsonArray developers = readAll(conn);
        out.print(developers.toString());
      }
    } catch (SQLException e) {
      response.setStatus(HttpServletResponse.SC_INTERNAL_SERVER_ERROR);
      out.print("{\"error\":\"Database error: " + e.getMessage() + "\"}");
    }
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");
    PrintWriter out = response.getWriter();

    try (Connection conn = getConnection()) {
      String name = request.getParameter("name");
      String age = request.getParameter("age");
      String skills = request.getParameter("skills");

      boolean success = create(conn, name, age, skills);
      JsonObject result = new JsonObject();
      result.addProperty("success", success);
      out.print(result.toString());

    } catch (SQLException e) {
      response.setStatus(HttpServletResponse.SC_INTERNAL_SERVER_ERROR);
      out.print("{\"error\":\"Database error: " + e.getMessage() + "\"}");
    }
  }

  @Override
  protected void doPut(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");
    PrintWriter out = response.getWriter();

    try (Connection conn = getConnection()) {
      String id = request.getParameter("id");
      String name = request.getParameter("name");
      String age = request.getParameter("age");
      String skills = request.getParameter("skills");

      boolean success = update(conn, id, name, age, skills);
      JsonObject result = new JsonObject();
      result.addProperty("success", success);
      out.print(result.toString());

    } catch (SQLException e) {
      response.setStatus(HttpServletResponse.SC_INTERNAL_SERVER_ERROR);
      out.print("{\"error\":\"Database error: " + e.getMessage() + "\"}");
    }
  }

  @Override
  protected void doDelete(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");
    PrintWriter out = response.getWriter();

    try (Connection conn = getConnection()) {
      String id = request.getParameter("id");
      boolean success = delete(conn, id);
      JsonObject result = new JsonObject();
      result.addProperty("success", success);
      out.print(result.toString());

    } catch (SQLException e) {
      response.setStatus(HttpServletResponse.SC_INTERNAL_SERVER_ERROR);
      out.print("{\"error\":\"Database error: " + e.getMessage() + "\"}");
    }
  }

  private JsonArray readAll(Connection conn) throws SQLException {
    String query = "SELECT id, name, age, skills FROM SoftwareDeveloper";
    JsonArray developers = new JsonArray();

    try (PreparedStatement stmt = conn.prepareStatement(query);
        ResultSet rs = stmt.executeQuery()) {

      while (rs.next()) {
        JsonObject developer = new JsonObject();
        developer.addProperty("id", rs.getInt("id"));
        developer.addProperty("name", rs.getString("name"));
        developer.addProperty("age", rs.getInt("age"));
        developer.addProperty("skills", rs.getString("skills"));
        developers.add(developer);
      }
    }

    return developers;
  }

  private JsonObject findOne(Connection conn, String name) throws SQLException {
    String query = "SELECT id, name, age, skills FROM SoftwareDeveloper WHERE name = ?";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setString(1, name);

      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          JsonObject developer = new JsonObject();
          developer.addProperty("id", rs.getInt("id"));
          developer.addProperty("name", rs.getString("name"));
          developer.addProperty("age", rs.getInt("age"));
          developer.addProperty("skills", rs.getString("skills"));
          return developer;
        }
      }
    }

    return null;
  }

  private boolean create(Connection conn, String name, String age, String skills) throws SQLException {
    String query = "INSERT INTO SoftwareDeveloper SET name = ?, age = ?, skills = ?";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      // Clean and sanitize input (similar to htmlspecialchars(strip_tags()) in PHP)
      String cleanName = name != null ? name.trim() : "";
      String cleanAge = age != null ? age.trim() : "0";
      String cleanSkills = skills != null ? skills.trim() : "";

      stmt.setString(1, cleanName);
      stmt.setInt(2, Integer.parseInt(cleanAge));
      stmt.setString(3, cleanSkills);

      return stmt.executeUpdate() > 0;
    }
  }

  private boolean delete(Connection conn, String id) throws SQLException {
    String query = "DELETE FROM SoftwareDeveloper WHERE id = ?";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setInt(1, Integer.parseInt(id));
      return stmt.executeUpdate() > 0;
    }
  }

  private boolean update(Connection conn, String id, String name, String age, String skills) throws SQLException {
    String query = "UPDATE SoftwareDeveloper SET name = ?, age = ?, skills = ? WHERE id = ?";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setString(1, name);
      stmt.setInt(2, Integer.parseInt(age));
      stmt.setString(3, skills);
      stmt.setInt(4, Integer.parseInt(id));

      return stmt.executeUpdate() > 0;
    }
  }
}