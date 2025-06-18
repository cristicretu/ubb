import java.io.IOException;
import java.io.PrintWriter;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import model.Project;

@WebServlet("/project")
public class ProjectServlet extends HttpServlet {
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
    String projectManagerId = request.getParameter("projectManagerId");

    try (Connection conn = getConnection()) {
      if ("readAll".equals(action)) {
        JsonArray projects = readAll(conn);
        out.print(projects.toString());
      } else if ("readAllByProjectManagerID".equals(action)) {
        JsonArray projects = readAllByProjectManagerID(conn, projectManagerId);
        out.print(projects.toString());
      } else {
        JsonArray projects = readAll(conn);
        out.print(projects.toString());
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

    String action = request.getParameter("action");

    try (Connection conn = getConnection()) {
      if ("create".equals(action)) {
        String projectName = request.getParameter("name");
        String projectManagerId = request.getParameter("projectManagerId");
        String description = request.getParameter("description");
        String members = request.getParameter("members");

        boolean success = create(conn, projectName, projectManagerId, description, members);
        JsonObject result = new JsonObject();
        result.addProperty("success", success);
        out.print(result.toString());

      } else if ("assignProject".equals(action)) {
        String projectName = request.getParameter("projectName");
        String projectManagerId = request.getParameter("projectManagerId");

        JsonObject project = assignProject(conn, projectName, projectManagerId);
        if (project != null) {
          out.print(project.toString());
        } else {
          response.setStatus(HttpServletResponse.SC_INTERNAL_SERVER_ERROR);
          out.print("{\"error\":\"Failed to assign project\"}");
        }
      }
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
      String projectManagerId = request.getParameter("projectManagerId");
      String name = request.getParameter("name");
      String description = request.getParameter("description");
      String members = request.getParameter("members");

      boolean success = update(conn, id, projectManagerId, name, description, members);
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
    String query = "SELECT id, ProjectManagerID, name, description, members FROM Project";
    JsonArray projects = new JsonArray();

    try (PreparedStatement stmt = conn.prepareStatement(query);
        ResultSet rs = stmt.executeQuery()) {

      while (rs.next()) {
        JsonObject project = new JsonObject();
        project.addProperty("id", rs.getInt("id"));
        project.addProperty("projectManagerId", rs.getInt("ProjectManagerID"));
        project.addProperty("name", rs.getString("name"));
        project.addProperty("description", rs.getString("description"));
        project.addProperty("members", rs.getString("members"));
        projects.add(project);
      }
    }

    return projects;
  }

  private JsonArray readAllByProjectManagerID(Connection conn, String projectManagerId) throws SQLException {
    if (projectManagerId == null) {
      return readAll(conn);
    }

    String query = "SELECT id, ProjectManagerID, name, description, members FROM Project WHERE ProjectManagerID = ?";
    JsonArray projects = new JsonArray();

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setInt(1, Integer.parseInt(projectManagerId));

      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          JsonObject project = new JsonObject();
          project.addProperty("id", rs.getInt("id"));
          project.addProperty("projectManagerId", rs.getInt("ProjectManagerID"));
          project.addProperty("name", rs.getString("name"));
          project.addProperty("description", rs.getString("description"));
          project.addProperty("members", rs.getString("members"));
          projects.add(project);
        }
      }
    }

    return projects;
  }

  private JsonObject assignProject(Connection conn, String projectName, String projectManagerId) throws SQLException {
    // Check if project already exists
    String selectQuery = "SELECT id FROM Project WHERE name = ?";
    try (PreparedStatement stmt = conn.prepareStatement(selectQuery)) {
      stmt.setString(1, projectName);

      try (ResultSet rs = stmt.executeQuery()) {
        int projectId;

        if (!rs.next()) {
          // Create new project
          if (create(conn, projectName, projectManagerId, "", "")) {
            String lastIdQuery = "SELECT LAST_INSERT_ID()";
            try (Statement lastIdStmt = conn.createStatement();
                ResultSet lastIdRs = lastIdStmt.executeQuery(lastIdQuery)) {
              if (lastIdRs.next()) {
                projectId = lastIdRs.getInt(1);
              } else {
                return null;
              }
            }
          } else {
            return null;
          }
        } else {
          // Update existing project
          projectId = rs.getInt("id");
          String updateQuery = "UPDATE Project SET ProjectManagerID = ? WHERE id = ?";
          try (PreparedStatement updateStmt = conn.prepareStatement(updateQuery)) {
            updateStmt.setInt(1, Integer.parseInt(projectManagerId));
            updateStmt.setInt(2, projectId);
            if (!updateStmt.execute()) {
              return null;
            }
          }
        }

        // Return the project details
        String detailsQuery = "SELECT id, ProjectManagerID, name, description, members FROM Project WHERE id = ?";
        try (PreparedStatement detailsStmt = conn.prepareStatement(detailsQuery)) {
          detailsStmt.setInt(1, projectId);

          try (ResultSet detailsRs = detailsStmt.executeQuery()) {
            if (detailsRs.next()) {
              JsonObject project = new JsonObject();
              project.addProperty("id", detailsRs.getInt("id"));
              project.addProperty("projectManagerId", detailsRs.getInt("ProjectManagerID"));
              project.addProperty("name", detailsRs.getString("name"));
              project.addProperty("description", detailsRs.getString("description"));
              project.addProperty("members", detailsRs.getString("members"));
              return project;
            }
          }
        }
      }
    }

    return null;
  }

  private boolean create(Connection conn, String projectName, String projectManagerId, String description,
      String members) throws SQLException {
    String query = "INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (?, ?, ?, ?)";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setInt(1, Integer.parseInt(projectManagerId));
      stmt.setString(2, projectName != null ? projectName : "");
      stmt.setString(3, description != null ? description : "");
      stmt.setString(4, members != null ? members : "");

      return stmt.executeUpdate() > 0;
    }
  }

  private boolean delete(Connection conn, String id) throws SQLException {
    String query = "DELETE FROM Project WHERE id = ?";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setInt(1, Integer.parseInt(id));
      return stmt.executeUpdate() > 0;
    }
  }

  private boolean update(Connection conn, String id, String projectManagerId, String name, String description,
      String members) throws SQLException {
    String query = "UPDATE Project SET ProjectManagerID = ?, name = ?, description = ?, members = ? WHERE id = ?";

    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setInt(1, Integer.parseInt(projectManagerId));
      stmt.setString(2, name);
      stmt.setString(3, description);
      stmt.setString(4, members);
      stmt.setInt(5, Integer.parseInt(id));

      return stmt.executeUpdate() > 0;
    }
  }
}