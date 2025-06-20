import java.io.IOException;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;
import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;
import model.Project;
import model.SoftwareDeveloper;
import dto.ProjectData;
import dto.DeveloperData;

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
      // Get current user ID
      Integer currentUserID = getCurrentUserID(conn, currentUser);

      // Get all projects
      List<ProjectData> allProjects = getAllProjects(conn);

      // Get user's managed projects
      List<ProjectData> yourProjects = new ArrayList<>();
      if (currentUserID != null) {
        yourProjects = getProjectsByManagerID(conn, currentUserID);
      }

      // Get projects where user is a member
      List<ProjectData> memberProjects = getMemberProjects(allProjects, currentUser);

      // Get all developers
      List<DeveloperData> allDevelopers = getAllDevelopers(conn);

      // Set attributes for JSP
      request.setAttribute("currentUser", currentUser);
      request.setAttribute("allProjects", allProjects);
      request.setAttribute("yourProjects", yourProjects);
      request.setAttribute("memberProjects", memberProjects);
      request.setAttribute("allDevelopers", allDevelopers);

      request.getRequestDispatcher("main.jsp").forward(request, response);

    } catch (SQLException e) {
      request.setAttribute("error_message", "Database error: " + e.getMessage());
      request.getRequestDispatcher("main.jsp").forward(request, response);
    }
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
      if ("assign_project".equals(action)) {
        String projectName = request.getParameter("project_name");
        String projectManagerName = request.getParameter("project_manager_name");

        if (projectName != null && !projectName.trim().isEmpty() &&
            projectManagerName != null && !projectManagerName.trim().isEmpty()) {

          projectName = projectName.trim();
          projectManagerName = projectManagerName.trim();

          Integer managerId = findDeveloperByName(conn, projectManagerName);
          if (managerId != null) {
            boolean success = assignProject(conn, projectName, managerId);
            if (success) {
              successMessage = "Project '" + projectName + "' assigned to '" + projectManagerName + "' successfully!";
            } else {
              errorMessage = "Failed to assign project.";
            }
          } else {
            errorMessage = "Developer '" + projectManagerName + "' not found!";
          }
        } else {
          errorMessage = "Please fill in all fields.";
        }
      }
    } catch (SQLException e) {
      errorMessage = "Database error: " + e.getMessage();
    }

    // Set messages in session to persist across redirect
    if (!successMessage.isEmpty()) {
      session.setAttribute("success_message", successMessage);
    }
    if (!errorMessage.isEmpty()) {
      session.setAttribute("error_message", errorMessage);
    }

    response.sendRedirect("main");
  }

  private Integer getCurrentUserID(Connection conn, String currentUser) throws SQLException {
    String query = "SELECT id FROM SoftwareDeveloper WHERE name = ?";
    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setString(1, currentUser);
      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          return rs.getInt("id");
        }
      }
    }
    return null;
  }

  private List<ProjectData> getAllProjects(Connection conn) throws SQLException {
    List<ProjectData> projects = new ArrayList<>();
    String query = "SELECT id, ProjectManagerID, name, description, members FROM Project";
    try (PreparedStatement stmt = conn.prepareStatement(query);
        ResultSet rs = stmt.executeQuery()) {

      while (rs.next()) {
        ProjectData project = new ProjectData();
        project.id = rs.getInt("id");
        project.projectManagerID = rs.getInt("ProjectManagerID");
        project.name = rs.getString("name");
        project.description = rs.getString("description");
        project.members = rs.getString("members");
        projects.add(project);
      }
    }
    return projects;
  }

  private List<ProjectData> getProjectsByManagerID(Connection conn, Integer managerID) throws SQLException {
    List<ProjectData> projects = new ArrayList<>();
    String query = "SELECT id, ProjectManagerID, name, description, members FROM Project WHERE ProjectManagerID = ?";
    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setInt(1, managerID);
      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          ProjectData project = new ProjectData();
          project.id = rs.getInt("id");
          project.projectManagerID = rs.getInt("ProjectManagerID");
          project.name = rs.getString("name");
          project.description = rs.getString("description");
          project.members = rs.getString("members");
          projects.add(project);
        }
      }
    }
    return projects;
  }

  private List<ProjectData> getMemberProjects(List<ProjectData> allProjects, String currentUser) {
    List<ProjectData> memberProjects = new ArrayList<>();
    for (ProjectData project : allProjects) {
      if (project.members != null && project.members.contains(currentUser)) {
        memberProjects.add(project);
      }
    }
    return memberProjects;
  }

  private List<DeveloperData> getAllDevelopers(Connection conn) throws SQLException {
    List<DeveloperData> developers = new ArrayList<>();
    String query = "SELECT id, name, age, skills FROM SoftwareDeveloper";
    try (PreparedStatement stmt = conn.prepareStatement(query);
        ResultSet rs = stmt.executeQuery()) {

      while (rs.next()) {
        DeveloperData developer = new DeveloperData();
        developer.id = rs.getInt("id");
        developer.name = rs.getString("name");
        developer.age = rs.getInt("age");
        developer.skills = rs.getString("skills");
        developers.add(developer);
      }
    }
    return developers;
  }

  private Integer findDeveloperByName(Connection conn, String name) throws SQLException {
    String query = "SELECT id FROM SoftwareDeveloper WHERE name = ?";
    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setString(1, name);
      try (ResultSet rs = stmt.executeQuery()) {
        if (rs.next()) {
          return rs.getInt("id");
        }
      }
    }
    return null;
  }

  private boolean assignProject(Connection conn, String projectName, Integer projectManagerID) throws SQLException {
    // Check if project already exists
    String selectQuery = "SELECT id FROM Project WHERE name = ?";
    try (PreparedStatement stmt = conn.prepareStatement(selectQuery)) {
      stmt.setString(1, projectName);
      try (ResultSet rs = stmt.executeQuery()) {
        if (!rs.next()) {
          // Create new project
          String insertQuery = "INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (?, ?, ?, ?)";
          try (PreparedStatement insertStmt = conn.prepareStatement(insertQuery)) {
            insertStmt.setInt(1, projectManagerID);
            insertStmt.setString(2, projectName);
            insertStmt.setString(3, "");
            insertStmt.setString(4, "");
            return insertStmt.executeUpdate() > 0;
          }
        } else {
          // Update existing project
          int projectId = rs.getInt("id");
          String updateQuery = "UPDATE Project SET ProjectManagerID = ? WHERE id = ?";
          try (PreparedStatement updateStmt = conn.prepareStatement(updateQuery)) {
            updateStmt.setInt(1, projectManagerID);
            updateStmt.setInt(2, projectId);
            return updateStmt.executeUpdate() > 0;
          }
        }
      }
    }
  }

}