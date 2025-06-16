import java.io.IOException;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
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

  public ArrayList<Object> getDocumentsAndMovies(Connection conn, String currentUser) throws SQLException {
    ArrayList<Object> documentsAndMovies = new ArrayList<>();
    ArrayList<Integer> documentIds = new ArrayList<>();
    ArrayList<Integer> movieIds = new ArrayList<>();
    String query = "select documentList, movieList FROM Authors WHERE name = ?";
    try (PreparedStatement stmt = conn.prepareStatement(query)) {
      stmt.setString(1, currentUser);
      try (ResultSet rs = stmt.executeQuery()) {
        while (rs.next()) {
          String documentList = rs.getString("documentList");
          String movieList = rs.getString("movieList");

          if (documentList != null && !documentList.trim().isEmpty()) {
            documentIds = (ArrayList<Integer>) Arrays.asList(documentList.split(",")).stream()
                .filter(s -> !s.trim().isEmpty())
                .map(Integer::parseInt)
                .collect(Collectors.toList());
          }

          if (movieList != null && !movieList.trim().isEmpty()) {
            movieIds = (ArrayList<Integer>) Arrays.asList(movieList.split(",")).stream()
                .filter(s -> !s.trim().isEmpty())
                .map(Integer::parseInt)
                .collect(Collectors.toList());
          }
        }
      }
    }

    System.out.println("DEBUG:" + movieIds);
    System.out.println("DEBUGGG" + documentIds);

    ArrayList<Object> documents = new ArrayList<>();
    ArrayList<Object> movies = new ArrayList<>();

    for (Integer documentId : documentIds) {
      String query2 = "SELECT * FROM Documents WHERE id = ?";
      try (PreparedStatement stmt2 = conn.prepareStatement(query2)) {
        stmt2.setInt(1, documentId);
        try (ResultSet rs2 = stmt2.executeQuery()) {
          while (rs2.next()) {
            String title = rs2.getString("name");
            String contents = rs2.getString("contents");
            documents.add("document: " + title + " " + contents + "." + documentId);
          }
        }
      } catch (SQLException e) {
        throw e;
      }
    }

    for (Integer movieId : movieIds) {
      System.out.println("DEBUG: Querying movie with ID: " + movieId);
      String query3 = "SELECT * FROM Movies WHERE id = ?";
      try (PreparedStatement stmt3 = conn.prepareStatement(query3)) {
        stmt3.setInt(1, movieId);
        try (ResultSet rs3 = stmt3.executeQuery()) {
          while (rs3.next()) {
            String title = rs3.getString("title");
            int duration = rs3.getInt("duration");
            movies.add("movie: " + title + " " + duration + "." + movieId);
          }
        }
      } catch (SQLException e) {
        throw e;
      }
    }

    // Interleave documents and movies
    int docIndex = 0;
    int movieIndex = 0;

    while (docIndex < documents.size() || movieIndex < movies.size()) {
      // Add document if available and it's even index
      if (docIndex < documents.size() && documentsAndMovies.size() % 2 == 0) {
        documentsAndMovies.add(documents.get(docIndex));
        docIndex++;
      }
      // Add movie if available and it's odd index
      else if (movieIndex < movies.size() && documentsAndMovies.size() % 2 == 1) {
        documentsAndMovies.add(movies.get(movieIndex));
        movieIndex++;
      }
      // If one list is exhausted, add from the other
      else if (docIndex < documents.size()) {
        documentsAndMovies.add(documents.get(docIndex));
        docIndex++;
      } else if (movieIndex < movies.size()) {
        documentsAndMovies.add(movies.get(movieIndex));
        movieIndex++;
      }
    }

    return documentsAndMovies;
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
      ArrayList<Object> documentsAndMovies = getDocumentsAndMovies(conn, currentUser);
      request.setAttribute("documentsAndMovies", documentsAndMovies);

    } catch (SQLException e) {
      request.setAttribute("error_message", "Database error: " + e.getMessage());
      // Set empty list to avoid null pointer exception in JSP
      request.setAttribute("documentsAndMovies", new ArrayList<Object>());
    } catch (Exception e) {
      request.setAttribute("error_message", "Unexpected error: " + e.getMessage());
      // Set empty list to avoid null pointer exception in JSP
      request.setAttribute("documentsAndMovies", new ArrayList<Object>());
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
      if ("delete_movie".equals(action)) {
        String movieId = request.getParameter("movieId");
        String query = "DELETE FROM Movies WHERE id = ?";
        String movieList = "";
        try (PreparedStatement stmt = conn.prepareStatement(query)) {
          stmt.setInt(1, Integer.parseInt(movieId));
          stmt.executeUpdate();
        }

        String query2 = "SELECT movieList FROM Authors WHERE name = ?";
        try (PreparedStatement stmt2 = conn.prepareStatement(query2)) {
          stmt2.setString(1, currentUser);
          try (ResultSet rs = stmt2.executeQuery()) {
            while (rs.next()) {
              movieList = rs.getString("movieList");
            }
          }
        }

        String newMovieList = movieList.replace(movieId, "");

        String query1 = "UPDATE Authors SET movieList = ? WHERE name = ?";
        try (PreparedStatement stmt1 = conn.prepareStatement(query1)) {
          stmt1.setString(1, newMovieList);
          stmt1.setString(2, currentUser);
          stmt1.executeUpdate();
        }
        successMessage = "Movie deleted successfully!";
      }
    } catch (SQLException e) {
      errorMessage = "Database error: " + e.getMessage();
    }

    //
    response.sendRedirect("main");
  }

  // private Integer getCurrentUserID(Connection conn, String currentUser) throws
  // SQLException {
  // String query = "SELECT id FROM SoftwareDeveloper WHERE name = ?";
  // try (PreparedStatement stmt = conn.prepareStatement(query)) {
  // stmt.setString(1, currentUser);
  // try (ResultSet rs = stmt.executeQuery()) {
  // if (rs.next()) {
  // return rs.getInt("id");
  // }
  // }
  // }
  // return null;
  // }

  // private List<ProjectData> getAllProjects(Connection conn) throws SQLException
  // {
  // List<ProjectData> projects = new ArrayList<>();
  // String query = "SELECT id, ProjectManagerID, name, description, members FROM
  // Project";
  // try (PreparedStatement stmt = conn.prepareStatement(query);
  // ResultSet rs = stmt.executeQuery()) {

  // while (rs.next()) {
  // ProjectData project = new ProjectData();
  // project.id = rs.getInt("id");
  // project.projectManagerID = rs.getInt("ProjectManagerID");
  // project.name = rs.getString("name");
  // project.description = rs.getString("description");
  // project.members = rs.getString("members");
  // projects.add(project);
  // }
  // }
  // return projects;
  // }

  // private List<ProjectData> getProjectsByManagerID(Connection conn, Integer
  // managerID) throws SQLException {
  // List<ProjectData> projects = new ArrayList<>();
  // String query = "SELECT id, ProjectManagerID, name, description, members FROM
  // Project WHERE ProjectManagerID = ?";
  // try (PreparedStatement stmt = conn.prepareStatement(query)) {
  // stmt.setInt(1, managerID);
  // try (ResultSet rs = stmt.executeQuery()) {
  // while (rs.next()) {
  // ProjectData project = new ProjectData();
  // project.id = rs.getInt("id");
  // project.projectManagerID = rs.getInt("ProjectManagerID");
  // project.name = rs.getString("name");
  // project.description = rs.getString("description");
  // project.members = rs.getString("members");
  // projects.add(project);
  // }
  // }
  // }
  // return projects;
  // }

  // private List<ProjectData> getMemberProjects(List<ProjectData> allProjects,
  // String currentUser) {
  // List<ProjectData> memberProjects = new ArrayList<>();
  // for (ProjectData project : allProjects) {
  // if (project.members != null && project.members.contains(currentUser)) {
  // memberProjects.add(project);
  // }
  // }
  // return memberProjects;
  // }

  // private List<DeveloperData> getAllDevelopers(Connection conn) throws
  // SQLException {
  // List<DeveloperData> developers = new ArrayList<>();
  // String query = "SELECT id, name, age, skills FROM SoftwareDeveloper";
  // try (PreparedStatement stmt = conn.prepareStatement(query);
  // ResultSet rs = stmt.executeQuery()) {

  // while (rs.next()) {
  // DeveloperData developer = new DeveloperData();
  // developer.id = rs.getInt("id");
  // developer.name = rs.getString("name");
  // developer.age = rs.getInt("age");
  // developer.skills = rs.getString("skills");
  // developers.add(developer);
  // }
  // }
  // return developers;
  // }

  // private Integer findDeveloperByName(Connection conn, String name) throws
  // SQLException {
  // String query = "SELECT id FROM SoftwareDeveloper WHERE name = ?";
  // try (PreparedStatement stmt = conn.prepareStatement(query)) {
  // stmt.setString(1, name);
  // try (ResultSet rs = stmt.executeQuery()) {
  // if (rs.next()) {
  // return rs.getInt("id");
  // }
  // }
  // }
  // return null;
  // }

  // private boolean assignProject(Connection conn, String projectName, Integer
  // projectManagerID) throws SQLException {
  // // Check if project already exists
  // String selectQuery = "SELECT id FROM Project WHERE name = ?";
  // try (PreparedStatement stmt = conn.prepareStatement(selectQuery)) {
  // stmt.setString(1, projectName);
  // try (ResultSet rs = stmt.executeQuery()) {
  // if (!rs.next()) {
  // // Create new project
  // String insertQuery = "INSERT INTO Project (ProjectManagerID, name,
  // description, members) VALUES (?, ?, ?, ?)";
  // try (PreparedStatement insertStmt = conn.prepareStatement(insertQuery)) {
  // insertStmt.setInt(1, projectManagerID);
  // insertStmt.setString(2, projectName);
  // insertStmt.setString(3, "");
  // insertStmt.setString(4, "");
  // return insertStmt.executeUpdate() > 0;
  // }
  // } else {
  // // Update existing project
  // int projectId = rs.getInt("id");
  // String updateQuery = "UPDATE Project SET ProjectManagerID = ? WHERE id = ?";
  // try (PreparedStatement updateStmt = conn.prepareStatement(updateQuery)) {
  // updateStmt.setInt(1, projectManagerID);
  // updateStmt.setInt(2, projectId);
  // return updateStmt.executeUpdate() > 0;
  // }
  // }
  // }
  // }
  // }

}
