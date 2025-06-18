import java.io.IOException;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
      if ("add_product".equals(action)) {
        String productName = request.getParameter("product_name");
        String productDescription = request.getParameter("product_description");

        if (productName.isEmpty() || productDescription.isEmpty()) {
          errorMessage = "Product name and description cannot be empty";
          return;
        }

        String query1 = "INSERT INTO Products (name, description) VALUES (?, ?)";
        try (PreparedStatement stmt1 = conn.prepareStatement(query1)) {
          stmt1.setString(1, productName);
          stmt1.setString(2, productDescription);
          stmt1.executeUpdate();
        }

        successMessage = "Product added successfully!";
        response.sendRedirect("main");
      } else if ("search_product".equals(action)) {
        String productName = request.getParameter("name");

        if (productName.isEmpty()) {
          errorMessage = "Product name cannot be empty";
        }

        Map<Integer, String> products = new HashMap<>();

        String query = "SELECT * FROM Products WHERE name LIKE ?";
        try (PreparedStatement stmt = conn.prepareStatement(query)) {
          stmt.setString(1, "%" + productName + "%");
          try (ResultSet rs = stmt.executeQuery()) {
            while (rs.next()) {
              products.put(rs.getInt("id"), rs.getString("name") + " - " + rs.getString("description"));
            }
          }
        }

        if (errorMessage.isEmpty()) {
          request.setAttribute("products", products);
          request.getRequestDispatcher("main.jsp").forward(request, response);
          successMessage = "Product found!";
        }

      } else if ("add_to_cart".equals(action)) {
        String productId = request.getParameter("product_id");

        var oldCart = (List<Integer>) session.getAttribute("cart");
        if (oldCart == null) {
          oldCart = new ArrayList<>();
        }
        oldCart.add(Integer.parseInt(productId));
        session.setAttribute("cart", oldCart);

        successMessage = "Product added to cart!";
        response.sendRedirect("main");
      } else if ("finalize_order".equals(action)) {
        var cart = (List<Integer>) session.getAttribute("cart");
        if (cart == null) {
          errorMessage = "Cart is empty";
          return;
        }

        for (Integer productId : cart) {
          String query = "INSERT INTO Orders (user, productId, quantity) VALUES (?, ?, ?)";
          try (PreparedStatement stmt = conn.prepareStatement(query)) {
            stmt.setString(1, currentUser);
            stmt.setInt(2, productId);
            stmt.setInt(3, 1);
            stmt.executeUpdate();
          }
        }

        session.removeAttribute("cart");
        successMessage = "Order finalized!";
        response.sendRedirect("main");
      }
    } catch (

    SQLException e) {
      errorMessage = "Database error: " + e.getMessage();
    }

    if (!successMessage.isEmpty()) {
      session.setAttribute("success_message", successMessage);
    }
    if (!errorMessage.isEmpty()) {
      session.setAttribute("error_message", errorMessage);
    }

  }
}
