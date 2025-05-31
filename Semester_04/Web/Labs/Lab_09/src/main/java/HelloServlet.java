import java.io.IOException;
import java.io.PrintWriter;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

public class HelloServlet extends HttpServlet {
  private static final long serialVersionUID = 1L;

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    response.setContentType("text/html");
    PrintWriter out = response.getWriter();

    out.println("<!DOCTYPE html>");
    out.println("<html>");
    out.println("<head><title>Hello World Servlet</title></head>");
    out.println("<body>");
    out.println("<h1>Hello World from Servlet!</h1>");
    out.println("<p>This is a basic JSP/Servlet application.</p>");
    out.println("<a href='index.jsp'>Back to JSP</a>");
    out.println("</body>");
    out.println("</html>");
  }
}