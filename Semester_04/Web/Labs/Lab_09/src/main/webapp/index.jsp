<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Hello World JSP</title>
</head>
<body>
    <h1>Hello World from JSP!</h1>
    <p>Current time: <%= new java.util.Date() %></p>
    <p><a href="hello">Go to Servlet</a></p>
    
    <hr>
    <h2>Your Project Structure:</h2>
    <ul>
        <li><strong>index.jsp</strong> - This JSP page</li>
        <li><strong>HelloServlet.java</strong> - Basic servlet</li>
        <li><strong>web.xml</strong> - Deployment descriptor</li>
        <li><strong>pom.xml</strong> - Maven configuration</li>
    </ul>
    
    <p><em>Now you can implement your login system and snake game!</em></p>
</body>
</html> 