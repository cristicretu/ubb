<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ page import="model.User" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Game Hub - JSP & Servlets</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 p-4">
    <%
        User currentUser = (User) session.getAttribute("user");
        boolean isLoggedIn = currentUser != null;
    %>
    
    <div class="max-w-4xl mx-auto">
        <!-- User Header (if logged in) -->
        <% if (isLoggedIn) { %>
            <div class="bg-white rounded-lg shadow-md p-4 mb-6 border border-gray-200">
                <div class="flex justify-between items-center">
                    <div class="flex items-center space-x-3">
                        <div class="w-10 h-10 bg-blue-500 rounded-full flex items-center justify-center text-white font-bold">
                            <%= currentUser.getUsername().substring(0, 1).toUpperCase() %>
                        </div>
                        <div>
                            <h3 class="font-semibold text-gray-800">Welcome back, <%= currentUser.getUsername() %>!</h3>
                            <p class="text-sm text-gray-600">High Score: <span class="font-bold text-green-600"><%= currentUser.getHighScore() %></span></p>
                        </div>
                    </div>
                    <div>
                        <a href="/user?action=logout" class="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded transition-colors duration-200">Logout</a>
                    </div>
                </div>
            </div>
        <% } %>

        <!-- Header -->
        <header class="text-center mb-8">
            <h1 class="text-4xl font-bold text-gray-800 mb-2">🐍 Snake Game</h1>
        </header>

        <% if (isLoggedIn) { %>
            <div id="game-container"></div>
        <% } else { %>
            <div class="flex justify-center space-x-4">
                <a href="/login/index.jsp" class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200 block text-center">🔑 Login</a>
                <a href="/register/index.jsp" class="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200 block text-center">📝 Register</a>
            </div>
        <% } %>

        

    </div>

    <script>
        const cells = 25;

        
        function startSnakeGame() {
            alert('Snake game coming soon! 🐍');
            // TODO: Implement snake game
        }
    </script>
</body>
</html> 