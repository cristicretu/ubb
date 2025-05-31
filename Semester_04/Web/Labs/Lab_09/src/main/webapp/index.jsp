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

        <!-- Main Content -->
        <div class="grid md:grid-cols-2 gap-6 mb-8">
            <!-- Welcome Card -->
            <div class="bg-white rounded-lg shadow-md p-6 border border-gray-200">
                <h2 class="text-2xl font-semibold text-gray-800 mb-4">
                    <% if (isLoggedIn) { %>
                        Ready to Play!
                    <% } else { %>
                        Welcome!
                    <% } %>
                </h2>
                <p class="text-gray-600 mb-4">Current time: <span class="font-mono text-blue-600"><%= new java.util.Date() %></span></p>
                <% if (isLoggedIn) { %>
                    <p class="text-gray-600 mb-4">You're logged in and ready to start gaming!</p>
                <% } else { %>
                    <p class="text-gray-600 mb-4">Please login or register to start playing and tracking your scores.</p>
                <% } %>
                <a href="hello" class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded transition-colors duration-200 inline-block">Test Servlet Connection</a>
            </div>

            <!-- Quick Actions -->
            <div class="bg-white rounded-lg shadow-md p-6 border border-gray-200">
                <h2 class="text-2xl font-semibold text-gray-800 mb-4">Quick Actions</h2>
                <div class="space-y-3">
                    <% if (isLoggedIn) { %>
                        <button onclick="startSnakeGame()" class="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200">🐍 Play Snake Game</button>
                    <% } else { %>
                        <a href="/login/index.jsp" class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200 block text-center">🔑 Login</a>
                        <a href="/register/index.jsp" class="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200 block text-center">📝 Register</a>
                    <% } %>
                </div>
            </div>
        </div>

    </div>

    <script>
        function startSnakeGame() {
            alert('Snake game coming soon! 🐍');
            // TODO: Implement snake game
        }
    </script>
</body>
</html> 