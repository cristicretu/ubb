<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Snake - Login</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="min-h-screen flex items-center justify-center bg-gradient-to-br from-blue-50 to-indigo-100">
    <div class="w-full max-w-md mx-auto">
        <!-- Header -->
        <header class="text-center mb-8">
            <h1 class="text-4xl font-bold text-gray-800 mb-2">🐍 Snake Game Login</h1>
            <p class="text-gray-600">Sign in to play and track your high scores!</p>
        </header>

        <!-- Login Card -->
        <div class="bg-white rounded-lg shadow-lg p-8 border border-gray-200">
            <%
                String error = (String) request.getAttribute("error");
                String username = (String) request.getAttribute("username");
                if (username == null) username = "";
            %>
            
            <form action="/user?action=login" method="post" class="space-y-6">
                <div>
                    <label for="username" class="block text-gray-700 font-semibold mb-1">Username</label>
                    <input type="text" id="username" name="username" required autofocus
                        value="<%= username %>"
                        class="w-full px-4 py-2 border <%= error != null && error.contains("Username") ? "border-red-400" : "border-gray-300" %> rounded focus:outline-none focus:ring-2 focus:ring-blue-400" />
                    <% if (error != null && error.contains("Username")) { %>
                        <p class="text-red-600 text-sm mt-1"><%= error %></p>
                    <% } %>
                </div>
                
                <div>
                    <label for="password" class="block text-gray-700 font-semibold mb-1">Password</label>
                    <input type="password" id="password" name="password" required
                        class="w-full px-4 py-2 border <%= error != null && (error.contains("Password") || error.contains("Invalid")) ? "border-red-400" : "border-gray-300" %> rounded focus:outline-none focus:ring-2 focus:ring-blue-400" />
                    <% if (error != null && (error.contains("Password") || error.contains("Invalid"))) { %>
                        <p class="text-red-600 text-sm mt-1"><%= error %></p>
                    <% } %>
                </div>
                
                <button type="submit"
                    class="w-full bg-blue-600 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded transition-colors duration-200">
                    Login
                </button>
            </form>
            
            <div class="mt-6 text-center text-gray-600">
                Don't have an account?
                <a href="/register/index.jsp" class="text-blue-600 hover:underline font-semibold">Register here</a>
            </div>
        </div>
    </div>
</body>
</html>