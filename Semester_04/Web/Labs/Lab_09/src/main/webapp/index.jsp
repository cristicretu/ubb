<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Game Hub - JSP & Servlets</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 p-4">
    <div class="max-w-4xl mx-auto">
        <!-- Header -->
        <header class="text-center mb-8">
            <h1 class="text-4xl font-bold text-gray-800 mb-2">🎮 Game Hub</h1>
            <p class="text-gray-600">Welcome to your JSP & Servlets Gaming Platform</p>
        </header>

        <!-- Main Content -->
        <div class="grid md:grid-cols-2 gap-6 mb-8">
            <!-- Welcome Card -->
            <div class="bg-white rounded-lg shadow-md p-6 border border-gray-200">
                <h2 class="text-2xl font-semibold text-gray-800 mb-4">Welcome!</h2>
                <p class="text-gray-600 mb-4">Current time: <span class="font-mono text-blue-600"><%= new java.util.Date() %></span></p>
                <a href="hello" class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded transition-colors duration-200 inline-block">Test Servlet Connection</a>
            </div>

            <!-- Quick Actions -->
            <div class="bg-white rounded-lg shadow-md p-6 border border-gray-200">
                <h2 class="text-2xl font-semibold text-gray-800 mb-4">Quick Actions</h2>
                <div class="space-y-3">
                    <button class="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200">🐍 Play Snake Game</button>
                    <button class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200">👤 Login / Register</button>
                    <button class="bg-gray-500 hover:bg-gray-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200">📊 View Leaderboard</button>
                </div>
            </div>
        </div>

        <!-- Footer -->
        <footer class="text-center mt-8 text-gray-500 text-sm">
            <p>Built with ❤️ using JSP, Servlets & Tailwind CSS</p>
        </footer>
    </div>
</body>
</html> 