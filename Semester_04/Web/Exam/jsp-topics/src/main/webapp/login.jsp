<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Login - Project Management</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-gray-100 min-h-screen flex items-center justify-center">

<div class="max-w-md w-full mx-auto">
    <div class="mb-6">
        <h1 class="text-3xl font-bold text-neutral-800 mb-4">Login</h1>
        <div class="bg-white p-4 rounded shadow-md mb-6">
            <% String errorMessage = (String) request.getAttribute("error_message"); %>
            <% if (errorMessage != null && !errorMessage.isEmpty()) { %>
                <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                    <%= errorMessage %>
                </div>
            <% } %>
            
            <form method="post" action="login">
                <div class="mb-4">
                    <input type="text" name="name" placeholder="name" 
                           class="w-full p-2 border border-gray-300 rounded" required>
                    <input type="text" name="searchParam" placeholder="Document or Movie (name or id)" 
                           class="w-full p-2 border border-gray-300 rounded" required>
                </div>
                <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Login</button>
            </form>
        </div>
    </div>
</div>

</body>
</html> 