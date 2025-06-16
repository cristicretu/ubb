<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ page import="java.util.List" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Project Management</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-gray-100 min-h-screen">

<div class="container mx-auto px-4 py-8">
    <div class="mb-6">
        <h1 class="text-3xl font-bold text-neutral-800 mb-4">Document and Movie Management</h1>
        <p class="text-gray-600 mb-4">
            Welcome, <strong><%= session.getAttribute("currentUser") %></strong>! 
            <a href="logout" class="text-blue-500 hover:underline">Logout</a>
        </p>
        
        <%
            String successMessage = (String) session.getAttribute("success_message");
            String errorMessage = (String) session.getAttribute("error_message");
            
            if (successMessage != null) {
                session.removeAttribute("success_message");
            }
            if (errorMessage != null) {
                session.removeAttribute("error_message");
            }
        %>
        
        <% if (successMessage != null && !successMessage.isEmpty()) { %>
            <div class="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-4">
                <%= successMessage %>
            </div>
        <% } %>
        
        <% if (errorMessage != null && !errorMessage.isEmpty()) { %>
            <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                <%= errorMessage %>
            </div>
        <% } %>
        
        <div class="bg-white p-4 rounded shadow-md mb-6">
            <div class="overflow-x-auto">
                <h2 class="text-lg font-bold text-neutral-800 mb-4">All your authored movies and documents</h2>

                <div>
                    <% 
                    java.util.List documentsAndMovies = (java.util.List)request.getAttribute("documentsAndMovies");
                    if (documentsAndMovies != null) {
                        for (int i = 0; i < documentsAndMovies.size(); ++i) { 
                            String item = (String)documentsAndMovies.get(i);
                            if (item.startsWith("document")) {
                    %>
                            <div class="bg-red-100">
                                <%= documentsAndMovies.get(i) %>
                            </div>
                    <% 
                            } else if (item.startsWith("movie")) { 
                    %>
                            <form method="post" action="main">
                                <input type="hidden" name="movieId" value="<%= item.split("\\.")[1] %>">
                                <%= documentsAndMovies.get(i) %>
                                <button type="submit" name="action" value="delete_movie">Delete</button>
                            </form>
                    <% 
                            }
                        }
                    } else { 
                    %>
                        <p class="text-gray-500">No documents or movies found.</p>
                    <% } %>
                </div>
            </div>
        </div>

        <div class="bg-white p-4 rounded shadow-md mb-6">
            <h2 class="text-lg font-bold text-neutral-800 mb-4">Add a new document</h2>

            <form method="post" action="main" class="space-y-4">
                <input type="text" name="document_name" placeholder="Document Name" 
                       class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                <input type="text" name="document_content" placeholder="Document Content" 
                       class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                <button type="submit" name="action" value="add_document" 
                        class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">
                    Add Document
                </button>
            </form>
        </div>

        <div class="bg-white p-4 rounded shadow-md mb-6">
            <h2 class="text-lg font-bold text-neutral-800 mb-4">Document with the largest number of authors</h2>

            <div>
                <% 
                String documentsWithMostAuthors = (String) request.getAttribute("documentsWithMostAuthors");
                if (documentsWithMostAuthors != null) {
                    String[] parts = documentsWithMostAuthors.split("\\.");
                    %>
                    <div class="bg-blue-100 p-3 rounded">
                        <strong>Title:</strong> <%= parts[0] %><br>
                        <% if (parts.length > 1) { %>
                            <strong>ID:</strong> <%= parts[1] %>
                        <% } %>
                    </div>
                <% } else { %>
                    <p class="text-gray-500">No documents with the largest number of authors found.</p>
                <% } %>
            </div>
        </div>
    </div>
</div>