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
                <h2>All your authored movies and documents</h2>

                <div>
                    <% for (int i = 0; i < request.getAttribute("documentsAndMovies").size(); ++i) { %>
                        <% if (i % 2 == 0) { %>
                            <div class="bg-red-100">
                                <%= ((java.util.List)request.getAttribute("documentsAndMovies")).get(i) %>
                                </div>

                        <% } else { %>
                            <form>
                                <%= ((java.util.List)request.getAttribute("documentsAndMovies")).get(i) %>
                            </form>

                        <%} %>
                    <% } %>
                </div>
            </div>
        </div>
    </div>
</div>