<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ page import="java.util.List" %>
<%@ page import="java.util.ArrayList" %>
<%@ page import="java.util.Map" %>
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
        <h1 class="text-3xl font-bold text-neutral-800 mb-4">Posts and topics</h1>
        <p class="text-gray-600 mb-4">
            Welcome, <strong><%= session.getAttribute("currentUser") %></strong>! 
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

            Map<Integer, String> products = (Map<Integer, String>) request.getAttribute("products");
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

                <h2 class="text-lg font-bold text-neutral-800 mb-4">Add a new product</h2>
                <form method="post" action="main" class="space-y-4 flex flex-col">
                    <input type="text" name="product_name" placeholder="Product name" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <input type="text" name="product_description" placeholder="Product description" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <button type="submit" name="action" value="add_product" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">Add Product</button>
                </form>

                <h2 class="text-lg font-bold text-neutral-800 mb-4">Products</h2>
                <form method="post" action="main" class="space-y-4 flex flex-col">
                    <input type="text" name="name" placeholder="Product name" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <button type="submit" name="action" value="search_product" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">Search</button>
                </form>

                <% if (products != null && !products.isEmpty()) { %>
                    <ul>
                        <% for (Map.Entry<Integer, String> entry : products.entrySet()) { %>
                            <li>
                                <form method="post" action="main" class="space-y-4 flex flex-row items-center w-full my-8">
                                    <%= entry.getValue() %>
                                    <input type="hidden" name="product_id" value="<%= entry.getKey() %>">
                                    <input type="number" name="quantity" value="1" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500">
                                    <button type="submit" name="action" value="add_to_cart" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">Add to cart</button>
                                </form>
                            </li>
                        <% } %>
                    </ul>
                <% } %>
            
                <h2 class="text-lg font-bold text-neutral-800 my-4">Finalize order</h2>
                <form method="post" action="main" class="space-y-4 flex flex-col">
                    <button type="submit" name="action" value="finalize_order" class="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded">Finalize order</button>
                </form>
              
                <!-- <h2 class="text-lg font-bold text-neutral-800 my-4">Update a post</h2>
                <form method="post" action="main" class="space-y-4 flex flex-col">
                    <input type="text" name="post_id" placeholder="Post ID" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <input type="text" name="post_text" placeholder="Post text" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <input type="text" name="topic_text" placeholder="Topic text" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <button type="submit" name="action" value="update_post" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">Update Post</button>
                </form> -->
            </div>
        </div>

    </div>
</div>