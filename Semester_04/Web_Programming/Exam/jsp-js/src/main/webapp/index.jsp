<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%
    // Check if user is logged in
    String currentUser = (String) session.getAttribute("currentUser");
    
    if (currentUser != null) {
        // User is logged in, redirect to main page
        response.sendRedirect("main");
    } else {
        // User is not logged in, redirect to login page
        response.sendRedirect("login.jsp");
    }
%> 