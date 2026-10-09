<?php
// This file serves as the entry point for the Angular app
// Place this file in the root directory

// Check if the request is for the API
if (strpos($_SERVER['REQUEST_URI'], '/api/') === 0) {
    // Handle API requests with the PHP backend
    require_once __DIR__ . '/index.php';
    exit;
}

// Otherwise, serve the Angular app
$angularIndex = __DIR__ . '/dist/index.html';

if (file_exists($angularIndex)) {
    // Serve the Angular app
    readfile($angularIndex);
} else {
    // If Angular app is not built, redirect to the PHP version
    header('Location: index.php');
}
?> 