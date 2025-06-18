<?php
session_start();

// Check if user is logged in
$currentUser = isset($_SESSION['currentUser']) ? $_SESSION['currentUser'] : null;

if ($currentUser !== null) {
    // User is logged in, redirect to main page
    header('Location: main.php');
} else {
    // User is not logged in, redirect to login page
    header('Location: login.php');
}
exit();
?> 