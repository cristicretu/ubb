<?php
session_start();

class LoginController {
    
    public function showLogin() {
        // If user is already logged in, redirect to main
        if (isset($_SESSION['currentUser'])) {
            header('Location: main.php');
            exit();
        }
        
        include 'views/login.php';
    }
    
    public function handleLogin() {
        $username = isset($_POST['name']) ? trim($_POST['name']) : '';
        $errorMessage = '';
        
        if (empty($username)) {
            $errorMessage = 'Please enter a valid username';
            $_SESSION['error_message'] = $errorMessage;
            include 'views/login.php';
            return;
        }
        
        $_SESSION['currentUser'] = $username;
        header('Location: main.php');
        exit();
    }
    
    public function logout() {
        session_destroy();
        header('Location: login.php');
        exit();
    }
}

// Handle the request
$controller = new LoginController();

if ($_SERVER['REQUEST_METHOD'] === 'GET') {
    if (isset($_GET['action']) && $_GET['action'] === 'logout') {
        $controller->logout();
    } else {
        $controller->showLogin();
    }
} elseif ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $controller->handleLogin();
}
?> 