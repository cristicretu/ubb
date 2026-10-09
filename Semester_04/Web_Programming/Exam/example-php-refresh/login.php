<?php
session_start();
include_once 'config/database.php';
include_once 'includes/header.php';

$error_message = '';

// Handle login form submission
if ($_POST && isset($_POST['username'])) {
    $username = trim($_POST['username']);
    
    if (empty($username)) {
        $error_message = 'Please enter a username';
    } else {
        // Store username in session
        $_SESSION['currentUser'] = $username;
        // Redirect to index page
        header('Location: index.php');
        exit();
    }
}
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4"> Login</h1>
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <?php if (!empty($error_message)): ?>
            <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                <?php echo htmlspecialchars($error_message); ?>
            </div>
        <?php endif; ?>
        
        <form method="post" action="login.php">
            <div class="mb-4">
                <input type="text" name="username" placeholder="Username" class="w-full p-2 border border-gray-300 rounded" required>
            </div>
            <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Login</button>
        </form>
    </div>
</div>

<?php include_once 'includes/footer.php'; ?> 