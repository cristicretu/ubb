<?php
session_start();
include_once 'config/database.php';
include_once 'models/User.php';
include_once 'includes/header.php';

$error_message = '';
$database = new Database();
$db = $database->getConnection();
$user = new User($db);

if ($_POST) {
    $username = $_POST['username'];
    $password = $_POST['password'];

    if (empty($username) || empty($password)) {
        $error_message = 'pls input username and pass';
    }

    $stmt = $user->findOne($username);
    $row = $stmt->fetch(PDO::FETCH_ASSOC);

    if (!$row) {
        $error_message = 'User not found';
    } else {
        if ($password == $row['password']) {
            $_SESSION['currentUser'] = $username;
            $_SESSION['userId'] = $row['id'];
            header('Location: index.php');
            exit();
        } else {
            $error_message = 'Invalid password';
        }
    }
}

?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Login</h1>
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <?php if (!empty($error_message)): ?>
            <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                <?php echo htmlspecialchars($error_message); ?>
            </div>
        <?php endif; ?>

        <form method="post" action="login.php" class="flex flex-col gap-2">
            <input type="text" name="username" placeholder="Username" required class="border border-gray-300 rounded-md p-2" />
            <input type="number" name="password" placeholder="*******" required class="border border-gray-300 rounded-md p-2" />
            <button type="submit" class="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md">
                Login
            </button>
        </form>
        
    </div>
</div>

<?php include_once 'includes/footer.php'; ?> 