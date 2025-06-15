<?php
session_start();
include_once 'config/database.php';
include_once 'models/User.php';
include_once 'includes/header.php';

$error_message = '';
$database = new Database();
$db = $database->getConnection();
$user = new User($db);

$username = "";
$secretQuestion = "";
$showSecretQuestion = false;

if ($_POST && isset($_POST['username']) && isset($_POST['secretAnswer']) && !empty($_POST['secretAnswer'])) {
    $username = trim($_POST['username']);
    $secretAnswer = trim($_POST['secretAnswer']);

    if (empty($username) || empty($secretAnswer)) {
        $error_message = 'Please enter a username and secret answer';
    } else {
        $stmt = $user->findOne($username);
        $row = $stmt->fetch(PDO::FETCH_ASSOC);

        if (!$row) {
            $error_message = 'User not found';
        } else {
            $secretQuestion = $row['secretQuestion'];
            
            if ($secretAnswer == $row['secretAnswer']) {
                $_SESSION['currentUser'] = $username;
                header('Location: index.php');
                exit();
            } else {
                $error_message = 'Invalid secret answer';
            }
        }
    }
}
else if ($_POST && isset($_POST['username']) && !isset($_POST['secretAnswer'])) {
    $username = trim($_POST['username']);
    
    if (empty($username)) {
        $error_message = 'Please enter a username';
    } else {
        $stmt = $user->findOne($username);
        $row = $stmt->fetch(PDO::FETCH_ASSOC);

        if (!$row) {
            $error_message = 'User not found';
        } else {
            $secretQuestion = $row['secretQuestion'];
            $showSecretQuestion = true;
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
        
        <?php if (!$showSecretQuestion): ?>
            <!-- Step 1: Enter Username -->
            <form method="post" action="login.php">
                <div class="mb-4">
                    <label for="username" class="block text-sm font-medium text-gray-700 mb-2">Username</label>
                    <input type="text" name="username" id="username" placeholder="Enter your username" 
                           class="w-full p-2 border border-gray-300 rounded" 
                           value="<?php echo htmlspecialchars($username); ?>" required>
                </div>
                <button type="submit" class="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md">
                    Show Secret Question
                </button>
            </form>
        <?php else: ?>
            <!-- Step 2: Show Secret Question and Answer Input -->
            <form method="post" action="login.php">
                <div class="mb-4">
                    <label class="block text-sm font-medium text-gray-700 mb-2">Username</label>
                    <input type="text" value="<?php echo htmlspecialchars($username); ?>" 
                           class="w-full p-2 border border-gray-300 rounded bg-gray-100" readonly>
                    <input type="hidden" name="username" value="<?php echo htmlspecialchars($username); ?>">
                </div>
                
                <div class="mb-4">
                    <label class="block text-sm font-medium text-gray-700 mb-2">Secret Question</label>
                    <div class="w-full p-2 border border-gray-300 rounded bg-gray-50 text-gray-700">
                        <?php echo htmlspecialchars($secretQuestion); ?>
                    </div>
                </div>
                
                <div class="mb-4">
                    <label for="secretAnswer" class="block text-sm font-medium text-gray-700 mb-2">Secret Answer</label>
                    <input type="text" name="secretAnswer" id="secretAnswer" placeholder="Enter your secret answer" 
                           class="w-full p-2 border border-gray-300 rounded" required>
                </div>
                
                <div class="flex gap-2">
                    <button type="submit" class="bg-green-500 hover:bg-green-600 text-white px-4 py-2 rounded-md">
                        Login
                    </button>
                    <a href="login.php" class="bg-gray-500 hover:bg-gray-600 text-white px-4 py-2 rounded-md text-decoration-none">
                        Back
                    </a>
                </div>
            </form>
        <?php endif; ?>
    </div>
</div>

<?php include_once 'includes/footer.php'; ?> 