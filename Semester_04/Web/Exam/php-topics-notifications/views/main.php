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
            Welcome, <strong><?php echo htmlspecialchars($_SESSION['currentUser']); ?></strong>! 
            <a href="login.php?action=logout" class="text-blue-500 hover:text-blue-700 ml-4">Logout</a>
        </p>
        
        <?php
        $successMessage = isset($_SESSION['success_message']) ? $_SESSION['success_message'] : '';
        $errorMessage = isset($_SESSION['error_message']) ? $_SESSION['error_message'] : '';
        
        if (!empty($successMessage)) {
            unset($_SESSION['success_message']);
        }
        if (!empty($errorMessage)) {
            unset($_SESSION['error_message']);
        }
        ?>
        
        <?php if (!empty($successMessage)) { ?>
            <div class="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-4">
                <?php echo htmlspecialchars($successMessage); ?>
            </div>
        <?php } ?>
        
        <?php if (!empty($errorMessage)) { ?>
            <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                <?php echo htmlspecialchars($errorMessage); ?>
            </div>
        <?php } ?>
        
        <div class="bg-white p-4 rounded shadow-md mb-6">
            <div class="overflow-x-auto">

                <h2 class="text-lg font-bold text-neutral-800 mb-4">Add a new post</h2>
                <form method="post" action="main.php" class="space-y-4 flex flex-col">
                    <input type="text" name="post_text" placeholder="Post text" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <input type="text" name="topic_text" placeholder="Topic text" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <button type="submit" name="action" value="add_post" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">Add Post</button>
                </form>
              
                <h2 class="text-lg font-bold text-neutral-800 my-4">Update a post</h2>
                <form method="post" action="main.php" class="space-y-4 flex flex-col">
                    <input type="text" name="post_id" placeholder="Post ID" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <input type="text" name="post_text" placeholder="Post text" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <input type="text" name="topic_text" placeholder="Topic text" class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500" required>
                    <button type="submit" name="action" value="update_post" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded">Update Post</button>
                </form>
            </div>
        </div>

    </div>
</div>

</body>
</html> 