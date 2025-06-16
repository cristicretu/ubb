<?php
session_start();
include_once 'config/database.php';
include_once 'models/User.php';
include_once 'models/Property.php';
include_once 'models/UserToProperties.php';
include_once 'includes/header.php';

if (!isset($_SESSION['currentUser'])) {
    header('Location: login.php');
    exit();
}

$currentUser = $_SESSION['currentUser'];
$database = new Database();
$db = $database->getConnection();
$property = new Property($db);
$userToProperties = new UserToProperties($db);

$success_message = '';
$error_message = '';

$filteredProperties = [];

if ($_POST) {
    if (isset($_POST['description']) && !empty(trim($_POST['description']))) {
        $stmt = $property->searchAll($_POST['description']);

        if ($stmt) {
            while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
                $filteredProperties[] = $row;
            }
            if (count($filteredProperties) > 0) {
                $success_message = "Found " . count($filteredProperties) . " properties.";
            } else {
                $error_message = "No properties found matching your search.";
            }
        }
        else {
            $error_message = "Search failed. Please try again.";
        }
    }else if (isset($_POST['idProperty']) && !empty(trim($_POST['idProperty']))) {
        $stmt = $userToProperties->create($_SESSION['userId'], $_POST['idProperty']);

        if ($stmt) {
            $success_message = "Added property!";
        } else {
            $error_message = "Did not add property";
        }
    } else if (isset($_POST['propAddr']) && !empty(trim($_POST['propAddr'])) && isset($_POST['propDescr']) && !empty(trim($_POST['propDescr']))) {
        $stmt1 = $property->create($_POST['propAddr'], $_POST['propDescr']);

        if (!$stmt1) {
            $error_message = "Did not add property";
            return;
        }

        $stmt = $userToProperties->create($_SESSION['userId'], $stmt1);

        if ($stmt) {
            $success_message = "Added property!";
        } else {
            $error_message = "Did not add property";
        }

    }
    else {
        $error_message = "Error occured.";
    } 
}

?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Propety Management</h1>
    <p class="text-gray-600 mb-4">Welcome, <?php echo htmlspecialchars($currentUser); ?>! 
        <a href="logout.php" class="text-blue-500 hover:underline">Logout</a>
    </p>
    
    <?php if (!empty($success_message)): ?>
        <div class="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-4">
            <?php echo htmlspecialchars($success_message); ?>
        </div>
    <?php endif; ?>
    
    <?php if (!empty($error_message)): ?>
        <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
            <?php echo htmlspecialchars($error_message); ?>
        </div>
    <?php endif; ?>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">

            
            <h2 class="text-xl font-bold text-neutral-800 mb-4">Search properties</h2>
            <form method="post" action="index.php" class="flex flex-col space-y-2 mb-6">
                <input type="text" name="description" placeholder="Description" class="p-2 border border-gray-300 rounded" required>
                <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Search</button>
            </form>

            <?php if (!empty($filteredProperties)): ?>
                <?php foreach ($filteredProperties as $property): ?>
                    <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                        <p>ID: <?php echo htmlspecialchars($property['id']); ?></p>
                        <h3 class="font-semibold"><?php echo htmlspecialchars($property['address']); ?></h3>
                        <p><?php echo htmlspecialchars($property['description'] ?: 'No description'); ?></p>
                    </div>
                <?php endforeach; ?>
            <?php endif; ?>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Add a property to self</h2>
            <form method="post" action="index.php" class="flex flex-col space-y-2 mb-6">
                <input type="text" name="idProperty" placeholder="Property Id" class="p-2 border border-gray-300 rounded" required>
                <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Add</button>
            </form>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">CREATE a new property and then add to self</h2>
            <form method="post" action="index.php" class="flex flex-col space-y-2 mb-6">
                <input type="text" name="propAddr" placeholder="Address" class="p-2 border border-gray-300 rounded" required>
                <input type="text" name="propDescr" placeholder="Description" class="p-2 border border-gray-300 rounded" required>
                <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Add</button>
            </form>


        </div>
    </div>
</div>