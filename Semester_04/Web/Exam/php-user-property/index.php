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
$user = new User($db);

$success_message = '';
$error_message = '';

$filteredProperties = [];
$my_properties_stmt = $userToProperties->readAllByUserID($_SESSION['userId']);
$my_properties = [];
if ($my_properties_stmt) {
    while ($row = $my_properties_stmt->fetch(PDO::FETCH_ASSOC)) {
        $my_properties[] = $row;
    }
}

$properties_with_more_than_one_owner = [];
$properties_with_more_than_one_owner_stmt = $userToProperties->getPropertiesWithMoreThanOneOwner();
if ($properties_with_more_than_one_owner_stmt) {
    while ($row = $properties_with_more_than_one_owner_stmt->fetch(PDO::FETCH_ASSOC)) {
        $properties_with_more_than_one_owner[] = $row;
    }
}

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
            $my_properties_stmt = $userToProperties->readAllByUserID($_SESSION['userId']);
            $my_properties = [];
            if ($my_properties_stmt) {
                while ($row = $my_properties_stmt->fetch(PDO::FETCH_ASSOC)) {
                    $my_properties[] = $row;
                }
            }
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
            $my_properties_stmt = $userToProperties->readAllByUserID($_SESSION['userId']);
                $my_properties = [];
                if ($my_properties_stmt) {
                    while ($row = $my_properties_stmt->fetch(PDO::FETCH_ASSOC)) {
                        $my_properties[] = $row;
                    }
                }
        } else {
            $error_message = "Did not add property";
        }

    } else if (isset($_POST['delete_property_id']) && !empty(trim($_POST['delete_property_id']))) {
        // First, get the property ID from the UserToProperties record
        $userToPropertyId = $_POST['delete_property_id'];
        
        // Get the actual property ID before deleting
        $query = "SELECT idProperty FROM UserToProperties WHERE id = ?";
        $stmt_get_prop = $db->prepare($query);
        $stmt_get_prop->bindParam(1, $userToPropertyId);
        $stmt_get_prop->execute();
        $result = $stmt_get_prop->fetch(PDO::FETCH_ASSOC);
        
        if ($result) {
            $actualPropertyId = $result['idProperty'];
            
            // Check number of owners BEFORE deleting from UserToProperties
            $stmt2 = $userToProperties->getNumberOfOwners($actualPropertyId);
            $numberOfOwners = $stmt2['COUNT(*)'];
            
            // Delete from UserToProperties table
            $stmt = $userToProperties->deleteById($userToPropertyId);
            
            if ($stmt) {
                // If this was the only owner, also delete from Property table
                if ($numberOfOwners == 1) {
                    $stmt3 = $property->deleteById($actualPropertyId);
                    if ($stmt3) {
                        $success_message = "Property and its association removed successfully!";
                    } else {
                        $success_message = "Property association removed, but failed to delete property record.";
                    }
                } else {
                    $success_message = "Property removed from your list successfully!";
                }
                
                // Refresh the properties list after deletion
                $my_properties_stmt = $userToProperties->readAllByUserID($_SESSION['userId']);
                $my_properties = [];
                if ($my_properties_stmt) {
                    while ($row = $my_properties_stmt->fetch(PDO::FETCH_ASSOC)) {
                        $my_properties[] = $row;
                    }
                }
            } else {
                $error_message = "Failed to remove property";
            }
        } else {
            $error_message = "Property not found";
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

            <h2 class="text-xl font-bold text-neutral-800 mb-4">My properties</h2>
            <?php if (!empty($my_properties)): ?>
                <?php foreach ($my_properties as $my_property): ?>
                    <div class="flex flex-row gap-2 items-center justify-between p-4 border border-gray-300 rounded-md mb-2 bg-gray-50">
                        <div class="flex-grow">
                            <p class="text-sm text-gray-600">Property ID: <?php echo htmlspecialchars($my_property['idProperty']); ?></p>
                            <h3 class="font-semibold text-lg"><?php echo htmlspecialchars($my_property['address']); ?></h3>
                            <p class="text-gray-700"><?php echo htmlspecialchars($my_property['description'] ?: 'No description'); ?></p>
                        </div>
                        <form method="post" action="index.php" class="flex-shrink-0">
                            <input type="hidden" name="delete_property_id" value="<?php echo htmlspecialchars($my_property['id']); ?>">
                            <button type="submit" class="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded-md" onclick="return confirm('Are you sure you want to remove this property from your list?')">Remove</button>
                        </form>
                    </div>
                <?php endforeach; ?>
            <?php else: ?>
                <p class="text-gray-600 italic">You don't have any properties yet.</p>
            <?php endif; ?>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Properties that have more than one owner</h2>
            <?php if (!empty($properties_with_more_than_one_owner)): ?>
                <?php foreach ($properties_with_more_than_one_owner as $property): ?>
                    <div class="flex flex-row gap-2 items-center justify-between p-4 border border-gray-300 rounded-md mb-2 bg-gray-50">
                        <div class="flex-grow">
                            <p class="text-sm text-gray-600">Property ID: <?php echo htmlspecialchars($property['id']); ?></p>
                            <h3 class="font-semibold text-lg"><?php echo htmlspecialchars($property['address']); ?></h3>
                            <p class="text-gray-700"><?php echo htmlspecialchars($property['description'] ?: 'No description'); ?></p>
                        </div>
                    </div>
                <?php endforeach; ?>
            <?php else: ?>
                <p class="text-gray-600 italic">No properties have more than one owner.</p>
            <?php endif; ?>






        </div>
    </div>
</div>