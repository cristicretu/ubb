<?php
session_start();
include_once 'config/database.php';
include_once 'models/Websites.php';
include_once 'models/Documents.php';
include_once 'includes/header.php';

// if (!isset($_SESSION['currentUser'])) {
//     header('Location: login.php');
//     exit();
// }

$database = new Database();
$db = $database->getConnection();
$websites = new Websites($db);
$documents = new Documents($db);

$success_message = '';
$error_message = '';

$websites_stmt = $websites->readAllWithDocumentsCount();
$websites_array = [];
if ($websites_stmt) {
    while ($row = $websites_stmt->fetch(PDO::FETCH_ASSOC)) {
        $websites_array[] = $row;
    }
}

$search_results = [];

if ($_POST) {
    if (isset($_POST['action']) && $_POST['action'] == 'update_keywords') {
        $website_id = $_POST['website_id'];
        $keywords = $_POST['keywords'];
        $keywords_array = explode(',', $keywords);
        if (empty($keywords_array) || count($keywords_array) != 5) {
            $error_message = 'Please enter 5 keywords separated by commas';
        }
        else {
            $result = $documents->update($website_id, $keywords_array[0], $keywords_array[1], $keywords_array[2], $keywords_array[3], $keywords_array[4]);
            if ($result) {
                $success_message = 'Keywords updated successfully';
            }
            else {
                $error_message = 'Failed to update keywords';
            }
        }
    } else if (isset($_POST['action']) && $_POST['action'] == 'search_by_keywords') {
        $keywords = $_POST['keywords'];
        $keywords_array = explode(',', $keywords);
        if (empty($keywords_array)) {
            $error_message = 'Please enter keywords';
        }
        else {
            $result = $documents->searchByKeywords($keywords_array);
            if ($result) {
                $search_results = $result->fetchAll(PDO::FETCH_ASSOC);
            }
        }
    }

}


?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Website Management</h1>
    
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
    
    <div class="bg-white p-4 rounded shadow-md mb-6 w-full">
        <div class="overflow-x-auto w-full">

          <h2 class="font-bold text-lg"> Websites </h2> 
          <table class="text-center w-full">
                <tr>
                    <th>ID</th>
                    <th>URL</th>
                    <th>No. of Documents</th>
                </tr>
                <?php foreach ($websites_array as $website) : ?>
                    <tr>
                        <td><?php echo isset($website['id']) ? htmlspecialchars($website['id']) : 'N/A'; ?></td>
                        <td><?php echo isset($website['URL']) ? htmlspecialchars($website['URL']) : 'N/A'; ?></td>
                        <td><?php echo isset($website['document_count']) ? htmlspecialchars($website['document_count']) : '0'; ?></td>
                    </tr>
                <?php endforeach; ?>
            </table>

            <h2 class="font-bold text-lg mt-4"> Update Keywords </h2> 
            <form action="index.php" method="post" class="flex flex-col gap-2 my-4">
                <input type="text" name="website_id" placeholder="Website ID" class="w-full p-2 border border-gray-300 rounded">
                <input type="text" name="keywords" placeholder="Keywords (separated by comma)" class="w-full p-2 border border-gray-300 rounded">
                <button type="submit" name="action" value="update_keywords" class="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md">Update Keywords</button>
            </form>

            <h2 class="font-bold text-lg mt-4"> Search by 3 keywords </h2> 
            <form action="index.php" method="post" class="flex flex-col gap-2 my-4">
                <input type="text" name="keywords" placeholder="Keywords (separated by comma)" class="w-full p-2 border border-gray-300 rounded">
                <button type="submit" name="action" value="search_by_keywords" class="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md">Search</button>
            </form>

            <?php if (!empty($search_results)): ?>
                <h2 class="font-bold text-lg mt-4"> Search Results </h2> 
                <table class="text-center w-full">
                    <tr>
                        <th>Document ID</th>
                        <th>Website ID</th>
                        <th>Document Name</th>
                        <th>Keyword 1</th>
                        <th>Keyword 2</th>
                        <th>Keyword 3</th>
                        <th>Keyword 4</th>
                        <th>Keyword 5</th>
                    </tr>
                    <?php foreach ($search_results as $result) : ?>
                        <tr>
                            <td><?php echo isset($result['id']) ? htmlspecialchars($result['id']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['idWebsite']) ? htmlspecialchars($result['idWebsite']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['name']) ? htmlspecialchars($result['name']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['keyword1']) ? htmlspecialchars($result['keyword1']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['keyword2']) ? htmlspecialchars($result['keyword2']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['keyword3']) ? htmlspecialchars($result['keyword3']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['keyword4']) ? htmlspecialchars($result['keyword4']) : 'N/A'; ?></td>
                            <td><?php echo isset($result['keyword5']) ? htmlspecialchars($result['keyword5']) : 'N/A'; ?></td>
                        </tr>
                    <?php endforeach; ?>
                </table>
            <?php endif; ?>





        </div>
    </div>
</div>