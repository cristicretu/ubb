<?php
include_once '../includes/cors.php';

header("Content-Type: application/json; charset=UTF-8");

include_once '../config/database.php';
include_once '../models/Category.php';

$database = new Database();
$db = $database->getConnection();

$category = new Category($db);

$stmt = $category->readAll();
$num = $stmt->rowCount();

if ($num > 0) {
    $categories_arr = array();
    $categories_arr["records"] = array();

    while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
        extract($row);

        $category_item = array(
            "id" => $id,
            "name" => $name,
            "description" => $description
        );

        array_push($categories_arr["records"], $category_item);
    }
}

echo json_encode($categories_arr);
?>