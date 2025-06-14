<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/Project.php';

$database = new Database();
$db = $database->getConnection();

$project = new Project($db);

$projectManagerID = isset($_GET['projectManagerID']) ? $_GET['projectManagerID'] : null;

$stmt = $project->readAllByProjectManagerID($projectManagerID);

$projects_arr = array();
$projects_arr["records"] = array();

while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
    extract($row);

    $project_item = array(
        "id" => $id,
        "ProjectManagerID" => $ProjectManagerID,
        "name" => $name,
        "description" => $description,
        "members" => $members
        );

    array_push($projects_arr["records"], $project_item);
}

http_response_code(200);
echo json_encode($projects_arr);
?>