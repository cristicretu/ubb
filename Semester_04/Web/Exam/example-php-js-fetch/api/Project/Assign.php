<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/Project.php';

$database = new Database();
$db = $database->getConnection();

$project = new Project($db);

$projectName = isset($_GET['projectName']) ? $_GET['projectName'] : null;
$projectManagerID = isset($_GET['projectManagerID']) ? $_GET['projectManagerID'] : null;

// Validate input parameters
if (empty($projectName)) {
    http_response_code(400);
    echo json_encode(array("message" => "Project name is required."));
    exit;
}

if (empty($projectManagerID)) {
    http_response_code(400);
    echo json_encode(array("message" => "Project manager ID is required."));
    exit;
}

error_log("Assigning project: '$projectName' to manager ID: '$projectManagerID'");
$stmt = $project->assignProject($projectName, $projectManagerID);
error_log("Assignment result: " . ($stmt ? "success" : "null"));

if ($stmt) {
    $row = $stmt->fetch(PDO::FETCH_ASSOC);
    if ($row) {
        extract($row);

        $project_item = array(
            "id" => $id,
            "ProjectManagerID" => $ProjectManagerID,
            "name" => $name,
            "description" => $description,
            "members" => $members
        );

        http_response_code(200);
        echo json_encode(array("record" => $project_item));
    } else {
        http_response_code(400);
        echo json_encode(array("message" => "No project data found."));
    }
} else {
    http_response_code(400);
    echo json_encode(array("message" => "Project assignment failed."));
}
?>