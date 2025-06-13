<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/SoftwareDeveloper.php';

$database = new Database();
$db = $database->getConnection();

$softwareDeveloper = new SoftwareDeveloper($db);

$name = isset($_GET['name']) ? $_GET['name'] : die(json_encode(array("success" => false, "message" => "Software Developer Name is required.")));

$found = $softwareDeveloper->findOne($name);

if ($found) {
    $softwareDeveloper_arr = [
        "id" => $softwareDeveloper->id,
        "name" => $softwareDeveloper->name,
        "age" => $softwareDeveloper->age,
        "skills" => $softwareDeveloper->skills,
    ];

    http_response_code(200);
    echo json_encode(["success" => true, "record" => $softwareDeveloper_arr]);
} else {
    http_response_code(404);
    echo json_encode(["success" => false, "message" => "Software Developer not found."]);
}
?> 