<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/SoftwareDeveloper.php';

$database = new Database();
$db = $database->getConnection();

$softwareDeveloper = new SoftwareDeveloper($db);

$stmt = $softwareDeveloper->readAll();

$softwareDeveloper_arr = array();
$softwareDeveloper_arr["records"] = array();

while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
    extract($row);

    $softwareDeveloper_item = [
        "id" => $id,
        "name" => $name,
        "age" => $age,
        "skills" => $skills,
    ];

    array_push($softwareDeveloper_arr["records"], $softwareDeveloper_item);
}

http_response_code(200);
echo json_encode($softwareDeveloper_arr);
?> 