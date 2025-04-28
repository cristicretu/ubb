<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");
header("Access-Control-Allow-Methods: POST");
header("Access-Control-Max-Age: 3600");
header("Access-Control-Allow-Headers: Content-Type, Access-Control-Allow-Headers, Authorization, X-Requested-With");

include_once '../../config/database.php';
include_once '../../models/Car.php';

$database = new Database();
$db = $database->getConnection();

$car = new Car($db);

$data = $_POST;

if (
    !empty($data['model']) &&
    !empty($data['engine_power']) &&
    !empty($data['fuel_type']) &&
    !empty($data['price']) &&
    !empty($data['color']) &&
    !empty($data['year']) &&
    !empty($data['category_id'])
) {
    $car->model = $data['model'];
    $car->engine_power = $data['engine_power'];
    $car->fuel_type = $data['fuel_type'];
    $car->price = $data['price'];
    $car->color = $data['color'];
    $car->year = $data['year'];
    $car->history = isset($data['history']) ? $data['history'] : "";
    $car->category_id = $data['category_id'];
    $car->created_at = date('Y-m-d H:i:s');

    if ($car->create()) {
        http_response_code(201);
        echo json_encode(array("success" => true, "message" => "Car was created."));
    } else {
        http_response_code(503);
        echo json_encode(array("success" => false, "message" => "Unable to create car."));
    }
} else {
    http_response_code(400);
    echo json_encode(array("success" => false, "message" => "Unable to create car. Data is incomplete."));
}
?> 