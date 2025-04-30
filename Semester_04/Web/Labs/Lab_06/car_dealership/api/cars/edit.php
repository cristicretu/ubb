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

if (empty($data['id'])) {
    http_response_code(400);
    echo json_encode(array("success" => false, "message" => "Car ID is required."));
    exit();
}

$car->id = $data['id'];

if (
    !empty($data['model']) &&
    !empty($data['engine_power']) &&
    !empty($data['fuel_type']) &&
    !empty($data['price']) &&
    !empty($data['color']) &&
    !empty($data['year']) &&
    !empty($data['category_id']) &&
    !empty($data['features'])
) {
    $car->model = $data['model'];
    $car->engine_power = $data['engine_power'];
    $car->fuel_type = $data['fuel_type'];
    $car->price = $data['price'];
    $car->color = $data['color'];
    $car->year = $data['year'];
    $car->history = isset($data['history']) ? $data['history'] : "";
    $car->category_id = $data['category_id'];
    $car->features = isset($data['features']) ? $data['features'] : "";

    if ($car->update()) {
        http_response_code(200);
        echo json_encode(array("success" => true, "message" => "Car was updated."));
    } else {
        http_response_code(503);
        echo json_encode(array("success" => false, "message" => "Unable to update car."));
    }
} else {
    http_response_code(400);
    echo json_encode(array("success" => false, "message" => "Unable to update car. Data is incomplete."));
}
?> 