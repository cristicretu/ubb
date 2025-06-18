<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/Car.php';

$database = new Database();
$db = $database->getConnection();

$car = new Car($db);

$car->id = isset($_GET['id']) ? $_GET['id'] : die(json_encode(array("success" => false, "message" => "Car ID is required.")));

$car->readOne();

if ($car->model != null) {
    $car_arr = array(
        "id" => $car->id,
        "model" => $car->model,
        "engine_power" => $car->engine_power,
        "fuel_type" => $car->fuel_type,
        "price" => $car->price,
        "color" => $car->color,
        "year" => $car->year,
        "history" => $car->history,
        "category_id" => $car->category_id,
        "created_at" => $car->created_at,
        "features" => $car->features
    );

    http_response_code(200);
    echo json_encode(array("success" => true, "record" => $car_arr));
} else {
    http_response_code(404);
    echo json_encode(array("success" => false, "message" => "Car not found."));
}
?> 