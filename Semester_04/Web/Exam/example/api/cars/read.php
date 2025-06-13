<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/Car.php';

$database = new Database();
$db = $database->getConnection();

$car = new Car($db);

$category_id = isset($_GET['category_id']) ? $_GET['category_id'] : null;

$stmt = $car->readAll($category_id);
$num = $stmt->rowCount();

$cars_arr = array();
$cars_arr["records"] = array();

if ($num > 0) {
    while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
        extract($row);

        $car_item = array(
            "id" => $id,
            "model" => $model,
            "engine_power" => $engine_power,
            "fuel_type" => $fuel_type,
            "price" => $price,
            "color" => $color,
            "year" => $year,
            "history" => $history,
            "category_id" => $category_id,
            "features" => $features
            );

        array_push($cars_arr["records"], $car_item);
    }
}

http_response_code(200);
echo json_encode($cars_arr);
?>