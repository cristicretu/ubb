<?php
include_once '../../includes/cors.php';

header("Content-Type: application/json; charset=UTF-8");

include_once '../../config/database.php';
include_once '../../models/Car.php';

$database = new Database();
$db = $database->getConnection();

$car = new Car($db);

$car->id = isset($_GET['id']) ? $_GET['id'] : die(json_encode(array("success" => false, "message" => "Car ID is required.")));

if ($car->delete()) {
    http_response_code(200);
    echo json_encode(array("success" => true, "message" => "Car was deleted."));
} else {
    http_response_code(503);
    echo json_encode(array("success" => false, "message" => "Unable to delete car."));
}
?>