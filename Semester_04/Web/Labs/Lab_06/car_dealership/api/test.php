<?php
header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

$response = [
    "message" => "costel!",
    "timestamp" => date("Y-m-d H:i:s")
];

echo json_encode($response);
?> 