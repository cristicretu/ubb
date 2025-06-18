<?php
header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: GET, POST, PUT, DELETE');
header('Access-Control-Allow-Headers: Content-Type');

require_once '../models/SoftwareDeveloper.php';

class SoftwareDeveloperAPI {
    private $developerModel;
    
    public function __construct() {
        $this->developerModel = new SoftwareDeveloper();
    }
    
    public function handleRequest() {
        $method = $_SERVER['REQUEST_METHOD'];
        
        try {
            switch ($method) {
                case 'GET':
                    $this->handleGet();
                    break;
                case 'POST':
                    $this->handlePost();
                    break;
                case 'PUT':
                    $this->handlePut();
                    break;
                case 'DELETE':
                    $this->handleDelete();
                    break;
                default:
                    http_response_code(405);
                    echo json_encode(['error' => 'Method not allowed']);
                    break;
            }
        } catch (Exception $e) {
            http_response_code(500);
            echo json_encode(['error' => 'Database error: ' . $e->getMessage()]);
        }
    }
    
    private function handleGet() {
        $action = isset($_GET['action']) ? $_GET['action'] : '';
        $name = isset($_GET['name']) ? $_GET['name'] : null;
        
        if ($action === 'readAll') {
            $developers = $this->developerModel->getAllDevelopers();
            echo json_encode($developers);
        } elseif ($action === 'findOne' && $name) {
            $developer = $this->developerModel->getDeveloperByName($name);
            if ($developer) {
                echo json_encode($developer);
            } else {
                http_response_code(404);
                echo json_encode(['error' => 'Developer not found']);
            }
        } else {
            $developers = $this->developerModel->getAllDevelopers();
            echo json_encode($developers);
        }
    }
    
    private function handlePost() {
        $name = isset($_POST['name']) ? $_POST['name'] : '';
        $age = isset($_POST['age']) ? $_POST['age'] : '';
        $skills = isset($_POST['skills']) ? $_POST['skills'] : '';
        
        $success = $this->developerModel->createDeveloper($name, $age, $skills);
        echo json_encode(['success' => $success]);
    }
    
    private function handlePut() {
        parse_str(file_get_contents("php://input"), $_PUT);
        
        $id = isset($_PUT['id']) ? $_PUT['id'] : '';
        $name = isset($_PUT['name']) ? $_PUT['name'] : '';
        $age = isset($_PUT['age']) ? $_PUT['age'] : '';
        $skills = isset($_PUT['skills']) ? $_PUT['skills'] : '';
        
        $success = $this->developerModel->updateDeveloper($id, $name, $age, $skills);
        echo json_encode(['success' => $success]);
    }
    
    private function handleDelete() {
        parse_str(file_get_contents("php://input"), $_DELETE);
        $id = isset($_DELETE['id']) ? $_DELETE['id'] : (isset($_GET['id']) ? $_GET['id'] : '');
        
        $success = $this->developerModel->deleteDeveloper($id);
        echo json_encode(['success' => $success]);
    }
}

$api = new SoftwareDeveloperAPI();
$api->handleRequest();
?> 