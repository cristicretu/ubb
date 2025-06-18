<?php
header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: GET, POST, PUT, DELETE');
header('Access-Control-Allow-Headers: Content-Type');

require_once '../models/Project.php';

class ProjectAPI {
    private $projectModel;
    
    public function __construct() {
        $this->projectModel = new Project();
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
        $projectManagerId = isset($_GET['projectManagerId']) ? $_GET['projectManagerId'] : null;
        
        if ($action === 'readAll') {
            $projects = $this->projectModel->getAllProjects();
            echo json_encode($projects);
        } elseif ($action === 'readAllByProjectManagerID' && $projectManagerId) {
            $projects = $this->projectModel->getProjectsByManagerId($projectManagerId);
            echo json_encode($projects);
        } else {
            $projects = $this->projectModel->getAllProjects();
            echo json_encode($projects);
        }
    }
    
    private function handlePost() {
        $action = isset($_POST['action']) ? $_POST['action'] : '';
        
        if ($action === 'create') {
            $name = isset($_POST['name']) ? $_POST['name'] : '';
            $projectManagerId = isset($_POST['projectManagerId']) ? $_POST['projectManagerId'] : '';
            $description = isset($_POST['description']) ? $_POST['description'] : '';
            $members = isset($_POST['members']) ? $_POST['members'] : '';
            
            $success = $this->projectModel->createProject($name, $projectManagerId, $description, $members);
            echo json_encode(['success' => $success]);
            
        } elseif ($action === 'assignProject') {
            $projectName = isset($_POST['projectName']) ? $_POST['projectName'] : '';
            $projectManagerId = isset($_POST['projectManagerId']) ? $_POST['projectManagerId'] : '';
            
            $success = $this->projectModel->assignProject($projectName, $projectManagerId);
            if ($success) {
                $project = $this->projectModel->getProjectByName($projectName);
                echo json_encode($project);
            } else {
                http_response_code(500);
                echo json_encode(['error' => 'Failed to assign project']);
            }
        }
    }
    
    private function handlePut() {
        parse_str(file_get_contents("php://input"), $_PUT);
        
        $id = isset($_PUT['id']) ? $_PUT['id'] : '';
        $projectManagerId = isset($_PUT['projectManagerId']) ? $_PUT['projectManagerId'] : '';
        $name = isset($_PUT['name']) ? $_PUT['name'] : '';
        $description = isset($_PUT['description']) ? $_PUT['description'] : '';
        $members = isset($_PUT['members']) ? $_PUT['members'] : '';
        
        $success = $this->projectModel->updateProject($id, $projectManagerId, $name, $description, $members);
        echo json_encode(['success' => $success]);
    }
    
    private function handleDelete() {
        parse_str(file_get_contents("php://input"), $_DELETE);
        $id = isset($_DELETE['id']) ? $_DELETE['id'] : (isset($_GET['id']) ? $_GET['id'] : '');
        
        $success = $this->projectModel->deleteProject($id);
        echo json_encode(['success' => $success]);
    }
}

$api = new ProjectAPI();
$api->handleRequest();
?> 