<?php
session_start();
require_once 'models/Post.php';
require_once 'models/Topic.php';

class MainController {
    private $postModel;
    private $topicModel;
    
    public function __construct() {
        $this->postModel = new Post();
        $this->topicModel = new Topic();
    }
    
    private function checkAuthentication() {
        if (!isset($_SESSION['currentUser'])) {
            header('Location: login.php');
            exit();
        }
    }
    
    private function isNewPost($currentUser) {
        $oldPosts = isset($_SESSION['allPosts']) ? $_SESSION['allPosts'] : [];
        $newPosts = $this->postModel->getAllPosts();
        $_SESSION['allPosts'] = $newPosts;
        
        // Get list of old post IDs for comparison
        $oldIds = array_column($oldPosts, 'id');
        
        // Check for new posts that are NOT from the current user
        foreach ($newPosts as $post) {
            if (!in_array($post['id'], $oldIds) && $post['user'] !== $currentUser) {
                return true;
            }
        }
        return false;
    }
    
    public function showMain() {
        $this->checkAuthentication();
        
        $currentUser = $_SESSION['currentUser'];
        
        try {
            if ($this->isNewPost($currentUser)) {
                $_SESSION['success_message'] = 'New posts available!';
            }
        } catch (Exception $e) {
            $_SESSION['error_message'] = 'Database error: ' . $e->getMessage();
        }
        
        include 'views/main.php';
    }
    
    public function handlePost() {
        $this->checkAuthentication();
        
        $currentUser = $_SESSION['currentUser'];
        $action = isset($_POST['action']) ? $_POST['action'] : '';
        $successMessage = '';
        $errorMessage = '';
        
        try {
            if ($action === 'add_post') {
                $postText = isset($_POST['post_text']) ? trim($_POST['post_text']) : '';
                $topicText = isset($_POST['topic_text']) ? trim($_POST['topic_text']) : '';
                
                if (empty($postText) || empty($topicText)) {
                    $errorMessage = 'Post and topic cannot be empty';
                } else {
                    // Check if topic exists
                    $existingTopic = $this->topicModel->getTopicByName($topicText);
                    
                    if (!$existingTopic) {
                        // Create new topic
                        $topicId = $this->topicModel->addTopic($topicText);
                    } else {
                        $topicId = $existingTopic['id'];
                    }
                    
                    if ($topicId) {
                        $date = time(); // Current timestamp
                        if ($this->postModel->addPost($currentUser, $topicId, $postText, $date)) {
                            $successMessage = 'Post added successfully!';
                        } else {
                            $errorMessage = 'Failed to add post';
                        }
                    } else {
                        $errorMessage = 'Failed to create topic';
                    }
                }
                
            } elseif ($action === 'update_post') {
                $postId = isset($_POST['post_id']) ? trim($_POST['post_id']) : '';
                $postText = isset($_POST['post_text']) ? trim($_POST['post_text']) : '';
                $topicText = isset($_POST['topic_text']) ? trim($_POST['topic_text']) : '';
                
                if (empty($postId) || empty($postText) || empty($topicText)) {
                    $errorMessage = 'Post ID, post text, and topic text cannot be empty';
                } else {
                    // Validate that postId is a valid integer
                    if (!is_numeric($postId)) {
                        $errorMessage = 'Post ID must be a valid number';
                    } else {
                        // Check if topic exists
                        $existingTopic = $this->topicModel->getTopicByName($topicText);
                        
                        if (!$existingTopic) {
                            $errorMessage = 'Topic does not exist';
                        } else {
                            // Check if post exists
                            $existingPost = $this->postModel->getPostById($postId);
                            
                            if (!$existingPost) {
                                $errorMessage = 'Post does not exist';
                            } else {
                                if ($this->postModel->updatePost($postId, $existingTopic['id'], $postText)) {
                                    $successMessage = 'Post updated successfully!';
                                } else {
                                    $errorMessage = 'Failed to update post';
                                }
                            }
                        }
                    }
                }
            }
        } catch (Exception $e) {
            $errorMessage = 'Database error: ' . $e->getMessage();
        }
        
        if (!empty($successMessage)) {
            $_SESSION['success_message'] = $successMessage;
        }
        if (!empty($errorMessage)) {
            $_SESSION['error_message'] = $errorMessage;
        }
        
        header('Location: main.php');
        exit();
    }
}

// Handle the request
$controller = new MainController();

if ($_SERVER['REQUEST_METHOD'] === 'GET') {
    $controller->showMain();
} elseif ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $controller->handlePost();
}
?> 