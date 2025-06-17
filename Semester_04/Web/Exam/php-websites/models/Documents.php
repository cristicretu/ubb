<?php
class Documents {
    private $conn;
    private $table_name = "Documents";

    public $id;
    public $idWebsite;
    public $name;
    public $keyword1;
    public $keyword2;
    public $keyword3;
    public $keyword4;
    public $keyword5;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, idWebsite, name, keyword1, keyword2, keyword3, keyword4, keyword5 FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function readAllByWebsiteID($websiteID) {
        if ($websiteID == null) {
            return $this->readAll();
        }
        
        $query = "SELECT id, idWebsite, name, keyword1, keyword2, keyword3, keyword4, keyword5 FROM " . $this->table_name . " WHERE idWebsite = ?";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $websiteID);

        $stmt->execute();

        return $stmt;
    }

    public function searchByKeywords($keywords) {
        // Remove empty keywords and trim whitespace
        $keywords = array_filter(array_map('trim', $keywords), function($k) { return $k !== ''; });
        if (empty($keywords)) {
            // Return empty result set if no keywords provided
            $query = "SELECT * FROM " . $this->table_name . " WHERE 1=0";
            return $this->conn->query($query);
        }

        $conditions = [];
        $params = [];
        foreach ($keywords as $keyword) {
            // For each keyword, check all 5 keyword columns
            $conditions[] = "(keyword1 = ? OR keyword2 = ? OR keyword3 = ? OR keyword4 = ? OR keyword5 = ?)";
            // Add the same keyword 5 times for the 5 columns
            for ($i = 0; $i < 5; $i++) {
                $params[] = $keyword;
            }
        }
        $whereClause = implode(' OR ', $conditions);
        $query = "SELECT * FROM " . $this->table_name . " WHERE " . $whereClause;
        $stmt = $this->conn->prepare($query);

        // Bind all parameters
        foreach ($params as $i => $param) {
            $stmt->bindValue($i + 1, $param);
        }

        $stmt->execute();
        return $stmt;
    }

    public function update(
      $idWebsite,
      $keyword1,
      $keyword2,
      $keyword3,
      $keyword4,
      $keyword5
    ) {
        $query = "UPDATE " . $this->table_name . " SET id = ?, keyword1 = ?, keyword2 = ?, keyword3 = ?, keyword4 = ?, keyword5 = ? WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $idWebsite);
        $stmt->bindParam(2, $keyword1);
        $stmt->bindParam(3, $keyword2);
        $stmt->bindParam(4, $keyword3);
        $stmt->bindParam(5, $keyword4);
        $stmt->bindParam(6, $keyword5);
        $stmt->bindParam(7, $idWebsite);
        if($stmt->execute()) {
            return true;
        }

        return false;
    }

}
?>