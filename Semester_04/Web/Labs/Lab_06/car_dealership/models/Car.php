<?php
class Car {
    private $conn;
    private $table_name = "cars";

    public $id;
    public $model;
    public $engine_power;
    public $fuel_type;
    public $price;
    public $color;
    public $year;

    public $history;
    public $category_id;
    public $created_at;
    public $features;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll($category_id = null) {
        $query = "SELECT id, model, engine_power, fuel_type, price, color, year, history, category_id, created_at, features FROM " . $this->table_name;
        
        if ($category_id !== null) {
            $query .= " WHERE category_id = ?";
        }
        
        $query .= " ORDER BY created_at DESC";
        
        $stmt = $this->conn->prepare($query);
        
        if ($category_id !== null) {
            $stmt->bindParam(1, $category_id);
        }
        
        $stmt->execute();
        
        return $stmt;
    }

    public function readOne() {
        $query = "SELECT id, model, engine_power, fuel_type, price, color, year, history, category_id, created_at, features FROM " . $this->table_name . " WHERE id = ? LIMIT 0,1";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->id);
        
        $stmt->execute();
        
        $row = $stmt->fetch(PDO::FETCH_ASSOC);
        
        $this->model = $row['model'];
        $this->engine_power = $row['engine_power'];
        $this->fuel_type = $row['fuel_type'];
        $this->price = $row['price'];
        $this->color = $row['color'];
        $this->year = $row['year'];
        $this->history = $row['history'];
        $this->category_id = $row['category_id'];
        $this->created_at = $row['created_at'];
        $this->features = $row['features'];
    }

    public function create() {
        $query = "INSERT INTO " . $this->table_name . " SET model = ?, engine_power = ?, fuel_type = ?, price = ?, color = ?, year = ?, history = ?, category_id = ?, created_at = ?, features = ?";

        $stmt = $this->conn->prepare($query);
        
        $this->model = htmlspecialchars(strip_tags($this->model));
        $this->engine_power = htmlspecialchars(strip_tags($this->engine_power));
        $this->fuel_type = htmlspecialchars(strip_tags($this->fuel_type));
        $this->price = htmlspecialchars(strip_tags($this->price));
        $this->color = htmlspecialchars(strip_tags($this->color));
        $this->year = htmlspecialchars(strip_tags($this->year));
        $this->history = htmlspecialchars(strip_tags($this->history));
        $this->category_id = htmlspecialchars(strip_tags($this->category_id));
        $this->features = htmlspecialchars(strip_tags($this->features));
        $stmt->bindParam(1, $this->model);
        $stmt->bindParam(2, $this->engine_power);
        $stmt->bindParam(3, $this->fuel_type);
        $stmt->bindParam(4, $this->price);
        $stmt->bindParam(5, $this->color);
        $stmt->bindParam(6, $this->year);
        $stmt->bindParam(7, $this->history);
        $stmt->bindParam(8, $this->category_id);
        $stmt->bindParam(9, $this->created_at);
        $stmt->bindParam(10, $this->features);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }

    public function delete() {
        $query = "DELETE FROM " . $this->table_name . " WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->id);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }

    public function update() {
        $query = "UPDATE " . $this->table_name . " SET model = ?, engine_power = ?, fuel_type = ?, price = ?, color = ?, year = ?, history = ?, category_id = ?, features = ? WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->model);
        $stmt->bindParam(2, $this->engine_power);
        $stmt->bindParam(3, $this->fuel_type);
        $stmt->bindParam(4, $this->price);
        $stmt->bindParam(5, $this->color);
        $stmt->bindParam(6, $this->year);
        $stmt->bindParam(7, $this->history);
        $stmt->bindParam(8, $this->category_id);
        $stmt->bindParam(9, $this->features);
        $stmt->bindParam(10, $this->id);
        if($stmt->execute()) {
            return true;
        }
    }

}
?>