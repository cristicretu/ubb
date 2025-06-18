<?php
include_once 'Reservations.php';

class HotelRoom {
    
    private $conn;
    private $table_name = "HotelRoom";


    public $id;
    public $roomNumber;
    public $capacity;
    public $basePrice;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, roomNumber, capacity, basePrice FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function create($roomNumber, $capacity, $basePrice) {
        $query = "INSERT INTO " . $this->table_name . " (roomNumber, capacity, basePrice) VALUES (?, ?, ?)";

        $stmt = $this->conn->prepare($query);
        
        $cleanRoomNumber = htmlspecialchars(strip_tags($roomNumber));
        $cleanCapacity = htmlspecialchars(strip_tags($capacity));
        $cleanBasePrice = htmlspecialchars(strip_tags($basePrice));
        
        $stmt->bindParam(1, $cleanRoomNumber);
        $stmt->bindParam(2, $cleanCapacity);
        $stmt->bindParam(3, $cleanBasePrice);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }


    public function readAvailableRooms($start_date, $end_date) {
        // first get all rooms
        // then for each of these rooms, find all possible reservations.
        // then check for all possible overlap cases
        $allRooms = $this->readAll();
        $availableRooms = [];
        $reservation = new Reservation($this->conn);

        while ($row = $allRooms->fetch()) {
            $roomId = $row['id'];
            $roomNumber = $row['roomNumber'];
            $capacity = $row['capacity'];
            $basePrice = $row['basePrice'];

            $reservations = $reservation->readAllByRoomId($roomId);

            $available = true;
            while ($reservationRow = $reservations->fetch()) {
                $checkInDate = $reservationRow['checkInDate'];
                $checkOutDate = $reservationRow['checkOutDate'];

                if ($start_date < $checkOutDate && $checkInDate < $end_date) {
                    $available = false;
                    break;
                }
            }

            if ($available) {
                $availableRooms[] = $row;
            }
        }

        return $availableRooms;
    }

    public function getBasePrice($roomId) {
        $query = "SELECT basePrice FROM " . $this->table_name . " WHERE id = ?";
        $stmt = $this->conn->prepare($query);
        $stmt->bindParam(1, $roomId);
        $stmt->execute();
        return $stmt->fetch()['basePrice'];
    }

    public function bookRoom($userId, $room_id, $start_date, $end_date) {
        $reservation = new Reservation($this->conn);
        $allRoomsStmt = $this->readAll();
        $allRooms = $allRoomsStmt->fetchAll();
        $base_price = $this->getBasePrice($room_id);

        $isReserved = $reservation->isReservedUserId($room_id, $start_date, $end_date, $userId);
        if ($isReserved) {
            return false;
        }

        $numberOfRooms = count($allRooms);
        $bookedRooms = 0;

        foreach ($allRooms as $room) {
            $roomId = $room['id'];
            $isReserved = $reservation->isReserved($roomId, $start_date, $end_date);
            
            if ($isReserved) {
                $bookedRooms = $bookedRooms + 1;
            }
        }

        if ($bookedRooms <= $numberOfRooms * 0.5) {
            $base_price = $base_price * 1;
        } else if ($bookedRooms > $numberOfRooms * 0.5 && $bookedRooms <= $numberOfRooms * 0.8) {
            $base_price = $base_price + (20/100) * $base_price;
        } else if ($bookedRooms > $numberOfRooms * 0.8) {
            $base_price = $base_price + (50/100) * $base_price;
        }

        $reservation->create($userId, $room_id, $start_date, $end_date, $base_price);



    }


}
?>