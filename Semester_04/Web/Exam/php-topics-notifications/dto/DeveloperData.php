<?php
class DeveloperData {
    public $id;
    public $name;
    public $age;
    public $skills;

    public function __construct($id = 0, $name = '', $age = 0, $skills = '') {
        $this->id = $id;
        $this->name = $name;
        $this->age = $age;
        $this->skills = $skills;
    }

    public function toArray() {
        return [
            'id' => $this->id,
            'name' => $this->name,
            'age' => $this->age,
            'skills' => $this->skills
        ];
    }

    public static function fromArray($data) {
        return new DeveloperData(
            isset($data['id']) ? $data['id'] : 0,
            isset($data['name']) ? $data['name'] : '',
            isset($data['age']) ? $data['age'] : 0,
            isset($data['skills']) ? $data['skills'] : ''
        );
    }
}
?> 