<?php
class ProjectData {
    public $id;
    public $projectManagerID;
    public $name;
    public $description;
    public $members;

    public function __construct($id = 0, $projectManagerID = 0, $name = '', $description = '', $members = '') {
        $this->id = $id;
        $this->projectManagerID = $projectManagerID;
        $this->name = $name;
        $this->description = $description;
        $this->members = $members;
    }

    public function toArray() {
        return [
            'id' => $this->id,
            'projectManagerID' => $this->projectManagerID,
            'name' => $this->name,
            'description' => $this->description,
            'members' => $this->members
        ];
    }

    public static function fromArray($data) {
        return new ProjectData(
            isset($data['id']) ? $data['id'] : 0,
            isset($data['projectManagerID']) ? $data['projectManagerID'] : 0,
            isset($data['name']) ? $data['name'] : '',
            isset($data['description']) ? $data['description'] : '',
            isset($data['members']) ? $data['members'] : ''
        );
    }
}
?> 