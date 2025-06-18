<?php
session_start();
include_once 'config/database.php';
include_once 'models/Project.php';
include_once 'models/SoftwareDeveloper.php';
include_once 'includes/header.php';

if (!isset($_SESSION['currentUser'])) {
    header('Location: login.php');
    exit();
}

$currentUser = $_SESSION['currentUser'];
$database = new Database();
$db = $database->getConnection();
$project = new Project($db);
$softwareDeveloper = new SoftwareDeveloper($db);

$success_message = '';
$error_message = '';

if ($_POST) {
    if (isset($_POST['action'])) {
        switch ($_POST['action']) {
            case 'assign_project':
                $projectName = trim($_POST['project_name']);
                $projectManagerName = trim($_POST['project_manager_name']);
                
                if (!empty($projectName) && !empty($projectManagerName)) {
                    $found = $softwareDeveloper->findOne($projectManagerName);
                    if ($found) {
                        $stmt = $project->assignProject($projectName, $softwareDeveloper->id);
                        if ($stmt) {
                            $success_message = "Project '$projectName' assigned to '$projectManagerName' successfully!";
                        } else {
                            $error_message = "Failed to assign project.";
                        }
                    } else {
                        $error_message = "Developer '$projectManagerName' not found!";
                    }
                } else {
                    $error_message = "Please fill in all fields.";
                }
                break;
        }
    }
}

$currentUserID = null;
$found = $softwareDeveloper->findOne($currentUser);
if ($found) {
    $currentUserID = $softwareDeveloper->id;
}

$allProjects = [];
$stmt = $project->readAll();
while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
    $allProjects[] = $row;
}

$yourProjects = [];
if ($currentUserID) {
    $stmt = $project->readAllByProjectManagerID($currentUserID);
    while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
        $yourProjects[] = $row;
    }
}

$memberProjects = [];
foreach ($allProjects as $proj) {
    if ($proj['members'] && strpos($proj['members'], $currentUser) !== false) {
        $memberProjects[] = $proj;
    }
}

$allDevelopers = [];
$stmt = $softwareDeveloper->readAll();
while ($row = $stmt->fetch(PDO::FETCH_ASSOC)) {
    $allDevelopers[] = $row;
}
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Project Management</h1>
    <p class="text-gray-600 mb-4">Welcome, <?php echo htmlspecialchars($currentUser); ?>! 
        <a href="logout.php" class="text-blue-500 hover:underline">Logout</a>
    </p>
    
    <?php if (!empty($success_message)): ?>
        <div class="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-4">
            <?php echo htmlspecialchars($success_message); ?>
        </div>
    <?php endif; ?>
    
    <?php if (!empty($error_message)): ?>
        <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
            <?php echo htmlspecialchars($error_message); ?>
        </div>
    <?php endif; ?>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">
            
            <h2 class="text-xl font-bold text-neutral-800 mb-4">All Projects</h2>
            <div class="flex flex-col space-y-2 mb-6">
                <?php if (empty($allProjects)): ?>
                    <p class="text-gray-500 italic">No projects found.</p>
                <?php else: ?>
                    <?php foreach ($allProjects as $proj): ?>
                        <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                            <h3 class="font-semibold"><?php echo htmlspecialchars($proj['name']); ?></h3>
                            <p><?php echo htmlspecialchars($proj['description'] ?: 'No description'); ?></p>
                            <p>Members: <?php echo htmlspecialchars($proj['members'] ?: 'None'); ?></p>
                        </div>
                    <?php endforeach; ?>
                <?php endif; ?>
            </div>

            <!-- Your Managed Projects Section -->
            <h2 class="text-xl font-bold text-neutral-800 mb-4">Your Managed Projects</h2>
            <div class="flex flex-col space-y-2 mb-6">
                <?php if (empty($yourProjects)): ?>
                    <p class="text-gray-500 italic">You are not managing any projects.</p>
                <?php else: ?>
                    <?php foreach ($yourProjects as $proj): ?>
                        <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                            <h3 class="font-semibold"><?php echo htmlspecialchars($proj['name']); ?></h3>
                            <p><?php echo htmlspecialchars($proj['description'] ?: 'No description'); ?></p>
                            <p>Members: <?php echo htmlspecialchars($proj['members'] ?: 'None'); ?></p>
                        </div>
                    <?php endforeach; ?>
                <?php endif; ?>
            </div>

            <!-- Projects You're Member Of Section -->
            <h2 class="text-xl font-bold text-neutral-800 mb-4">Projects You're Member Of</h2>
            <div class="flex flex-col space-y-2 mb-6">
                <?php if (empty($memberProjects)): ?>
                    <p class="text-gray-500 italic">You are not a member of any projects.</p>
                <?php else: ?>
                    <?php foreach ($memberProjects as $proj): ?>
                        <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                            <h3 class="font-semibold"><?php echo htmlspecialchars($proj['name']); ?></h3>
                            <p><?php echo htmlspecialchars($proj['description'] ?: 'No description'); ?></p>
                            <p>Members: <?php echo htmlspecialchars($proj['members']); ?></p>
                        </div>
                    <?php endforeach; ?>
                <?php endif; ?>
            </div>

            <!-- Assign Project Section -->
            <h2 class="text-xl font-bold text-neutral-800 mb-4">Assign Project</h2>
            <form method="post" action="index.php" class="flex flex-col space-y-2 mb-6">
                <input type="hidden" name="action" value="assign_project">
                <input type="text" name="project_name" placeholder="Project Name" class="p-2 border border-gray-300 rounded" required>
                <input type="text" name="project_manager_name" placeholder="Project Manager Name" class="p-2 border border-gray-300 rounded" required>
                <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Add Project</button>
            </form>

            <!-- Developers Section -->
            <h2 class="text-xl font-bold text-neutral-800 mb-4">Developers</h2>
            <div class="flex flex-row gap-2 mb-4">
                <button id="show-all-devs" class="bg-blue-500 text-white px-4 py-2 rounded-md">Show All Developers</button>
                <input type="text" id="skill-filter" placeholder="Filter by skill (e.g., Java)" class="p-2 border border-gray-300 rounded">
                <button id="filter-devs" class="bg-green-500 text-white px-4 py-2 rounded-md">Filter</button>
            </div>

            <div id="all-devs-tabs" class="flex flex-col space-y-2 mb-4" style="display: none;">
                <?php foreach ($allDevelopers as $dev): ?>
                    <div class="developer-item p-2 border border-gray-300 rounded-md" 
                         data-name="<?php echo htmlspecialchars($dev['name']); ?>" 
                         data-skills="<?php echo htmlspecialchars($dev['skills']); ?>">
                        <?php echo htmlspecialchars($dev['name']) . ' - ' . htmlspecialchars($dev['skills']); ?>
                    </div>
                <?php endforeach; ?>
            </div>
        </div>
    </div>
</div>

<script>
const showAllDevelopers = () => {
    const devsContainer = document.getElementById('all-devs-tabs');
    const allDevItems = devsContainer.querySelectorAll('.developer-item');
    
    allDevItems.forEach(item => {
        item.style.display = 'block';
    });
    
    devsContainer.style.display = 'block';
}

const filterDevelopersBySkill = (skill) => {
    const devsContainer = document.getElementById('all-devs-tabs');
    const allDevItems = devsContainer.querySelectorAll('.developer-item');
    let visibleCount = 0;
    
    if (skill.trim() === '') {
        showAllDevelopers();
        return;
    }
    
    // Filter developers based on skill
    allDevItems.forEach(item => {
        const devSkills = item.getAttribute('data-skills').toLowerCase();
        if (devSkills.includes(skill.toLowerCase())) {
            item.style.display = 'block';
            visibleCount++;
        } else {
            item.style.display = 'none';
        }
    });
    
    // Show the container
    devsContainer.style.display = 'block';
    
    // Handle no results case
    if (visibleCount === 0) {
        // Create or update no results message
        let noResultsMsg = document.getElementById('no-devs-message');
        if (!noResultsMsg) {
            noResultsMsg = document.createElement('div');
            noResultsMsg.id = 'no-devs-message';
            noResultsMsg.className = 'p-2 text-gray-500 italic';
            devsContainer.appendChild(noResultsMsg);
        }
        noResultsMsg.textContent = 'No developers found with this skill';
        noResultsMsg.style.display = 'block';
    } else {
        // Hide no results message if it exists
        const noResultsMsg = document.getElementById('no-devs-message');
        if (noResultsMsg) {
            noResultsMsg.style.display = 'none';
        }
    }
}

// Event listeners
document.getElementById('show-all-devs').addEventListener('click', () => {
    showAllDevelopers();
    document.getElementById('skill-filter').value = '';
});

document.getElementById('filter-devs').addEventListener('click', () => {
    const skill = document.getElementById('skill-filter').value.trim();
    filterDevelopersBySkill(skill);
});

// Allow filtering on Enter key press
document.getElementById('skill-filter').addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
        const skill = e.target.value.trim();
        filterDevelopersBySkill(skill);
    }
});
</script>

<?php include_once 'includes/footer.php'; ?> 