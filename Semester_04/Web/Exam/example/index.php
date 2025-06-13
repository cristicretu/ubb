<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4"> Project Management</h1>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">
            <div id="projects-tabs" class="flex space-x-2  mb-4 pb-1 whitespace-nowrap">
            </div>
        </div>
        
        <div id="category-content" class="mt-4">
        </div>

        <div id="cars-container" class="mt-4">
        </div>
    </div>
</div>

<script>
    const getUserID = async (name) => {
        const response = await fetch(`api/SoftwareDeveloper/findOne.php?name=${encodeURIComponent(name)}`, {
            method: 'GET',
        });
        const data = await response.json();
        return data.record.id;
    }

    const getProjects = async (userID) => {
        const response = await fetch(`api/Project/readAll.php?projectManagerID=${userID}`);
        const data = await response.json();
        console.log(data);
        return data;
    }
    document.addEventListener('DOMContentLoaded', async () => {
        const userID = await getUserID(localStorage.getItem('currentUser'));
        const projects = await getProjects(userID);
        console.log(projects);
    });
</script>

<?php include_once 'includes/footer.php'; ?> 