<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4"> Project Management</h1>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">
            <h2 class="text-xl font-bold text-neutral-800 mb-4">All Projects</h2>
            <div id="all-projects-tabs" class="flex flex-col space-y-2  mb-4 pb-1 whitespace-nowrap">
            </div>
            <h2 class="text-xl font-bold text-neutral-800 mb-4">Your Projects</h2>
            <div id="your-projects-tabs" class="flex flex-col space-y-2  mb-4 pb-1 whitespace-nowrap">
            </div>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Assign Project</h2>
            <form id="add-project-form" class="flex flex-col space-y-2  mb-4 pb-1 whitespace-nowrap">
                <input type="text" id="project-name" placeholder="Project Name">
                <input type="text" id="project-members" placeholder="Project Manager Name">
                <button type="submit" id="add-project-button">Add Project</button>
            </form>
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

    const assignProject = async (projectName, projectManagerName) => {
        const doesUserExist = await fetch(`api/SoftwareDeveloper/findOne.php?name=${encodeURIComponent(projectManagerName)}`, {
            method: 'GET',
        });
        const data = await doesUserExist.json();
        console.log(data);
        if (data.record) {
            const response = await fetch(`api/Project/Assign.php?projectName=${encodeURIComponent(projectName)}&projectManagerID=${data.record.id}`, {
                method: 'GET',
            });
            const result = await response.json();
            console.log('Assignment result:', result);
            
            if (result.record) {
                return result.record.id;
            } else {
                console.error('Assignment failed:', result.message);
                alert('Project assignment failed: ' + (result.message || 'Unknown error'));
                return null;
            }
        } else {
            return null;
        }
    }



    const getAllProjects = async () => {
        const response = await fetch(`api/Project/readAll.php`);
        const data = await response.json();
        data.records.forEach(project => {
            const projectDiv = document.createElement('div');
            projectDiv.classList.add('flex', 'flex-row', 'gap-2', 'items-center', 'justify-between', 'p-2', 'border', 'border-gray-300', 'rounded-md', 'divide-x-2');
            projectDiv.innerHTML = `
                <h3>${project.name}</h3>
                <p>${project.description}</p>
                <p>${project.members}</p>
            `;
            document.getElementById('all-projects-tabs').appendChild(projectDiv);
        })
        return data.records;
    }

    const getYourProjects = async (userID) => {
        const response = await fetch(`api/Project/readAll.php?projectManagerID=${userID}`);
        const data = await response.json();
        console.log(data.records);
        data.records.forEach(project => {
            const projectDiv = document.createElement('div');
            projectDiv.classList.add('flex', 'flex-row', 'gap-2', 'items-center', 'justify-between', 'p-2', 'border', 'border-gray-300', 'rounded-md', 'divide-x-2');
            projectDiv.innerHTML = `
                <h3>${project.name}</h3>
                <p>${project.description}</p>
                <p>${project.members}</p>
            `;
            document.getElementById('your-projects-tabs').appendChild(projectDiv);
        })
        return data.records;
    }
    document.addEventListener('DOMContentLoaded', async () => {
        const userID = await getUserID(localStorage.getItem('currentUser'));
        await getYourProjects(userID);
        await getAllProjects();
    });

    document.getElementById('add-project-form').addEventListener('submit', async (e) => {
        e.preventDefault();
        const projectName = document.getElementById('project-name').value;
        const projectManagerName = document.getElementById('project-members').value;
        
        if (projectName && projectManagerName) {
            await assignProject(projectName, projectManagerName);
            location.reload();
        }
    });
</script>

<?php include_once 'includes/footer.php'; ?> 