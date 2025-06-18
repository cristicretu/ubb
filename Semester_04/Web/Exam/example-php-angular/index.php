<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4"> Project Management</h1>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">
            <h2 class="text-xl font-bold text-neutral-800 mb-4">All Projects</h2>
            <div id="all-projects-tabs" class="flex flex-col space-y-2 mb-4 pb-1 whitespace-nowrap">
            </div>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Your Managed Projects</h2>
            <div id="your-projects-tabs" class="flex flex-col space-y-2 mb-4 pb-1 whitespace-nowrap">
            </div>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Projects You're Member Of</h2>
            <div id="member-projects-tabs" class="flex flex-col space-y-2 mb-4 pb-1 whitespace-nowrap">
            </div>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Assign Project</h2>
            <form id="add-project-form" class="flex flex-col space-y-2 mb-4 pb-1 whitespace-nowrap">
                <input type="text" id="project-name" placeholder="Project Name" class="p-2 border border-gray-300 rounded">
                <input type="text" id="project-members" placeholder="Project Manager Name" class="p-2 border border-gray-300 rounded">
                <button type="submit" id="add-project-button" class="bg-blue-500 text-white px-4 py-2 rounded-md">Add Project</button>
            </form>

            <h2 class="text-xl font-bold text-neutral-800 mb-4">Developers</h2>
            <div class="flex flex-row gap-2 mb-4">
                <button id="show-all-devs" class="bg-blue-500 text-white px-4 py-2 rounded-md">Show All Developers</button>
                <input type="text" id="skill-filter" placeholder="Filter by skill (e.g., Java)" class="p-2 border border-gray-300 rounded">
                <button id="filter-devs" class="bg-green-500 text-white px-4 py-2 rounded-md">Filter</button>
            </div>

            <div id="all-devs-tabs" class="flex flex-col space-y-2 mb-4 pb-1 whitespace-nowrap">
            </div>
        </div>
    </div>
</div>

<script>
    const getUserID = async (name) => {
        const response = await fetch(`api/SoftwareDeveloper/findOne.php?name=${encodeURIComponent(name)}`, {
            method: 'GET',
        });
        const data = await response.json();
        return data.record?.id;
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
            alert('Developer not found!');
            return null;
        }
    }

    const getAllDevelopers = async () => {
        document.getElementById('all-devs-tabs').innerHTML = '';
        const response = await fetch(`api/SoftwareDeveloper/readAll.php`);
        const data = await response.json();
        data.records.forEach(dev => {
            const div = document.createElement('div');
            div.className = 'p-2 border border-gray-300 rounded-md';
            div.textContent = dev.name + ' - ' + dev.skills;
            document.getElementById('all-devs-tabs').appendChild(div);
        });
    }

    const filterDevelopersBySkill = async (skill) => {
        document.getElementById('all-devs-tabs').innerHTML = '';
        const response = await fetch(`api/SoftwareDeveloper/readAll.php`);
        const data = await response.json();
        const filteredDevs = data.records.filter(dev => 
            dev.skills.toLowerCase().includes(skill.toLowerCase())
        );
        
        if (filteredDevs.length === 0) {
            const div = document.createElement('div');
            div.textContent = 'No developers found with this skill';
            div.className = 'p-2 text-gray-500 italic';
            document.getElementById('all-devs-tabs').appendChild(div);
        } else {
            filteredDevs.forEach(dev => {
                const div = document.createElement('div');
                div.className = 'p-2 border border-gray-300 rounded-md';
                div.textContent = dev.name + ' - ' + dev.skills;
                document.getElementById('all-devs-tabs').appendChild(div);
            });
        }
    }

    document.getElementById('show-all-devs').addEventListener('click', async () => {
        await getAllDevelopers();
    });

    document.getElementById('filter-devs').addEventListener('click', async () => {
        const skill = document.getElementById('skill-filter').value.trim();
        if (skill) {
            await filterDevelopersBySkill(skill);
        } else {
            await getAllDevelopers();
        }
    });

    const getAllProjects = async () => {
        const response = await fetch(`api/Project/readAll.php`);
        const data = await response.json();
        data.records.forEach(project => {
            const projectDiv = document.createElement('div');
            projectDiv.classList.add('flex', 'flex-row', 'gap-2', 'items-center', 'justify-between', 'p-2', 'border', 'border-gray-300', 'rounded-md', 'divide-x-2');
            projectDiv.innerHTML = `
                <h3>${project.name}</h3>
                <p>${project.description || 'No description'}</p>
                <p>Members: ${project.members || 'None'}</p>
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
                <p>${project.description || 'No description'}</p>
                <p>Members: ${project.members || 'None'}</p>
            `;
            document.getElementById('your-projects-tabs').appendChild(projectDiv);
        })
        return data.records;
    }

    const getMemberProjects = async (userName) => {
        const response = await fetch(`api/Project/readAll.php`);
        const data = await response.json();
        const memberProjects = data.records.filter(project => 
            project.members && project.members.includes(userName)
        );
        
        if (memberProjects.length === 0) {
            const div = document.createElement('div');
            div.textContent = 'You are not a member of any projects';
            div.className = 'p-2 text-gray-500 italic';
            document.getElementById('member-projects-tabs').appendChild(div);
        } else {
            memberProjects.forEach(project => {
                const projectDiv = document.createElement('div');
                projectDiv.classList.add('flex', 'flex-row', 'gap-2', 'items-center', 'justify-between', 'p-2', 'border', 'border-gray-300', 'rounded-md', 'divide-x-2');
                projectDiv.innerHTML = `
                    <h3>${project.name}</h3>
                    <p>${project.description || 'No description'}</p>
                    <p>Members: ${project.members}</p>
                `;
                document.getElementById('member-projects-tabs').appendChild(projectDiv);
            });
        }
    }

    document.addEventListener('DOMContentLoaded', async () => {
        const userName = localStorage.getItem('currentUser');
        const userID = await getUserID(userName);
        
        if (userID) {
            await getYourProjects(userID);
        }
        await getMemberProjects(userName);
        await getAllProjects();
    });

    document.getElementById('add-project-form').addEventListener('submit', async (e) => {
        e.preventDefault();
        const projectName = document.getElementById('project-name').value;
        const projectManagerName = document.getElementById('project-members').value;
        
        if (projectName && projectManagerName) {
            const result = await assignProject(projectName, projectManagerName);
            if (result) {
                location.reload();
            }
        }
    });
</script>

<?php include_once 'includes/footer.php'; ?> 