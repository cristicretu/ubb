<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ page import="java.util.List" %>
<%@ page import="dto.ProjectData" %>
<%@ page import="dto.DeveloperData" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Project Management</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-gray-100 min-h-screen">

<div class="container mx-auto px-4 py-8">
    <div class="mb-6">
        <h1 class="text-3xl font-bold text-neutral-800 mb-4">Project Management</h1>
        <p class="text-gray-600 mb-4">
            Welcome, <strong><%= request.getAttribute("currentUser") %></strong>! 
            <a href="logout" class="text-blue-500 hover:underline">Logout</a>
        </p>
        
        <%
            String successMessage = (String) session.getAttribute("success_message");
            String errorMessage = (String) session.getAttribute("error_message");
            
            if (successMessage != null) {
                session.removeAttribute("success_message");
            }
            if (errorMessage != null) {
                session.removeAttribute("error_message");
            }
        %>
        
        <% if (successMessage != null && !successMessage.isEmpty()) { %>
            <div class="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-4">
                <%= successMessage %>
            </div>
        <% } %>
        
        <% if (errorMessage != null && !errorMessage.isEmpty()) { %>
            <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
                <%= errorMessage %>
            </div>
        <% } %>
        
        <div class="bg-white p-4 rounded shadow-md mb-6">
            <div class="overflow-x-auto">
                
                <!-- All Projects Section -->
                <h2 class="text-xl font-bold text-neutral-800 mb-4">All Projects</h2>
                <div class="flex flex-col space-y-2 mb-6">
                    <%
                        @SuppressWarnings("unchecked")
                        List<ProjectData> allProjects = (List<ProjectData>) request.getAttribute("allProjects");
                        if (allProjects == null || allProjects.isEmpty()) {
                    %>
                        <p class="text-gray-500 italic">No projects found.</p>
                    <% } else { %>
                        <% for (ProjectData proj : allProjects) { %>
                            <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                                <h3 class="font-semibold"><%= proj.name != null ? proj.name : "" %></h3>
                                <p><%= proj.description != null && !proj.description.isEmpty() ? proj.description : "No description" %></p>
                                <p>Members: <%= proj.members != null && !proj.members.isEmpty() ? proj.members : "None" %></p>
                            </div>
                        <% } %>
                    <% } %>
                </div>

                <!-- Your Managed Projects Section -->
                <h2 class="text-xl font-bold text-neutral-800 mb-4">Your Managed Projects</h2>
                <div class="flex flex-col space-y-2 mb-6">
                    <%
                        @SuppressWarnings("unchecked")
                        List<ProjectData> yourProjects = (List<ProjectData>) request.getAttribute("yourProjects");
                        if (yourProjects == null || yourProjects.isEmpty()) {
                    %>
                        <p class="text-gray-500 italic">You are not managing any projects.</p>
                    <% } else { %>
                        <% for (ProjectData proj : yourProjects) { %>
                            <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                                <h3 class="font-semibold"><%= proj.name != null ? proj.name : "" %></h3>
                                <p><%= proj.description != null && !proj.description.isEmpty() ? proj.description : "No description" %></p>
                                <p>Members: <%= proj.members != null && !proj.members.isEmpty() ? proj.members : "None" %></p>
                            </div>
                        <% } %>
                    <% } %>
                </div>

                <!-- Projects You're Member Of Section -->
                <h2 class="text-xl font-bold text-neutral-800 mb-4">Projects You're Member Of</h2>
                <div class="flex flex-col space-y-2 mb-6">
                    <%
                        @SuppressWarnings("unchecked")
                        List<ProjectData> memberProjects = (List<ProjectData>) request.getAttribute("memberProjects");
                        if (memberProjects == null || memberProjects.isEmpty()) {
                    %>
                        <p class="text-gray-500 italic">You are not a member of any projects.</p>
                    <% } else { %>
                        <% for (ProjectData proj : memberProjects) { %>
                            <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                                <h3 class="font-semibold"><%= proj.name != null ? proj.name : "" %></h3>
                                <p><%= proj.description != null && !proj.description.isEmpty() ? proj.description : "No description" %></p>
                                <p>Members: <%= proj.members != null ? proj.members : "" %></p>
                            </div>
                        <% } %>
                    <% } %>
                </div>

                <!-- Assign Project Section -->
                <h2 class="text-xl font-bold text-neutral-800 mb-4">Assign Project</h2>
                <form method="post" action="main" class="flex flex-col space-y-2 mb-6">
                    <input type="hidden" name="action" value="assign_project">
                    <input type="text" name="project_name" placeholder="Project Name" 
                           class="p-2 border border-gray-300 rounded" required>
                    <input type="text" name="project_manager_name" placeholder="Project Manager Name" 
                           class="p-2 border border-gray-300 rounded" required>
                    <button type="submit" class="bg-blue-500 text-white px-4 py-2 rounded-md">Add Project</button>
                </form>

                <!-- Developers Section -->
                <h2 class="text-xl font-bold text-neutral-800 mb-4">Developers</h2>
                <div class="flex flex-row gap-2 mb-4">
                    <button id="show-all-devs" class="bg-blue-500 text-white px-4 py-2 rounded-md">Show All Developers</button>
                    <input type="text" id="skill-filter" placeholder="Filter by skill (e.g., Java)" 
                           class="p-2 border border-gray-300 rounded">
                    <button id="filter-devs" class="bg-green-500 text-white px-4 py-2 rounded-md">Filter</button>
                </div>

                <div id="all-devs-tabs" class="flex flex-col space-y-2 mb-4" style="display: none;">
                    <%
                        @SuppressWarnings("unchecked")
                        List<DeveloperData> allDevelopers = (List<DeveloperData>) request.getAttribute("allDevelopers");
                        if (allDevelopers != null) {
                            for (DeveloperData dev : allDevelopers) {
                    %>
                        <div class="developer-item p-2 border border-gray-300 rounded-md" 
                             data-name="<%= dev.name != null ? dev.name : "" %>" 
                             data-skills="<%= dev.skills != null ? dev.skills : "" %>">
                            <%= dev.name != null ? dev.name : "" %> - <%= dev.skills != null ? dev.skills : "" %>
                        </div>
                    <% 
                            }
                        }
                    %>
                </div>
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
    
    // Hide no results message if it exists
    const noResultsMsg = document.getElementById('no-devs-message');
    if (noResultsMsg) {
        noResultsMsg.style.display = 'none';
    }
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

</body>
</html> 