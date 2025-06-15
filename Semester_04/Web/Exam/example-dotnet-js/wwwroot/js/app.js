class ProjectManagementApp {
  constructor() {
    // Try to auto-detect API server URL or fallback to localhost:5000
    this.baseUrl = "http://localhost:3000";
    this.currentUser = null;
    this.developers = [];
    this.init();
  }

  init() {
    this.bindEvents();
    this.checkAuthStatus();
  }

  bindEvents() {
    // Login form
    document.getElementById("login-form").addEventListener("submit", (e) => {
      e.preventDefault();
      this.login();
    });

    // Logout button
    document.getElementById("logout-btn").addEventListener("click", () => {
      this.logout();
    });

    // Assign project form
    document
      .getElementById("assign-project-form")
      .addEventListener("submit", (e) => {
        e.preventDefault();
        this.assignProject();
      });

    // Developer filtering
    document.getElementById("show-all-devs").addEventListener("click", () => {
      this.showAllDevelopers();
    });

    document.getElementById("filter-devs").addEventListener("click", () => {
      this.filterDevelopers();
    });

    document
      .getElementById("skill-filter")
      .addEventListener("keypress", (e) => {
        if (e.key === "Enter") {
          this.filterDevelopers();
        }
      });
  }

  async checkAuthStatus() {
    console.log(`Checking auth status at: ${this.baseUrl}/api/auth/status`);
    try {
      const response = await fetch(`${this.baseUrl}/api/auth/status`, {
        credentials: "include",
      });
      console.log("Auth response:", response.status, response.statusText);
      const data = await response.json();
      console.log("Auth response data:", data);

      if (data.isAuthenticated) {
        this.currentUser = { username: data.username, userId: data.userId };
        this.showDashboard();
        this.loadProjects();
      } else {
        console.log("Not authenticated:", data.debug || "No debug info");
        this.showLogin();
      }
    } catch (error) {
      console.error("Auth check failed:", error);
      this.showLogin();
    }
  }

  async login() {
    const username = document.getElementById("username").value.trim();
    const loading = document.getElementById("login-loading");
    const errorDiv = document.getElementById("login-error");

    if (!username) {
      this.showError("Please enter a username", "login-error");
      return;
    }

    loading.classList.remove("hidden");
    errorDiv.classList.add("hidden");

    console.log(`Attempting login to: ${this.baseUrl}/api/auth/login`);

    try {
      const response = await fetch(`${this.baseUrl}/api/auth/login`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include",
        body: JSON.stringify({ username }),
      });

      console.log("Login response status:", response.status);
      console.log("Login response headers:", [...response.headers.entries()]);

      const data = await response.json();
      console.log("Login response data:", data);

      if (response.ok) {
        this.currentUser = { username: data.username, userId: data.userId };
        console.log("Login successful, checking auth status...");

        // Immediately check auth status to verify session
        await this.checkAuthStatus();
      } else {
        this.showError(data.message || "Login failed", "login-error");
      }
    } catch (error) {
      console.error("Login error:", error);
      this.showError("Network error. Please try again.", "login-error");
    } finally {
      loading.classList.add("hidden");
    }
  }

  async logout() {
    try {
      await fetch(`${this.baseUrl}/api/auth/logout`, {
        method: "POST",
        credentials: "include",
      });

      this.currentUser = null;
      this.showLogin();
      document.getElementById("username").value = "";
    } catch (error) {
      console.error("Logout error:", error);
    }
  }

  async loadProjects() {
    this.showLoading([
      "all-projects-loading",
      "your-projects-loading",
      "member-projects-loading",
    ]);

    try {
      const response = await fetch(`${this.baseUrl}/api/projects`, {
        credentials: "include",
      });

      if (response.ok) {
        const data = await response.json();
        this.renderProjects(data);
      } else if (response.status === 401) {
        this.showLogin();
      } else {
        this.showError("Failed to load projects");
      }
    } catch (error) {
      console.error("Load projects error:", error);
      this.showError("Network error while loading projects");
    } finally {
      this.hideLoading([
        "all-projects-loading",
        "your-projects-loading",
        "member-projects-loading",
      ]);
    }
  }

  async assignProject() {
    const projectName = document.getElementById("project-name").value.trim();
    const projectManagerName = document
      .getElementById("project-manager-name")
      .value.trim();
    const loading = document.getElementById("assign-loading");

    if (!projectName || !projectManagerName) {
      this.showError("Please fill in all fields");
      return;
    }

    loading.classList.remove("hidden");

    try {
      const response = await fetch(`${this.baseUrl}/api/projects`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include",
        body: JSON.stringify({
          projectName,
          projectManagerName,
        }),
      });

      const data = await response.json();

      if (response.ok) {
        this.showSuccess(data.message);
        document.getElementById("project-name").value = "";
        document.getElementById("project-manager-name").value = "";
        this.loadProjects(); // Refresh projects
      } else {
        this.showError(data.message || "Failed to assign project");
      }
    } catch (error) {
      console.error("Assign project error:", error);
      this.showError("Network error while assigning project");
    } finally {
      loading.classList.add("hidden");
    }
  }

  async loadDevelopers(skill = null) {
    const loading = document.getElementById("devs-loading");
    loading.style.display = "block";

    try {
      const url = skill
        ? `${this.baseUrl}/api/developers?skill=${encodeURIComponent(skill)}`
        : `${this.baseUrl}/api/developers`;

      const response = await fetch(url, {
        credentials: "include",
      });

      if (response.ok) {
        const data = await response.json();
        this.developers = data.developers;
        this.renderDevelopers();
      } else {
        this.showError("Failed to load developers");
      }
    } catch (error) {
      console.error("Load developers error:", error);
      this.showError("Network error while loading developers");
    } finally {
      loading.style.display = "none";
    }
  }

  showAllDevelopers() {
    document.getElementById("skill-filter").value = "";
    this.loadDevelopers();
    document.getElementById("all-devs-tabs").style.display = "block";
  }

  filterDevelopers() {
    const skill = document.getElementById("skill-filter").value.trim();
    this.loadDevelopers(skill);
    document.getElementById("all-devs-tabs").style.display = "block";
  }

  renderProjects(data) {
    // Render all projects
    const allProjectsDiv = document.getElementById("all-projects");
    allProjectsDiv.innerHTML = this.renderProjectList(
      data.allProjects,
      "No projects found."
    );

    // Render your projects
    const yourProjectsDiv = document.getElementById("your-projects");
    yourProjectsDiv.innerHTML = this.renderProjectList(
      data.yourProjects,
      "You are not managing any projects."
    );

    // Render member projects
    const memberProjectsDiv = document.getElementById("member-projects");
    memberProjectsDiv.innerHTML = this.renderProjectList(
      data.memberProjects,
      "You are not a member of any projects."
    );
  }

  renderProjectList(projects, emptyMessage) {
    if (!projects || projects.length === 0) {
      return `<p class="text-gray-500 italic">${emptyMessage}</p>`;
    }

    return projects
      .map(
        (project) => `
            <div class="flex flex-row gap-2 items-center justify-between p-2 border border-gray-300 rounded-md">
                <h3 class="font-semibold">${this.escapeHtml(
                  project.name || "Unnamed Project"
                )}</h3>
                <p>${this.escapeHtml(
                  project.description || "No description"
                )}</p>
                <p>Members: ${this.escapeHtml(project.members || "None")}</p>
            </div>
        `
      )
      .join("");
  }

  renderDevelopers() {
    const devsContainer = document.getElementById("all-devs-tabs");
    const loadingDiv = document.getElementById("devs-loading");

    // Remove existing developer items but keep loading div
    const existingItems = devsContainer.querySelectorAll(".developer-item");
    existingItems.forEach((item) => item.remove());

    // Remove existing no results message
    const existingNoResults = document.getElementById("no-devs-message");
    if (existingNoResults) {
      existingNoResults.remove();
    }

    if (this.developers.length === 0) {
      const noResultsMsg = document.createElement("div");
      noResultsMsg.id = "no-devs-message";
      noResultsMsg.className = "p-2 text-gray-500 italic";
      noResultsMsg.textContent = "No developers found with this skill";
      devsContainer.appendChild(noResultsMsg);
    } else {
      this.developers.forEach((dev) => {
        const devDiv = document.createElement("div");
        devDiv.className =
          "developer-item p-2 border border-gray-300 rounded-md";
        devDiv.textContent = `${dev.name || "Unknown"} - ${
          dev.skills || "No skills listed"
        }`;
        devsContainer.appendChild(devDiv);
      });
    }
  }

  showLogin() {
    document.getElementById("login-page").classList.remove("hidden");
    document.getElementById("dashboard-page").classList.add("hidden");
    this.hideMessages();
  }

  showDashboard() {
    document.getElementById("login-page").classList.add("hidden");
    document.getElementById("dashboard-page").classList.remove("hidden");
    document.getElementById("welcome-username").textContent =
      this.currentUser.username;
    this.hideMessages();
  }

  showSuccess(message) {
    const successDiv = document.getElementById("success-message");
    successDiv.textContent = message;
    successDiv.classList.remove("hidden");
    this.hideError();
    setTimeout(() => this.hideMessages(), 5000);
  }

  showError(message, elementId = "error-message") {
    const errorDiv = document.getElementById(elementId);
    errorDiv.textContent = message;
    errorDiv.classList.remove("hidden");
    if (elementId === "error-message") {
      this.hideSuccess();
      setTimeout(() => this.hideMessages(), 5000);
    }
  }

  hideMessages() {
    document.getElementById("success-message").classList.add("hidden");
    document.getElementById("error-message").classList.add("hidden");
    document.getElementById("login-error").classList.add("hidden");
  }

  hideSuccess() {
    document.getElementById("success-message").classList.add("hidden");
  }

  hideError() {
    document.getElementById("error-message").classList.add("hidden");
  }

  showLoading(elementIds) {
    elementIds.forEach((id) => {
      const element = document.getElementById(id);
      if (element) element.style.display = "block";
    });
  }

  hideLoading(elementIds) {
    elementIds.forEach((id) => {
      const element = document.getElementById(id);
      if (element) element.style.display = "none";
    });
  }

  escapeHtml(text) {
    const map = {
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      '"': "&quot;",
      "'": "&#039;",
    };
    return text.replace(/[&<>"']/g, (m) => map[m]);
  }
}

// Initialize the app when DOM is loaded
document.addEventListener("DOMContentLoaded", () => {
  new ProjectManagementApp();
});
