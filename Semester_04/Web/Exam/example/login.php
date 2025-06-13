<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4"> Login</h1>
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <form id="login-form" action="index.php" method="post">
            <input type="text" name="username" placeholder="Username">
            <button type="submit" id="login-button">Login</button>
        </form>
    </div>
</div>

<script>
const saveName = (name) => {
    localStorage.setItem('currentUser', name);
}

document.getElementById('login-button').addEventListener('click', function(event) {
    const username = document.getElementById('username').value;
    event.preventDefault();
    saveName(username);
    window.location.href = 'index.php';
});
</script>

<?php include_once 'includes/footer.php'; ?> 