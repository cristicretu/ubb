import { createApp } from "./app.js";
import { repository } from "./repository/InMemoryRepository.js";
import { seed } from "./repository/seed.js";

const PORT = Number(process.env.PORT ?? 2070);

seed(repository);
const app = createApp(repository);
app.listen(PORT, () => {
  console.log(`[wc2026-backend] listening on http://localhost:${PORT}`);
});
