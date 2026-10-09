const Koa = require("koa");
const app = new Koa();
const server = require("http").createServer(app.callback());
const WebSocket = require("ws");
const wss = new WebSocket.Server({ server });
const Router = require("@koa/router");
const cors = require("@koa/cors");
const bodyParser = require("koa-bodyparser");

app.use(bodyParser());
app.use(cors());
app.use(middleware);

async function middleware(ctx, next) {
    const start = new Date();
    try {
        console.log(`[REQ] ${ctx.request.method} ${ctx.request.url}`);
        if (ctx.request.body && Object.keys(ctx.request.body).length > 0) {
            console.log(`[REQ] Payload:`, ctx.request.body);
        }
        await next();
    } catch (err) {
        ctx.status = err.status || 500;
        ctx.body = { error: err.message };
        console.error(`[ERROR] ${ctx.status} ${ctx.request.method} ${ctx.request.url} - Cause: ${err.message}`, err);
    } finally {
        const ms = new Date() - start;
        console.log(`[RES] ${start.toLocaleTimeString()} ${ctx.status} ${ctx.request.method} ${ctx.request.url} - ${ms}ms`);
    }
}

const payments = [
    { id: 1, date: "2026-06-01", amount: 5000.00, type: "salary", category: "tech", description: "Monthly Salary - Dev" },
];

const router = new Router();

router.get("/payments", async (ctx) => {
    ctx.body = payments;
    ctx.status = 200;
});

router.get("/payment/:id", async (ctx) => {
    const id = parseInt(ctx.params.id);
    const item = payments.find((t) => t.id === id);
    if (item) {
        ctx.body = item;
        ctx.status = 200;
    } else {
        ctx.throw(404, "Payment not found");
    }
});

router.post("/payment", async (ctx) => {
    const { date, amount, type, category, description } = ctx.request.body;
    if (!date || !amount || !type) {
        ctx.throw(400, "Missing required fields");
    }
    const newPayment = {
        id: Math.max(...payments.map((t) => t.id), 0) + 1,
        date,
        amount: parseFloat(amount),
        type,
        category: category || "general",
        description: description || ""
    };
    payments.push(newPayment);
    broadcast(newPayment);
    ctx.body = newPayment;
    ctx.status = 201;
});

router.del("/payment/:id", async (ctx) => {
    const id = parseInt(ctx.params.id);
    const idx = payments.findIndex((t) => t.id === id);
    if (idx === -1) {
        ctx.throw(404, "Payment not found");
    }
    const deleted = payments.splice(idx, 1)[0];
    ctx.body = deleted;
    ctx.status = 200;
});

router.get("/allPayments", async (ctx) => {
    ctx.body = payments;
    ctx.status = 200;
});

function broadcast(data) {
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(data));
        }
    });
}

app.use(router.routes());
app.use(router.allowedMethods());

const port = 2627;
server.listen(port, () => {
    console.log(`Payment Server running on port ${port}... 🚀`);
});
