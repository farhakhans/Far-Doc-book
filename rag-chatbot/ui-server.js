import express from 'express';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const app = express();
const PORT = 3000;

// Serve static files from the rag-chatbot directory
app.use(express.static(join(__dirname)));

app.get('/', (req, res) => {
    res.sendFile(join(__dirname, 'index.html'));
});

app.listen(PORT, () => {
    console.log(`RAG Chatbot UI running at http://localhost:${PORT}`);
    console.log(`Make sure these services are running:`);
    console.log(`- RAG Server: http://localhost:5004 (with health check at /health)`);
    console.log(`- MCP Adapter: http://localhost:5010 (with tools at /tools)`);
    console.log(`The frontend is configured to connect to MCP adapter at http://localhost:5010`);
});