import express from 'express';
import cors from 'cors';
import axios from 'axios';

const app = express();
const PORT = process.env.RAG_MCP_PORT || 5001;

app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Configuration
const RAG_SERVER_URL = process.env.RAG_SERVER_URL || 'http://localhost:5004';

// MCP Tools Configuration
const mcpTools = {
  version: '1.0',
  name: 'RAG Documentation Chatbot',
  description: 'MCP adapter for RAG-powered documentation assistant',
  tools: [
    {
      name: 'ask_documentation',
      description: 'Ask questions about the Physical AI & Humanoid Robotics documentation using RAG search',
      input_schema: {
        type: 'object',
        properties: {
          query: {
            type: 'string',
            description: 'Question or query about the documentation'
          },
          context_size: {
            type: 'number',
            description: 'Number of context sources to include (default: 3)',
            default: 3
          }
        },
        required: ['query']
      }
    },
    {
      name: 'search_documentation',
      description: 'Search the documentation for specific terms or topics',
      input_schema: {
        type: 'object',
        properties: {
          query: {
            type: 'string',
            description: 'Search query for documentation'
          },
          limit: {
            type: 'number',
            description: 'Maximum number of results to return (default: 5)',
            default: 5
          }
        },
        required: ['query']
      }
    },
    {
      name: 'list_documents',
      description: 'List all available documentation files',
      input_schema: {
        type: 'object',
        properties: {}
      }
    }
  ]
};

// Tool implementations
const tools = {
  async ask_documentation({ query, context_size = 3 }) {
    try {
      const response = await axios.post(`${RAG_SERVER_URL}/api/chat`, {
        message: query,
        contextSize: context_size
      });

      return {
        success: true,
        query: query,
        response: response.data.response,
        context_sources: response.data.contextSources,
        timestamp: response.data.timestamp
      };
    } catch (error) {
      return {
        success: false,
        error: error.message,
        query: query
      };
    }
  },

  async search_documentation({ query, limit = 5 }) {
    try {
      const response = await axios.get(`${RAG_SERVER_URL}/api/search?q=${encodeURIComponent(query)}&limit=${limit}`);

      return {
        success: true,
        query: query,
        results: response.data.results,
        count: response.data.count
      };
    } catch (error) {
      return {
        success: false,
        error: error.message,
        query: query
      };
    }
  },

  async list_documents() {
    try {
      const response = await axios.get(`${RAG_SERVER_URL}/api/documents`);

      return {
        success: true,
        documents: response.data.documents,
        count: response.data.count
      };
    } catch (error) {
      return {
        success: false,
        error: error.message
      };
    }
  }
};

// MCP Discovery endpoint
app.get('/tools', (req, res) => {
  res.json(mcpTools);
});

// Tool execution endpoint
app.post('/execute', async (req, res) => {
  const { tool_name, arguments: args } = req.body;

  if (!tool_name) {
    return res.status(400).json({ error: 'Missing tool_name' });
  }

  if (!tools[tool_name]) {
    return res.status(404).json({ error: `Tool ${tool_name} not found` });
  }

  try {
    const result = await tools[tool_name](args);
    res.json(result);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'healthy',
    timestamp: new Date().toISOString(),
    rag_server_url: RAG_SERVER_URL
  });
});

app.listen(PORT, () => {
  console.log(`RAG Chatbot MCP Adapter running on port ${PORT}`);
  console.log(`Tools endpoint: http://localhost:${PORT}/tools`);
  console.log(`Execute endpoint: http://localhost:${PORT}/execute`);
  console.log(`Connected to RAG server: ${RAG_SERVER_URL}`);
});

