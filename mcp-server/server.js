const express = require('express');
const cors = require('cors');
const { exec } = require('child_process');
const fs = require('fs').promises;
const path = require('path');

const app = express();
const PORT = process.env.PORT || 9000;

// Add logging middleware to track requests
app.use((req, res, next) => {
  console.log(`[${new Date().toISOString()}] ${req.method} ${req.path}`);
  if (req.body && Object.keys(req.body).length > 0) {
    console.log('Request body:', req.body);
  }
  next();
});

app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// MCP Server Configuration
const mcpConfig = {
  version: '1.0',
  name: 'Docusaurus Documentation MCP Server',
  description: 'MCP server for interacting with Docusaurus documentation site',
  tools: [
    {
      name: 'search_docs',
      description: 'Search the documentation content',
      input_schema: {
        type: 'object',
        properties: {
          query: { type: 'string', description: 'Search query for documentation' },
          limit: { type: 'number', description: 'Maximum number of results to return', default: 10 }
        },
        required: ['query']
      }
    },
    {
      name: 'read_doc_file',
      description: 'Read a specific documentation file',
      input_schema: {
        type: 'object',
        properties: {
          filepath: { type: 'string', description: 'Path to the documentation file to read' }
        },
        required: ['filepath']
      }
    },
    {
      name: 'list_docs',
      description: 'List all documentation files',
      input_schema: {
        type: 'object',
        properties: {
          directory: { type: 'string', description: 'Directory to list files from', default: 'docs' }
        }
      }
    }
  ]
};

// Tool implementations
const tools = {
  async search_docs({ query, limit = 10 }) {
    try {
      // This is a simple implementation - in a real scenario, you might want to use a search index
      const docsPath = path.join(__dirname, '../docs');
      const files = await findMarkdownFiles(docsPath);
      const results = [];

      for (const file of files) {
        const content = await fs.readFile(file, 'utf8');
        if (content.toLowerCase().includes(query.toLowerCase())) {
          results.push({
            file: path.relative(docsPath, file),
            content: content.substring(0, 500) + '...'
          });

          if (results.length >= limit) break;
        }
      }

      return {
        success: true,
        query,
        results,
        count: results.length
      };
    } catch (error) {
      return {
        success: false,
        error: error.message
      };
    }
  },

  async read_doc_file({ filepath }) {
    try {
      const fullPath = path.join(__dirname, '../', filepath);
      const content = await fs.readFile(fullPath, 'utf8');

      return {
        success: true,
        filepath,
        content
      };
    } catch (error) {
      return {
        success: false,
        error: error.message
      };
    }
  },

  async list_docs({ directory = 'docs' }) {
    try {
      const docsPath = path.join(__dirname, '../', directory);
      const files = await findMarkdownFiles(docsPath);
      const relativeFiles = files.map(f => path.relative(path.join(__dirname, '..'), f));

      return {
        success: true,
        directory,
        files: relativeFiles,
        count: relativeFiles.length
      };
    } catch (error) {
      return {
        success: false,
        error: error.message
      };
    }
  }
};

// Helper function to find markdown files
async function findMarkdownFiles(dir) {
  const dirents = await fs.readdir(dir, { withFileTypes: true });
  let files = [];

  for (const dirent of dirents) {
    const res = path.resolve(dir, dirent.name);
    if (dirent.isDirectory()) {
      files = files.concat(await findMarkdownFiles(res));
    } else if (dirent.isFile() && (dirent.name.endsWith('.md') || dirent.name.endsWith('.mdx'))) {
      files.push(res);
    }
  }

  return files;
}

// MCP Discovery endpoint
app.get('/tools', (req, res) => {
  res.json(mcpConfig);
});

// Tool execution endpoint
app.post('/execute', async (req, res) => {
  const { tool_name, arguments: args } = req.body;

  console.log(`Tool execution: ${tool_name}`, { args });

  if (!tool_name) {
    console.log('Error: Missing tool_name');
    return res.status(400).json({ error: 'Missing tool_name' });
  }

  if (!tools[tool_name]) {
    console.log(`Error: Tool ${tool_name} not found`);
    return res.status(404).json({ error: `Tool ${tool_name} not found` });
  }

  try {
    const result = await tools[tool_name](args);
    console.log(`Tool ${tool_name} executed successfully`);
    res.json(result);
  } catch (error) {
    console.log(`Error executing tool ${tool_name}:`, error.message);
    res.status(500).json({ error: error.message });
  }
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'healthy', timestamp: new Date().toISOString() });
});

app.listen(PORT, () => {
  console.log(`MCP Server running on port ${PORT}`);
  console.log(`Tools endpoint: http://localhost:${PORT}/tools`);
  console.log(`Execute endpoint: http://localhost:${PORT}/execute`);
});

module.exports = app;