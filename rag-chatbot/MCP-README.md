# RAG Chatbot MCP Adapter

This MCP (Model Context Protocol) adapter allows Claude and other AI systems to connect to your RAG chatbot and access documentation through semantic search capabilities.

## Features

- **Documentation Q&A**: Ask questions about Physical AI & Humanoid Robotics documentation
- **Semantic Search**: Search documentation using vector similarity
- **Context Sources**: See which documents were referenced in responses
- **Document Listing**: List all available documentation files

## Available Tools

### 1. ask_documentation
Ask questions about the documentation using RAG search.

**Parameters:**
- `query` (required): Question or query about the documentation
- `context_size` (optional): Number of context sources to include (default: 3)

**Example:**
```json
{
  "tool_name": "ask_documentation",
  "arguments": {
    "query": "What is ROS 2 and how does it differ from ROS 1?",
    "context_size": 3
  }
}
```

### 2. search_documentation
Search the documentation for specific terms or topics.

**Parameters:**
- `query` (required): Search query for documentation
- `limit` (optional): Maximum number of results to return (default: 5)

**Example:**
```json
{
  "tool_name": "search_documentation",
  "arguments": {
    "query": "simulation environments",
    "limit": 5
  }
}
```

### 3. list_documents
List all available documentation files.

**Parameters:** None

**Example:**
```json
{
  "tool_name": "list_documents",
  "arguments": {}
}
```

## Setup

1. Make sure your RAG server is running:
   ```bash
   npm start
   ```

2. Start the MCP adapter:
   ```bash
   npm run mcp
   ```
   Or directly:
   ```bash
   node mcp-adapter.js
   ```

3. The adapter will be available at `http://localhost:5001` by default

## Configuration

You can configure the adapter using environment variables:

- `RAG_MCP_PORT`: Port for the MCP adapter (default: 5001)
- `RAG_SERVER_URL`: URL of the RAG server (default: http://localhost:5000)

Example `.env` file:
```
RAG_MCP_PORT=5001
RAG_SERVER_URL=http://localhost:5000
```

## API Endpoints

- `GET /tools`: Get the list of available tools
- `POST /execute`: Execute a specific tool
- `GET /health`: Health check endpoint

## Integration with Claude

To use this MCP adapter with Claude:

1. Configure Claude to connect to `http://localhost:5001`
2. Claude will discover the available tools via the `/tools` endpoint
3. Claude can execute tools via the `/execute` endpoint

## Troubleshooting

- Make sure the RAG server is running before starting the MCP adapter
- Check that the ports are not blocked by firewall
- Verify that the RAG server URL is accessible from the MCP adapter
- Check logs for connection errors between the MCP adapter and RAG server