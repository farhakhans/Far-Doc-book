# Docusaurus Documentation API

This document describes how AI systems can connect to and interact with the Docusaurus documentation server.

## Base URL
`http://localhost:9000`

## Endpoints

### GET /tools
**Description**: Get the list of available tools and their schemas
**Response**:
```json
{
  "version": "1.0",
  "name": "Docusaurus Documentation MCP Server",
  "description": "MCP server for interacting with Docusaurus documentation site",
  "tools": [
    {
      "name": "search_docs",
      "description": "Search the documentation content",
      "input_schema": { ... }
    },
    {
      "name": "read_doc_file",
      "description": "Read a specific documentation file",
      "input_schema": { ... }
    },
    {
      "name": "list_docs",
      "description": "List all documentation files",
      "input_schema": { ... }
    }
  ]
}
```

### POST /execute
**Description**: Execute a specific tool with given arguments
**Request Body**:
```json
{
  "tool_name": "string",
  "arguments": {
    // Arguments specific to the tool
  }
}
```
**Response**:
```json
{
  "success": true,
  "result": { ... } // Tool-specific result
}
```

### GET /health
**Description**: Check server health status
**Response**:
```json
{
  "status": "healthy",
  "timestamp": "2023-12-17T10:00:00.000Z"
}
```

## Tool Details

### search_docs
- **Purpose**: Search documentation content
- **Arguments**:
  - `query` (required): Search query
  - `limit` (optional): Number of results (default: 10)
- **Response**: List of matching documents with content snippets

### read_doc_file
- **Purpose**: Read a specific documentation file
- **Arguments**:
  - `filepath` (required): Path to file to read
- **Response**: File content

### list_docs
- **Purpose**: List documentation files in a directory
- **Arguments**:
  - `directory` (optional): Directory to list (default: 'docs')
- **Response**: List of file paths

## Integration Examples

### For Claude
Claude can connect using MCP protocol by pointing to the base URL.

### For Qwen
Qwen can integrate by using the POST /execute endpoint with appropriate function schemas.

### For Other AI Systems
Any system can integrate by making HTTP requests to the endpoints with proper JSON payloads.