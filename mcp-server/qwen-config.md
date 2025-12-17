# Qwen Tool Configuration

This configuration allows Qwen to connect to your Docusaurus documentation server using API functions.

## Qwen Function Schema

Qwen can use the following functions to interact with your documentation:

### 1. search_docs
```json
{
  "name": "search_docs",
  "description": "Search the documentation content for relevant information",
  "parameters": {
    "type": "object",
    "properties": {
      "query": {
        "type": "string",
        "description": "Search query for documentation"
      },
      "limit": {
        "type": "number",
        "description": "Maximum number of results to return",
        "default": 10
      }
    },
    "required": ["query"]
  }
}
```

### 2. read_doc_file
```json
{
  "name": "read_doc_file",
  "description": "Read a specific documentation file",
  "parameters": {
    "type": "object",
    "properties": {
      "filepath": {
        "type": "string",
        "description": "Path to the documentation file to read"
      }
    },
    "required": ["filepath"]
  }
}
```

### 3. list_docs
```json
{
  "name": "list_docs",
  "description": "List all documentation files in a directory",
  "parameters": {
    "type": "object",
    "properties": {
      "directory": {
        "type": "string",
        "description": "Directory to list files from",
        "default": "docs"
      }
    }
  }
}
```

## Implementation Notes

For Qwen integration, you would typically implement these as API endpoints that Qwen can call. The existing MCP server already provides these functions via HTTP endpoints:

- Base URL: `http://localhost:8080`
- Function execution: `POST /execute` with the function name and parameters

## Example Usage in Qwen

When Qwen needs to search documentation:
```json
{
  "name": "search_docs",
  "arguments": {
    "query": "ROS 2 fundamentals",
    "limit": 5
  }
}
```