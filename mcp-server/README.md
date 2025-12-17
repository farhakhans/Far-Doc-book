# MCP Server for Docusaurus Documentation

This server allows AI systems like Claude and Qwen to interact with your Docusaurus documentation site.

## Server Details
- **URL**: `http://localhost:9000`
- **Tools Endpoint**: `http://localhost:9000/tools`
- **Execute Endpoint**: `http://localhost:9000/execute`
- **Health Check**: `http://localhost:9000/health`

## Available Tools

### 1. search_docs
- **Description**: Search the documentation content
- **Parameters**:
  - `query` (required): Search query for documentation
  - `limit` (optional): Maximum number of results to return (default: 10)

### 2. read_doc_file
- **Description**: Read a specific documentation file
- **Parameters**:
  - `filepath` (required): Path to the documentation file to read

### 3. list_docs
- **Description**: List all documentation files
- **Parameters**:
  - `directory` (optional): Directory to list files from (default: 'docs')

## Connection Instructions

### For Claude
See [claude-config.md](./claude-config.md) for specific configuration details.

### For Qwen
See [qwen-config.md](./qwen-config.md) for specific configuration details.

### For Other AI Systems
See [API.md](./API.md) for general API documentation.

## Installation and Setup

1. Navigate to the mcp-server directory:
   ```
   cd mcp-server
   ```

2. Install dependencies:
   ```
   npm install
   ```

3. Start the server:
   ```
   npm start
   ```

4. The server will be available at http://localhost:8080

## Testing

Run the test script to verify the server is working:
```
node test.js
```

Or run the connection test to specifically check if AI systems can connect:
```
node connection-test.js
```

## Checking Connection Status

### Server-Side Monitoring
1. When the server is running, look at the terminal logs for incoming requests
2. Each request will be logged with timestamp and details
3. Successful tool executions will be logged as "Tool X executed successfully"

### Client-Side Testing
1. Use the connection test script to verify connectivity
2. Look for success messages indicating each endpoint works
3. Check that tools return expected results

### For Claude
- Claude will indicate in its interface when MCP tools are available
- When Claude uses tools from your server, you'll see the requests in server logs

### For Qwen
- Qwen will use the function schemas to call your endpoints
- Monitor server logs to confirm Qwen is making requests to your API