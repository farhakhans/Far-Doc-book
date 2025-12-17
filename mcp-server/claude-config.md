# Claude MCP Configuration

This configuration allows Claude to connect to your Docusaurus documentation MCP server.

## Setup for Claude

1. **Server URL**: `http://localhost:9000`

2. **Tools Available**:
   - `search_docs`: Search documentation content
   - `read_doc_file`: Read specific documentation files
   - `list_docs`: List all documentation files

3. **Example Usage**:
   When Claude needs to search documentation:
   ```json
   {
     "tool_name": "search_docs",
     "arguments": {
       "query": "ROS 2 fundamentals",
       "limit": 5
     }
   }
   ```

4. **Connection Method**:
   Claude can connect to the MCP server by making HTTP requests to:
   - Tools endpoint: `GET http://localhost:8080/tools`
   - Execute endpoint: `POST http://localhost:8080/execute`

5. **Security Note**:
   For production use, ensure the server is properly secured and not exposed publicly without authentication.