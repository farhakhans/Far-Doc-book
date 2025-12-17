# RAG Chatbot for Docusaurus Documentation (with Qdrant & Cohere)

This RAG (Retrieval-Augmented Generation) chatbot allows users to ask questions about your Docusaurus documentation and get relevant answers based on the content in your docs directory. It's designed to use Qdrant as a vector database and Cohere for semantic embeddings.

## Features

- **Semantic Document Search**: Uses Cohere embeddings and Qdrant vector database for semantic search
- **Context-Aware Responses**: Provides answers based on relevant documentation content
- **Source Attribution**: Shows which documents were referenced to generate the response
- **Web Interface**: User-friendly chat interface
- **API Endpoints**: REST API for programmatic access

## Architecture

The RAG chatbot works by:
1. Reading all markdown files from the `docs/` directory
2. Creating semantic embeddings using Cohere API
3. Storing embeddings in Qdrant vector database
4. Matching user queries to relevant documents using vector similarity
5. Providing responses based on the retrieved context

## Installation

1. Navigate to the rag-chatbot directory:
   ```
   cd rag-chatbot
   ```

2. Install dependencies:
   ```
   npm install
   ```

3. Set up your API keys in `.env` file:
   ```
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=docusaurus_docs
   COHERE_API_KEY=your_cohere_api_key
   ```

4. For proper Qdrant integration, you may need to install the correct client:
   ```
   npm install @qdrant/js-client-rest
   ```

5. Start the server:
   ```
   npm start
   ```

The server will start at `http://localhost:4000`.

## API Endpoints

- `GET /health` - Server health check
- `GET /api/search?q={query}&limit={limit}` - Search documentation
- `POST /api/chat` - Chat with RAG bot (JSON: `{ "message": "your question" }`)
- `GET /api/documents` - List all available documents
- `POST /api/rebuild-index` - Rebuild document index with Cohere embeddings

## Usage

### Initial Setup
1. First, you need to index your documents:
   ```bash
   curl -X POST http://localhost:4000/api/rebuild-index
   ```

### Web Interface
Open `index.html` in your browser to use the chat interface.

### API Usage
```bash
# Search documentation
curl "http://localhost:4000/api/search?q=ros2&limit=3"

# Chat with the bot
curl -X POST http://localhost:4000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?","contextSize": 3}'
```

## Configuration

The chatbot automatically scans the `../docs` directory relative to the server file. It processes all `.md` and `.mdx` files and extracts titles from H1 headings or frontmatter.

### Required Services

- **Qdrant**: A vector database. You can run it locally with Docker:
  ```bash
  docker run -p 6333:6333 qdrant/qdrant
  ```

- **Cohere API**: Get an API key from [Cohere](https://cohere.com/) for semantic embeddings

## Integration with Docusaurus

The RAG chatbot is designed to work alongside your Docusaurus site. You can:

1. Run both servers simultaneously:
   - Docusaurus: `http://localhost:3000/physical-ai-humanoid-robotics-book/`
   - RAG Chatbot: `http://localhost:4000`

2. Embed the chatbot in your Docusaurus site by adding the HTML interface to your site

## How It Works

1. **Document Loading**: The system reads all markdown files from the docs directory
2. **Embedding Creation**: Cohere API creates semantic embeddings for each document
3. **Storage**: Embeddings are stored in Qdrant vector database with document metadata
4. **Query Processing**: When a user asks a question, it's converted to a semantic embedding
5. **Vector Search**: Qdrant finds documents with similar embeddings to the query
6. **Response Generation**: A response is generated using the context from relevant documents

## Complete Implementation

For the complete working implementation with proper Qdrant and Cohere integration, please see [QDRANT_COHERE_INTEGRATION.md](./QDRANT_COHERE_INTEGRATION.md) which contains the full working code and setup instructions.

## MCP Adapter

The RAG chatbot includes an MCP (Model Context Protocol) adapter that allows Claude and other AI systems to access documentation through semantic search capabilities.

### Features

- **Documentation Q&A**: Ask questions about Physical AI & Humanoid Robotics documentation
- **Semantic Search**: Search documentation using vector similarity
- **Context Sources**: See which documents were referenced in responses
- **Document Listing**: List all available documentation files

### Setup

1. Make sure your RAG server is running:
   ```bash
   npm start
   ```

2. Start the MCP adapter:
   ```bash
   npm run mcp
   ```

3. The adapter will be available at `http://localhost:5001` by default

### Available Tools

- `ask_documentation`: Ask questions about the documentation using RAG search
- `search_documentation`: Search the documentation for specific terms or topics
- `list_documents`: List all available documentation files

For complete setup instructions, see [MCP-README.md](./MCP-README.md).

## Limitations

- Requires Cohere API key for embeddings
- Requires Qdrant vector database
- API calls to Cohere may have rate limits and costs
- Package compatibility may vary; see integration guide for proper setup