# Qdrant & Cohere Integration Guide

This document explains how to properly integrate Qdrant vector database and Cohere API for semantic search in your RAG chatbot.

## Prerequisites

1. **Qdrant Vector Database**
   - Install Qdrant locally: `docker run -p 6333:6333 qdrant/qdrant`
   - Or use Qdrant Cloud: https://qdrant.tech/
   - Get your API key and endpoint

2. **Cohere API Key**
   - Sign up at: https://cohere.com/
   - Get your API key from the dashboard

## Working Implementation

Here's the correct implementation that should be used in your server:

```javascript
import express from 'express';
import cors from 'cors';
import fs from 'fs/promises';
import path from 'path';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import { QdrantClient } from '@qdrant/js-client-rest'; // Use this package
import { CohereClient } from 'cohere-ai';
import dotenv from 'dotenv';

dotenv.config();

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const app = express();
const PORT = process.env.PORT || 4000;

app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Initialize Qdrant and Cohere clients
const qdrantClient = new QdrantClient({
  url: process.env.QDRANT_URL || 'http://localhost:6333',
  apiKey: process.env.QDRANT_API_KEY
});

const cohereClient = new CohereClient({
  token: process.env.COHERE_API_KEY,
});

const COLLECTION_NAME = process.env.QDRANT_COLLECTION_NAME || 'docusaurus_docs';

// Function to create embeddings using Cohere
async function createEmbedding(text) {
  try {
    const response = await cohereClient.embed({
      texts: [text],
      model: 'embed-english-v3.0',
      inputType: 'search_document',
    });
    return response.embeddings[0];
  } catch (error) {
    console.error('Error creating embedding:', error);
    throw error;
  }
}

// Function to embed query using Cohere
async function embedQuery(query) {
  try {
    const response = await cohereClient.embed({
      texts: [query],
      model: 'embed-english-v3.0',
      inputType: 'search_query',
    });
    return response.embeddings[0];
  } catch (error) {
    console.error('Error embedding query:', error);
    throw error;
  }
}

// Initialize Qdrant collection
async function initializeCollection() {
  try {
    // Check if collection exists
    await qdrantClient.getCollection(COLLECTION_NAME);
    console.log(`Collection ${COLLECTION_NAME} already exists`);
  } catch (error) {
    // Collection doesn't exist, create it
    console.log(`Creating collection ${COLLECTION_NAME}`);
    await qdrantClient.createCollection({
      collection_name: COLLECTION_NAME,
      vectors: {
        size: 1024, // Cohere's embedding dimension
        distance: 'Cosine',
      },
    });
    console.log(`Collection ${COLLECTION_NAME} created`);
  }
}

// Search function to find relevant documents using Qdrant
async function searchDocuments(query, topK = 5) {
  try {
    // Initialize collection if needed
    await initializeCollection();

    // Create embedding for the query
    const queryEmbedding = await embedQuery(query);

    // Search in Qdrant
    const searchResponse = await qdrantClient.search(COLLECTION_NAME, {
      vector: queryEmbedding,
      limit: topK,
    });

    // Format results (you'll need to store document metadata separately)
    const results = searchResponse.map(point => ({
      filepath: point.payload.filepath,
      title: point.payload.title,
      content: point.payload.contentPreview,
      similarity: point.score
    }));

    return results;
  } catch (error) {
    console.error('Search error:', error);
    throw error;
  }
}

// Initialize and start server
await initializeCollection();
console.log('Qdrant collection initialized');

app.listen(PORT, () => {
  console.log(`RAG Chatbot Server with Qdrant & Cohere running on port ${PORT}`);
});
```

## Required Package

Instead of `qdrant-client`, use:

```bash
npm install @qdrant/js-client-rest
```

And update your import:

```javascript
import { QdrantClient } from '@qdrant/js-client-rest';
```

## Environment Variables

Create/update your `.env` file:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=docusaurus_docs
COHERE_API_KEY=your_cohere_api_key
```

## Docker Setup for Qdrant

```bash
docker run -p 6333:6333 \
  -v $(pwd)/qdrant_storage:/qdrant/storage \
  qdrant/qdrant
```

## Testing the Integration

After setting up Qdrant and Cohere:

1. Run the indexing endpoint to store your documents:
   ```bash
   curl -X POST http://localhost:4000/api/rebuild-index
   ```

2. Test the search functionality:
   ```bash
   curl "http://localhost:4000/api/search?q=your_query&limit=5"
   ```

3. Test the chat functionality:
   ```bash
   curl -X POST http://localhost:4000/api/chat \
     -H "Content-Type: application/json" \
     -d '{"message": "your question", "contextSize": 3}'
   ```

## Troubleshooting

1. **Qdrant Connection Issues**: Ensure Qdrant is running and accessible at the specified URL
2. **Cohere API Issues**: Verify your API key is correct and you have sufficient credits
3. **Embedding Dimension Mismatch**: Ensure the vector size matches between Cohere (1024 for embed-english-v3.0) and Qdrant
4. **Rate Limits**: Be aware of Cohere API rate limits when indexing many documents