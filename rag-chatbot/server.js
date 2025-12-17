const express = require('express');
const cors = require('cors');
const fs = require('fs').promises;
const path = require('path');
const { QdrantClient } = require('qdrant-client');
const { CohereClient } = require('cohere-ai');
require('dotenv').config();

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

// Function to read and parse markdown files from the docs directory
async function loadDocuments() {
  console.log('Loading documents...');
  const docsPath = path.join(__dirname, '../docs');
  const files = await findMarkdownFiles(docsPath);
  const documents = [];

  for (const file of files) {
    try {
      const content = await fs.readFile(file, 'utf8');
      // Extract frontmatter and content
      const { frontmatter, content: docContent } = parseFrontmatter(content);

      documents.push({
        filepath: path.relative(docsPath, file),
        content: docContent,
        title: extractTitle(docContent, frontmatter),
        path: file
      });
    } catch (error) {
      console.error(`Error reading file ${file}:`, error.message);
    }
  }

  console.log(`Loaded ${documents.length} documents`);
  return documents;
}

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

// Parse frontmatter from markdown files
function parseFrontmatter(content) {
  const frontmatterRegex = /^---\s*\n([\s\S]*?)\n?---\s*\n([\s\S]*)$/;
  const match = content.match(frontmatterRegex);

  if (match) {
    const frontmatter = match[1];
    const contentWithoutFrontmatter = match[2];
    return {
      frontmatter: parseYaml(frontmatter),
      content: contentWithoutFrontmatter
    };
  }

  return {
    frontmatter: {},
    content: content
  };
}

// Simple YAML parser for frontmatter (simplified)
function parseYaml(yamlString) {
  const result = {};
  const lines = yamlString.split('\n');

  for (const line of lines) {
    const colonIndex = line.indexOf(':');
    if (colonIndex > 0) {
      const key = line.substring(0, colonIndex).trim();
      const value = line.substring(colonIndex + 1).trim();
      result[key] = value.startsWith('"') && value.endsWith('"')
        ? value.substring(1, value.length - 1)
        : value;
    }
  }

  return result;
}

// Extract title from markdown content
function extractTitle(content, frontmatter) {
  if (frontmatter.title) {
    return frontmatter.title;
  }

  // Look for first H1 in content
  const h1Match = content.match(/^#\s+(.*)$/m);
  if (h1Match) {
    return h1Match[1].trim();
  }

  // If no H1, try H2
  const h2Match = content.match(/^##\s+(.*)$/m);
  if (h2Match) {
    return h2Match[1].trim();
  }

  // Return filename as fallback
  return 'Untitled';
}

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
    await qdrantClient.createCollection(COLLECTION_NAME, {
      vectors: {
        size: 1024, // Cohere's embedding dimension
        distance: 'Cosine',
      },
    });
    console.log(`Collection ${COLLECTION_NAME} created`);
  }
}

// Build document embeddings and store in Qdrant
async function buildEmbeddings() {
  console.log('Initializing Qdrant collection...');
  await initializeCollection();

  console.log('Loading documents...');
  const documents = await loadDocuments();

  console.log('Creating embeddings and storing in Qdrant...');
  for (let i = 0; i < documents.length; i++) {
    const doc = documents[i];
    const contentPreview = doc.content.substring(0, 200) + '...';

    try {
      // Create embedding for the document content
      const embedding = await createEmbedding(doc.content);

      // Store in Qdrant
      await qdrantClient.upsert(COLLECTION_NAME, {
        points: [{
          id: i,
          vector: embedding,
          payload: {
            filepath: doc.filepath,
            title: doc.title,
            content: doc.content,
            contentPreview: contentPreview
          }
        }]
      });

      console.log(`Stored document ${i+1}/${documents.length}: ${doc.filepath}`);
    } catch (error) {
      console.error(`Error processing document ${doc.filepath}:`, error.message);
    }
  }

  console.log(`Built embeddings for ${documents.length} documents and stored in Qdrant`);
  return documents.length;
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
      with_payload: true,
    });

    // Format results
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

// Initialize Qdrant collection on startup, but don't rebuild all embeddings unless needed
initializeCollection().then(() => {
  console.log('Qdrant collection initialized');
}).catch(console.error);

// API endpoint to search documentation
app.get('/api/search', async (req, res) => {
  try {
    const { q, limit = 5 } = req.query;

    if (!q) {
      return res.status(400).json({ error: 'Query parameter "q" is required' });
    }

    const results = await searchDocuments(q, parseInt(limit));

    res.json({
      query: q,
      results,
      count: results.length
    });
  } catch (error) {
    console.error('Search error:', error);
    res.status(500).json({ error: error.message });
  }
});

// API endpoint for chat with RAG
app.post('/api/chat', async (req, res) => {
  try {
    const { message, contextSize = 3 } = req.body;

    if (!message) {
      return res.status(400).json({ error: 'Message is required' });
    }

    // Search for relevant documents
    const searchResults = await searchDocuments(message, contextSize);

    // Create context from search results
    const context = searchResults.map(result =>
      `File: ${result.filepath}\nTitle: ${result.title}\nContent: ${result.content}\n---\n`
    ).join('');

    // Generate response using the context
    const response = {
      query: message,
      contextSources: searchResults.map(r => ({
        filepath: r.filepath,
        title: r.title,
        similarity: r.similarity
      })),
      response: generateResponse(message, context),
      timestamp: new Date().toISOString()
    };

    res.json(response);
  } catch (error) {
    console.error('Chat error:', error);
    res.status(500).json({ error: error.message });
  }
});

// Simple response generation (in a real implementation, you'd connect to an LLM)
function generateResponse(query, context) {
  if (!context) {
    return `I couldn't find relevant documentation for your query: "${query}". Please try rephrasing or check the documentation directly.`;
  }

  return `Based on the documentation, here's what I found regarding "${query}":\n\n${context}\n\nIf this doesn't fully answer your question, please ask for more specific details.`;
}

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'healthy',
    timestamp: new Date().toISOString(),
    documentCount: documentEmbeddings.length
  });
});

// Endpoint to get all available documents
app.get('/api/documents', async (req, res) => {
  try {
    // Initialize collection
    await initializeCollection();

    // Get all points from Qdrant to list documents
    const points = await qdrantClient.scroll(COLLECTION_NAME, {
      limit: 1000, // Adjust as needed
      with_payload: true,
    });

    const documents = points.points.map(point => ({
      filepath: point.payload.filepath,
      title: point.payload.title,
      id: point.id
    }));

    res.json({
      documents,
      count: documents.length
    });
  } catch (error) {
    console.error('Documents error:', error);
    res.status(500).json({ error: error.message });
  }
});

// Endpoint to rebuild embeddings (index documents)
app.post('/api/rebuild-index', async (req, res) => {
  try {
    console.log('Starting index rebuild...');
    const count = await buildEmbeddings();
    res.json({
      message: `Successfully indexed ${count} documents`,
      count: count
    });
  } catch (error) {
    console.error('Index rebuild error:', error);
    res.status(500).json({ error: error.message });
  }
});

app.listen(PORT, () => {
  console.log(`RAG Chatbot Server running on port ${PORT}`);
  console.log(`Health: http://localhost:${PORT}/health`);
  console.log(`Search API: http://localhost:${PORT}/api/search`);
  console.log(`Chat API: http://localhost:${PORT}/api/chat`);
});

module.exports = app;