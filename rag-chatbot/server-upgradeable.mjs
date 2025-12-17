import express from 'express';
import cors from 'cors';
import fs from 'fs/promises';
import path from 'path';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import dotenv from 'dotenv';

dotenv.config();

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const app = express();
const PORT = process.env.PORT || 5003;

app.use(cors());
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Configuration for external services
const config = {
  useExternalServices: process.env.USE_EXTERNAL_SERVICES === 'true',
  qdrantUrl: process.env.QDRANT_URL || 'http://localhost:6333',
  qdrantApiKey: process.env.QDRANT_API_KEY,
  qdrantCollectionName: process.env.QDRANT_COLLECTION_NAME || 'docusaurus_docs',
  cohereApiKey: process.env.COHERE_API_KEY
};

// External service clients (will be initialized when available)
let qdrantClient = null;
let cohereClient = null;

// Initialize external services asynchronously
async function initializeExternalServices() {
  if (config.useExternalServices && config.cohereApiKey) {
    try {
      // Initialize Cohere client
      const { CohereClient } = await import('cohere-ai');
      cohereClient = new CohereClient({
        token: config.cohereApiKey,
      });
      console.log('Cohere client initialized');
    } catch (error) {
      console.error('Failed to initialize Cohere client:', error.message);
      console.log('Falling back to simple embedding approach');
    }
  }

  if (config.useExternalServices && config.qdrantUrl) {
    try {
      // Initialize Qdrant client
      const { QdrantClient } = await import('@qdrant/js-client-rest');
      qdrantClient = new QdrantClient({
        url: config.qdrantUrl,
        apiKey: config.qdrantApiKey,
      });
      console.log('Qdrant client initialized');
    } catch (error) {
      console.error('Failed to initialize Qdrant client:', error.message);
      console.log('Falling back to in-memory storage');
    }
  }
}

// Call initialization function
initializeExternalServices().catch(console.error);

// In-memory fallback storage for document embeddings
let documentEmbeddings = [];

// Function to read and parse markdown files from the docs directory
async function loadDocuments() {
  console.log('Loading documents...');
  const docsPath = join(__dirname, '../docs');
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
    const res = join(dir, dirent.name);
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

// Initialize Qdrant collection
async function initializeCollection() {
  if (!qdrantClient) {
    // If no Qdrant client, use in-memory fallback
    return;
  }

  try {
    // Check if collection exists
    await qdrantClient.getCollections();
    const collections = await qdrantClient.getCollections();
    const collectionExists = collections.collections.some(col => col.name === config.qdrantCollectionName);

    if (!collectionExists) {
      // Create collection with appropriate vector size (Cohere embeddings are 1024 dimensions)
      await qdrantClient.createCollection(config.qdrantCollectionName, {
        vectors: {
          size: 1024, // Cohere's embedding dimension
          distance: 'Cosine',
        },
      });
      console.log(`Qdrant collection ${config.qdrantCollectionName} created`);
    } else {
      console.log(`Qdrant collection ${config.qdrantCollectionName} already exists`);
    }
  } catch (error) {
    console.error('Error initializing Qdrant collection:', error.message);
    throw error;
  }
}

// Embedding functions - will use Cohere when available, fallback to simple approach
async function createEmbedding(text) {
  if (cohereClient && config.useExternalServices) {
    try {
      const response = await cohereClient.embed({
        texts: [text],
        model: 'embed-english-v3.0',
        inputType: 'search_document',
      });
      return response.embeddings[0];
    } catch (error) {
      console.error('Cohere embedding failed, falling back to simple approach:', error.message);
    }
  }

  // Fallback: simple word-based embedding
  return createSimpleEmbedding(text);
}

async function embedQuery(query) {
  if (cohereClient && config.useExternalServices) {
    try {
      const response = await cohereClient.embed({
        texts: [query],
        model: 'embed-english-v3.0',
        inputType: 'search_query',
      });
      return response.embeddings[0];
    } catch (error) {
      console.error('Cohere query embedding failed, falling back to simple approach:', error.message);
    }
  }

  // Fallback: simple word-based embedding
  return createSimpleEmbedding(query);
}

// Simple embedding function (fallback)
function createSimpleEmbedding(text) {
  // Create a simple "embedding" by converting text to lowercase and splitting into words
  const words = text.toLowerCase()
    .replace(/[^\w\s]/g, ' ')
    .split(/\s+/)
    .filter(word => word.length > 2)
    .filter((word, index, arr) => arr.indexOf(word) === index); // Remove duplicates

  return words;
}

// Build document embeddings and store in Qdrant
async function buildEmbeddings() {
  console.log('Building embeddings with Qdrant integration...');

  // Initialize Qdrant collection if using external services
  if (qdrantClient && config.useExternalServices) {
    await initializeCollection();
  }

  const documents = await loadDocuments();
  let processedCount = 0;

  for (let i = 0; i < documents.length; i++) {
    const doc = documents[i];
    const contentPreview = doc.content.substring(0, 200) + '...';

    try {
      const embedding = await createEmbedding(doc.content);

      if (qdrantClient && config.useExternalServices) {
        // Store in Qdrant
        await qdrantClient.upsert(config.qdrantCollectionName, {
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
      } else {
        // Fallback: store in memory
        documentEmbeddings.push({
          ...doc,
          embedding,
          contentPreview
        });
      }

      console.log(`Processed document ${i+1}/${documents.length}: ${doc.filepath}`);
      processedCount++;
    } catch (error) {
      console.error(`Error processing document ${doc.filepath}:`, error.message);
    }
  }

  console.log(`Built embeddings for ${processedCount} documents`);
  return processedCount;
}

// Search function to find relevant documents using Qdrant or fallback
async function searchDocuments(query, topK = 5) {
  if (qdrantClient && config.useExternalServices) {
    // Use Qdrant for search
    try {
      const queryEmbedding = await embedQuery(query);

      // Search in Qdrant
      const searchResponse = await qdrantClient.search(config.qdrantCollectionName, {
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
      console.error('Qdrant search failed, falling back to in-memory search:', error.message);
      // Fall back to in-memory search
    }
  }

  // Fallback: in-memory search
  if (documentEmbeddings.length === 0) {
    await buildEmbeddings();
  }

  const queryEmbedding = await embedQuery(query);
  const similarities = documentEmbeddings.map((doc, index) => ({
    doc: doc,
    similarity: calculateSimilarity(queryEmbedding, doc.embedding),
    index
  }));

  // Sort by similarity and return top K
  similarities.sort((a, b) => b.similarity - a.similarity);

  return similarities
    .filter(item => item.similarity > 0.01) // Filter out very low similarities
    .slice(0, topK)
    .map(item => ({
      filepath: item.doc.filepath,
      title: item.doc.title,
      content: item.doc.contentPreview,
      similarity: item.similarity
    }));
}

// Simple similarity function (fallback)
function calculateSimilarity(queryEmbedding, docEmbedding) {
  // Handle both array (simple) and vector (Cohere) embeddings
  if (Array.isArray(queryEmbedding) && Array.isArray(docEmbedding)) {
    // Simple word overlap
    const querySet = new Set(queryEmbedding);
    const docSet = new Set(docEmbedding);

    // Find intersection
    const intersection = [...querySet].filter(word => docSet.has(word));

    // Calculate similarity (Jaccard similarity)
    const union = new Set([...querySet, ...docSet]);
    return intersection.length / union.size;
  } else {
    // For vector embeddings, use cosine similarity
    return calculateCosineSimilarity(queryEmbedding, docEmbedding);
  }
}

// Cosine similarity for vector embeddings
function calculateCosineSimilarity(vecA, vecB) {
  if (vecA.length !== vecB.length) return 0;

  let dotProduct = 0;
  let normA = 0;
  let normB = 0;

  for (let i = 0; i < vecA.length; i++) {
    dotProduct += vecA[i] * vecB[i];
    normA += vecA[i] ** 2;
    normB += vecB[i] ** 2;
  }

  return dotProduct / (Math.sqrt(normA) * Math.sqrt(normB));
}

// Improved response generation that focuses on specific answers
function generateResponse(query, context, contextSources = []) {
  if (!context) {
    return `I couldn't find relevant documentation for your query: "${query}". Please try rephrasing or check the documentation directly.`;
  }

  // Parse the context to extract the most relevant information
  const contextSections = context.split('---\n');

  // Extract the most relevant section based on the query
  let mostRelevantSection = '';
  let highestScore = 0;

  for (const section of contextSections) {
    if (!section.trim()) continue;

    // Calculate relevance score based on keyword matching with the query
    const sectionLower = section.toLowerCase();
    const queryWords = query.toLowerCase().split(/\s+/).filter(word => word.length > 2);

    let score = 0;
    for (const word of queryWords) {
      if (sectionLower.includes(word)) {
        score++;
      }
    }

    if (score > highestScore) {
      highestScore = score;
      mostRelevantSection = section;
    }
  }

  // If we found a particularly relevant section, focus on that
  if (mostRelevantSection && highestScore > 0) {
    // Extract the content part from the section
    const contentMatch = mostRelevantSection.match(/Content: ([\s\S]*?)(?:\n|$)/);
    const extractedContent = contentMatch ? contentMatch[1].trim() : mostRelevantSection.trim();

    let response = `Based on the documentation:\n\n${extractedContent}\n\n`;

    // Add specific source information
    if (contextSources.length > 0) {
      const topSource = contextSources[0]; // Most relevant source
      response += `Source: ${topSource.title} (${topSource.filepath})\n`;
    }

    response += `\nFor more detailed information about "${query}", you can check the documentation.`;
    return response;
  }

  // Fallback to the original approach if no particularly relevant section found
  return `Based on the documentation, here's what I found regarding "${query}":\n\n${context}\n\nFor more specific information, please check the referenced documents above.`;
}

// Initialize embeddings on startup
buildEmbeddings().catch(console.error);

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
    const contextSources = searchResults.map(r => ({
      filepath: r.filepath,
      title: r.title,
      similarity: r.similarity
    }));

    const response = {
      query: message,
      contextSources: contextSources,
      response: generateResponse(message, context, contextSources),
      timestamp: new Date().toISOString()
    };

    res.json(response);
  } catch (error) {
    console.error('Chat error:', error);
    res.status(500).json({ error: error.message });
  }
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'healthy',
    timestamp: new Date().toISOString(),
    documentCount: documentEmbeddings.length,
    usingExternalServices: config.useExternalServices
  });
});

// Endpoint to get all available documents
app.get('/api/documents', async (req, res) => {
  try {
    if (documentEmbeddings.length === 0) {
      await buildEmbeddings();
    }

    const documents = documentEmbeddings.map(doc => ({
      filepath: doc.filepath,
      title: doc.title
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
  console.log(`External services: ${config.useExternalServices ? 'ENABLED' : 'DISABLED (using fallback)'}`);
});

export default app;