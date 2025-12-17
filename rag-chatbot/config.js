// Configuration for Qdrant and Cohere API
require('dotenv').config();

module.exports = {
  qdrant: {
    url: process.env.QDRANT_URL || 'http://localhost:6333', // Default Qdrant URL
    apiKey: process.env.QDRANT_API_KEY,
    collectionName: process.env.QDRANT_COLLECTION_NAME || 'docusaurus_docs'
  },
  cohere: {
    apiKey: process.env.COHERE_API_KEY
  }
};