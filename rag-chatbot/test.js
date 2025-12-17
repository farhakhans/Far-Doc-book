const axios = require('axios');

async function testRAGChatbot() {
  const baseURL = 'http://localhost:4000';

  console.log('Testing RAG Chatbot with Qdrant & Cohere...\n');

  try {
    // Test 1: Health check
    console.log('1. Testing health endpoint...');
    const healthResponse = await axios.get(`${baseURL}/health`);
    console.log('✓ Health check successful:', healthResponse.data.status);

    // Test 2: List documents
    console.log('\n2. Testing documents endpoint...');
    const docsResponse = await axios.get(`${baseURL}/api/documents`);
    console.log('✓ Documents endpoint successful');
    console.log('  Total documents:', docsResponse.data.count);
    if (docsResponse.data.count > 0) {
      console.log('  Sample documents:', docsResponse.data.documents.slice(0, 3).map(d => d.title));
    } else {
      console.log('  No documents found. You may need to index documents first using /api/rebuild-index');
    }

    // Test 3: Search functionality
    console.log('\n3. Testing search functionality...');
    try {
      const searchResponse = await axios.get(`${baseURL}/api/search?q=ros2&limit=2`);
      console.log('✓ Search successful');
      console.log('  Query:', searchResponse.data.query);
      console.log('  Results found:', searchResponse.data.length);
    } catch (searchError) {
      console.log('⚠ Search failed (may need to index documents first):', searchError.message);
    }

    // Test 4: Chat functionality
    console.log('\n4. Testing chat functionality...');
    try {
      const chatResponse = await axios.post(`${baseURL}/api/chat`, {
        message: 'What is ROS 2?',
        contextSize: 2
      });
      console.log('✓ Chat successful');
      console.log('  Response preview:', chatResponse.data.response.substring(0, 100) + '...');
    } catch (chatError) {
      console.log('⚠ Chat failed (may need to index documents first):', chatError.message);
    }

    console.log('\n✓ Basic tests completed! RAG Chatbot is running.');
    console.log('\nTo fully use the chatbot:');
    console.log('- First index documents: POST to /api/rebuild-index');
    console.log('- Then use the web interface or API endpoints');
    console.log('\nTo index documents now:');
    console.log(`curl -X POST ${baseURL}/api/rebuild-index`);
  } catch (error) {
    console.error('✗ Test failed:', error.message);
    if (error.response) {
      console.error('Response data:', error.response.data);
      console.error('Status:', error.response.status);
    }
  }
}

if (require.main === module) {
  testRAGChatbot();
}

module.exports = testRAGChatbot;