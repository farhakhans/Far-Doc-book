const axios = require('axios');

async function testMCP() {
  const baseURL = 'http://localhost:8080';

  try {
    // Test 1: Get tools list
    console.log('Testing tools endpoint...');
    const toolsResponse = await axios.get(`${baseURL}/tools`);
    console.log('Available tools:', toolsResponse.data.tools.map(t => t.name));

    // Test 2: Health check
    console.log('\nTesting health check...');
    const healthResponse = await axios.get(`${baseURL}/health`);
    console.log('Health status:', healthResponse.data.status);

    // Test 3: Try listing docs
    console.log('\nTesting list_docs tool...');
    const listResponse = await axios.post(`${baseURL}/execute`, {
      tool_name: 'list_docs',
      arguments: { directory: 'docs' }
    });
    console.log('Docs count:', listResponse.data.count);
    if (listResponse.data.count > 0) {
      console.log('Sample files:', listResponse.data.files.slice(0, 3));
    }

    console.log('\nAll tests completed successfully!');
  } catch (error) {
    console.error('Test failed:', error.message);
    if (error.response) {
      console.error('Response data:', error.response.data);
      console.error('Response status:', error.response.status);
    }
  }
}

if (require.main === module) {
  testMCP();
}

module.exports = testMCP;