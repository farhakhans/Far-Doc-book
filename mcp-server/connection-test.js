const axios = require('axios');

async function testConnection() {
  const baseURL = 'http://localhost:8080';

  console.log('Testing connection to MCP Server...\n');

  try {
    // Test 1: Health check
    console.log('1. Testing health endpoint...');
    const healthResponse = await axios.get(`${baseURL}/health`);
    console.log('✓ Health check successful:', healthResponse.data.status);

    // Test 2: Get tools
    console.log('\n2. Testing tools endpoint...');
    const toolsResponse = await axios.get(`${baseURL}/tools`);
    console.log('✓ Tools endpoint successful');
    console.log('Available tools:', toolsResponse.data.tools.map(t => t.name).join(', '));

    // Test 3: Test search_docs
    console.log('\n3. Testing search_docs tool...');
    const searchResponse = await axios.post(`${baseURL}/execute`, {
      tool_name: 'search_docs',
      arguments: {
        query: 'introduction',
        limit: 2
      }
    });
    console.log('✓ search_docs executed successfully');
    console.log('Results count:', searchResponse.data.count);

    // Test 4: Test list_docs
    console.log('\n4. Testing list_docs tool...');
    const listResponse = await axios.post(`${baseURL}/execute`, {
      tool_name: 'list_docs',
      arguments: {
        directory: 'docs'
      }
    });
    console.log('✓ list_docs executed successfully');
    console.log('Total docs found:', listResponse.data.count);

    console.log('\n✓ All tests passed! MCP Server is working correctly.');
    console.log('\nTo connect Claude or Qwen:');
    console.log('- Claude: Configure MCP to connect to http://localhost:8080');
    console.log('- Qwen: Use the function schemas provided in qwen-config.md');
  } catch (error) {
    console.error('✗ Test failed:', error.message);
    if (error.response) {
      console.error('Response data:', error.response.data);
      console.error('Status:', error.response.status);
    }
  }
}

if (require.main === module) {
  testConnection();
}

module.exports = testConnection;