const { spawn } = require('child_process');
const path = require('path');

/**
 * Execute the Python RAG script and return results
 * @param {string} query - The query to search for in the RAG system
 * @returns {Promise<Object>} - The RAG response
 */
function executeRagQuery(query) {
  return new Promise((resolve, reject) => {
    // Path to the Python script that handles RAG retrieval
    const pythonScriptPath = path.join(__dirname, '..', 'RAG-DOCS', 'retrieving.py');

    // Create a temporary script to pass the query to the Python script
    const tempScript = `
import sys
import json
import os
sys.path.append('${path.join(__dirname, '..', 'RAG-DOCS').replace(/\\/g, '\\\\')}')

# Import the retrieve function from retrieving.py
exec(open('${path.join(__dirname, '..', 'RAG-DOCS', 'retrieving.py').replace(/\\/g, '\\\\')}').read())

# Execute the query
query = sys.argv[1] if len(sys.argv) > 1 else "default query"
try:
    results = retrieve(query)
    response = {
        "query": query,
        "response": f"Based on the robotics book content: {' '.join(results[:2]) if results else 'No relevant information found.'}",
        "sources": [{"title": f"Source {i+1}", "url": "/docs/intro"} for i in range(min(2, len(results)))]
    }
    print(json.dumps(response))
except Exception as e:
    error_response = {
        "query": query,
        "response": f"Error processing query: {str(e)}",
        "sources": []
    }
    print(json.dumps(error_response))
`;

    // Write temporary Python script
    const fs = require('fs');
    const tempScriptPath = path.join(__dirname, 'temp_rag_query.py');
    fs.writeFileSync(tempScriptPath, tempScript);

    // Execute the temporary Python script with the query as an argument
    const pythonProcess = spawn('python', [tempScriptPath, query]);

    let output = '';
    let errorOutput = '';

    pythonProcess.stdout.on('data', (data) => {
      output += data.toString();
    });

    pythonProcess.stderr.on('data', (data) => {
      errorOutput += data.toString();
    });

    pythonProcess.on('close', (code) => {
      // Clean up temporary file
      try {
        fs.unlinkSync(tempScriptPath);
      } catch (unlinkErr) {
        console.warn('Could not delete temporary file:', unlinkErr.message);
      }

      if (code === 0) {
        try {
          const result = JSON.parse(output.trim());
          resolve(result);
        } catch (parseErr) {
          reject(new Error(`Could not parse Python output: ${parseErr.message}`));
        }
      } else {
        reject(new Error(`Python script failed with code ${code}. Error: ${errorOutput}`));
      }
    });

    pythonProcess.on('error', (err) => {
      // Clean up temporary file in case of error
      try {
        fs.unlinkSync(tempScriptPath);
      } catch (unlinkErr) {
        console.warn('Could not delete temporary file:', unlinkErr.message);
      }

      reject(err);
    });
  });
}

module.exports = { executeRagQuery };