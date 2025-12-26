// Standalone API Server for Server-Side Computation
const express = require('express');
const cors = require('cors');
const { execFile } = require('child_process');
const path = require('path');
const app = express();

// Enable CORS for all routes
app.use(cors());

// Parse JSON bodies
app.use(express.json());

// Cache for frequently asked questions to improve response time
const qaCache = new Map();

// Handle different API routes
app.post('/api/rag/query', async (req, res) => {
  const { query } = req.body;

  if (!query) {
    return res.status(400).json({
      success: false,
      error: 'Query is required'
    });
  }

  // Check cache first to improve response time
  // For varied responses, we'll still check cache but allow for some variation
  const cacheKey = query.toLowerCase().trim();
  if (qaCache.has(cacheKey)) {
    const cachedResult = qaCache.get(cacheKey);
    // Return cached result for better performance, but note that this may reduce variation
    // due to the random nature of responses in the Python script
    res.json(cachedResult);
    return;
  }

  try {
    // Path to the Python script that handles RAG retrieval
    const pythonScriptPath = path.join(__dirname, 'RAG-DOCS', 'retrieving.py');
    const fallbackScriptPath = path.join(__dirname, 'RAG-DOCS', 'retrieving_fallback.py');

    // Use execFile instead of spawn for simpler handling
    const timeout = 25000; // Increased timeout to 25 seconds to allow for API calls

    execFile('python', [pythonScriptPath, query], { timeout }, (error, stdout, stderr) => {
      if (error) {
        if (error.signal === 'SIGTERM') {
          // Process timed out - try fallback
          console.warn('Primary RAG system timeout, trying fallback:', error);

          // Try the fallback system
          execFile('python', [fallbackScriptPath, query], { timeout: 10000 }, (fallbackError, fallbackStdout, fallbackStderr) => {
            if (fallbackError) {
              console.error('Fallback system also failed:', fallbackError);
              return res.status(500).json({
                success: false,
                error: 'Both primary and fallback systems failed. Please try again later.'
              });
            }

            if (fallbackStderr) {
              console.error('Fallback stderr:', fallbackStderr);
            }

            if (!fallbackStdout) {
              console.error('Fallback script returned empty output');
              return res.status(500).json({
                success: false,
                error: 'Fallback system returned empty output'
              });
            }

            try {
              const result = JSON.parse(fallbackStdout.trim());
              const response = {
                success: true,
                data: result,
                timestamp: new Date().toISOString()
              };

              // Cache the result for future queries (only cache successful responses)
              if (result.response && !result.response.includes("No relevant information")) {
                qaCache.set(cacheKey, response);
                // Limit cache size to prevent memory issues
                if (qaCache.size > 50) {
                  const firstKey = qaCache.keys().next().value;
                  qaCache.delete(firstKey);
                }
              }

              res.json(response);
            } catch (parseErr) {
              console.error('Error parsing fallback output:', parseErr);
              console.error('Fallback raw output:', fallbackStdout);
              res.status(500).json({
                success: false,
                error: 'Error parsing response from fallback system: ' + parseErr.message
              });
            }
          });
        } else {
          console.error('Primary Python script execution error:', error);
          console.error('Error stderr:', stderr);

          // Try the fallback system
          execFile('python', [fallbackScriptPath, query], { timeout: 10000 }, (fallbackError, fallbackStdout, fallbackStderr) => {
            if (fallbackError) {
              console.error('Fallback system also failed:', fallbackError);
              return res.status(500).json({
                success: false,
                error: `Primary system failed: ${error.message}. Fallback system also failed.`
              });
            }

            if (fallbackStderr) {
              console.error('Fallback stderr:', fallbackStderr);
            }

            if (!fallbackStdout) {
              console.error('Fallback script returned empty output');
              return res.status(500).json({
                success: false,
                error: 'Fallback system returned empty output'
              });
            }

            try {
              const result = JSON.parse(fallbackStdout.trim());
              const response = {
                success: true,
                data: result,
                timestamp: new Date().toISOString()
              };

              // Cache the result for future queries (only cache successful responses)
              if (result.response && !result.response.includes("No relevant information")) {
                qaCache.set(cacheKey, response);
                // Limit cache size to prevent memory issues
                if (qaCache.size > 50) {
                  const firstKey = qaCache.keys().next().value;
                  qaCache.delete(firstKey);
                }
              }

              res.json(response);
            } catch (parseErr) {
              console.error('Error parsing fallback output:', parseErr);
              console.error('Fallback raw output:', fallbackStdout);
              res.status(500).json({
                success: false,
                error: 'Error parsing response from fallback system: ' + parseErr.message
              });
            }
          });
        }
      } else {
        // Primary system succeeded
        if (stderr) {
          console.error('Python script stderr:', stderr);
        }

        if (!stdout) {
          console.error('Python script returned empty output');
          return res.status(500).json({
            success: false,
            error: 'Python script returned empty output'
          });
        }

        try {
          const result = JSON.parse(stdout.trim());
          const response = {
            success: true,
            data: result,
            timestamp: new Date().toISOString()
          };

          // Cache the result for future queries (only cache successful responses)
          if (result.response && !result.response.includes("No relevant information")) {
            qaCache.set(cacheKey, response);
            // Limit cache size to prevent memory issues
            if (qaCache.size > 50) {
              const firstKey = qaCache.keys().next().value;
              qaCache.delete(firstKey);
            }
          }

          res.json(response);
        } catch (parseErr) {
          console.error('Error parsing Python output:', parseErr);
          console.error('Raw output:', stdout);
          res.status(500).json({
            success: false,
            error: 'Error parsing response from RAG system: ' + parseErr.message
          });
        }
      }
    });
  } catch (error) {
    console.error('RAG Query Error:', error);
    res.status(500).json({
      success: false,
      error: error.message || 'Error processing RAG query'
    });
  }
});

// Server robotics calculations API
app.get('/api/robotics/calculate', (req, res) => {
  const { jointAngles, robotType } = req.query;

  // Simulate server-side computation
  let result = null;

  if (robotType === 'arm') {
    // Simple forward kinematics calculation example
    const angles = JSON.parse(jointAngles);
    const x = Math.cos(angles[0]) * Math.cos(angles[1]);
    const y = Math.sin(angles[0]) * Math.cos(angles[1]);
    const z = Math.sin(angles[1]);

    result = { x, y, z, type: 'forward_kinematics' };
  } else if (robotType === 'mobile') {
    // Simple path planning calculation
    const [startX, startY] = JSON.parse(jointAngles);
    const distance = Math.sqrt(startX * startX + startY * startY);
    result = { distance, path: `(${startX}, ${startY}) to (0, 0)`, type: 'path_calculation' };
  }

  res.json({
    success: true,
    data: result,
    timestamp: new Date().toISOString()
  });
});

app.post('/api/robotics/simulate', (req, res) => {
  const { parameters, simulationType } = req.body;

  // Simulate different types of robotics computations
  let result = {};

  switch (simulationType) {
    case 'physics':
      // Physics simulation
      const mass = parameters.mass || 1;
      const acceleration = parameters.acceleration || 1;
      const force = mass * acceleration;
      result = { force, energy: 0.5 * mass * acceleration * acceleration };
      break;

    case 'path':
      // Path planning simulation
      const start = parameters.start || [0, 0];
      const end = parameters.end || [10, 10];
      const distance = Math.sqrt(
        Math.pow(end[0] - start[0], 2) +
        Math.pow(end[1] - start[1], 2)
      );
      result = { distance, path: [start, end], time: distance / 5 }; // assuming 5 units/sec
      break;

    case 'control':
      // Control system simulation
      const setpoint = parameters.setpoint || 0;
      const current = parameters.current || 0;
      const error = setpoint - current;
      const kp = parameters.kp || 1;
      const output = error * kp;
      result = { error, output, correction: output };
      break;

    default:
      return res.status(400).json({
        success: false,
        error: 'Invalid simulation type'
      });
  }

  res.json({
    success: true,
    data: result,
    simulationType,
    timestamp: new Date().toISOString()
  });
});

app.get('/api/robotics/simulations', (req, res) => {
  // Get available simulation types
  res.json({
    success: true,
    availableSimulations: [
      {
        id: 'physics',
        name: 'Physics Simulation',
        description: 'Calculate forces, torques, and motion parameters'
      },
      {
        id: 'path',
        name: 'Path Planning',
        description: 'Calculate optimal paths and distances'
      },
      {
        id: 'control',
        name: 'Control Systems',
        description: 'Simulate PID control and feedback systems'
      }
    ],
    timestamp: new Date().toISOString()
  });
});

app.get('/api/health', (req, res) => {
  // Health check endpoint
  res.json({
    status: 'healthy',
    uptime: process.uptime ? process.uptime() : 'N/A',
    timestamp: new Date().toISOString()
  });
});

// Handle 404 for unknown API routes
app.use('/api/*', (req, res) => {
  res.status(404).json({
    success: false,
    error: 'API endpoint not found',
    path: req.path
  });
});

// Start the server
const PORT = process.env.PORT || 3001;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});