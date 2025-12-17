// Basic Express.js server for the Physical AI & Humanoid Robotics Book project
const express = require('express');
const cors = require('cors');
const path = require('path');
const app = express();
const PORT = process.env.PORT || 5000;

// Middleware
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Serve static files from the 'public' directory (where Docusaurus builds to)
app.use(express.static(path.join(__dirname, 'public')));

// API Routes
app.get('/api/health', (req, res) => {
  res.json({ status: 'OK', timestamp: new Date().toISOString() });
});

const fs = require('fs');
const path = require('path');

// Get book content metadata
app.get('/api/book/metadata', (req, res) => {
  const modulesPath = path.join(__dirname, 'data', 'modules.json');
  const modulesData = JSON.parse(fs.readFileSync(modulesPath, 'utf8'));

  res.json({
    title: 'Physical AI & Humanoid Robotics',
    description: 'Bridging Digital AI and Physical Robots',
    version: '1.0.0',
    moduleCount: modulesData.modules.length,
    modules: modulesData.modules.map(m => m.title)
  });
});

// Get learning outcomes
app.get('/api/learning-outcomes', (req, res) => {
  res.json({
    outcomes: [
      'Understand core concepts of Physical AI and embodied intelligence',
      'Master ROS 2 concepts including nodes, topics, services, rclpy, and URDF',
      'Implement simulation environments with Gazebo and Unity',
      'Work with NVIDIA Isaac tools including Isaac Sim, Isaac ROS, and Nav2',
      'Create Vision-Language-Action systems using Whisper and LLMs'
    ]
  });
});

// Get all modules
app.get('/api/modules', (req, res) => {
  const modulesPath = path.join(__dirname, 'data', 'modules.json');
  const modulesData = JSON.parse(fs.readFileSync(modulesPath, 'utf8'));
  res.json(modulesData);
});

// Get a specific module by ID
app.get('/api/modules/:id', (req, res) => {
  const modulesPath = path.join(__dirname, 'data', 'modules.json');
  const modulesData = JSON.parse(fs.readFileSync(modulesPath, 'utf8'));
  const module = modulesData.modules.find(m => m.id === req.params.id);

  if (module) {
    res.json(module);
  } else {
    res.status(404).json({ error: 'Module not found' });
  }
});

// Catch-all handler for frontend routes (Docusaurus)
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
  console.log(`Health check: http://localhost:${PORT}/api/health`);
});