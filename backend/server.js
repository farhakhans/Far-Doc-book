const express = require('express');
const cors = require('cors');
const mongoose = require('mongoose');
require('dotenv').config();

const app = express();
const PORT = process.env.PORT || 5000;

// Middleware
app.use(cors());
app.use(express.json());

// Database connection
mongoose.connect(process.env.DATABASE_URL || 'mongodb://localhost:27017/robotics-edu', {
  useNewUrlParser: true,
  useUnifiedTopology: true,
})
.then(() => console.log('MongoDB connected'))
.catch(err => console.log('MongoDB connection error:', err));

// Routes
app.get('/', (req, res) => {
  res.json({ message: 'Physical AI Humanoid Robotics Backend API' });
});

// Example route for educational content
app.get('/api/content', (req, res) => {
  const modules = require('./data/modules.json');
  res.json({
    modules: modules
  });
});

// Get specific module by ID
app.get('/api/content/:id', (req, res) => {
  const modules = require('./data/modules.json');
  const moduleId = parseInt(req.params.id);
  const module = modules.find(m => m.id === moduleId);

  if (module) {
    res.json(module);
  } else {
    res.status(404).json({ error: 'Module not found' });
  }
});

// Example route for assessments
app.get('/api/assessments', (req, res) => {
  res.json({
    assessments: [
      { id: 1, title: 'ROS2 Basics Quiz', module: 'ROS2 Fundamentals' },
      { id: 2, title: 'Simulation Project', module: 'Simulation' },
      { id: 3, title: 'Isaac Navigation Challenge', module: 'NVIDIA Isaac' }
    ]
  });
});

app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});