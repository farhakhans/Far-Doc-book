# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

Before starting with the Physical AI & Humanoid Robotics book, ensure you have:

- Node.js version 18 or higher
- npm or yarn package manager
- Git for version control
- A GitHub account for deployment
- Basic familiarity with command line tools

## Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/[your-org]/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Start Local Development Server
```bash
npm run start
# or
yarn start
```

This will start a local development server at `http://localhost:3000` with live reloading enabled.

## Basic Docusaurus Commands

### Development
```bash
# Start development server
npm run start

# Build static files for production
npm run build

# Serve built files locally
npm run serve

# Deploy to GitHub Pages
npm run deploy
```

## Project Structure Overview

```
docs/
├── intro/                 # Introduction and overview content
├── modules/              # Main learning modules
│   ├── module-1-ros2/    # ROS 2 fundamentals
│   ├── module-2-simulation/ # Gazebo & Unity simulation
│   ├── module-3-nvidia-isaac/ # NVIDIA Isaac platform
│   └── module-4-vla/     # Vision-Language-Action systems
├── weekly-breakdown/     # Week-by-week curriculum
├── learning-outcomes/    # Learning objectives
├── assessments/          # Project assignments
└── hardware-requirements/ # Hardware specs and options
```

## Adding New Content

### Create a New Document
```bash
# Create a new MDX file in the appropriate section
# Example: docs/modules/module-1-ros2/new-topic.mdx
```

### Create a New Blog Post (if needed)
```bash
npm run write-blog
```

## Configuration Files

### docusaurus.config.js
Contains site metadata, navigation, plugins, and theme configuration.

### sidebars.js
Defines the sidebar navigation structure for the documentation.

## Deployment to GitHub Pages

### 1. Configure GitHub Pages
- Go to your repository Settings
- Navigate to Pages section
- Set source to "Deploy from a branch"
- Select "gh-pages" branch

### 2. Deploy Command
```bash
GIT_USER=<Your GitHub username> npm run deploy
```

Or set up GitHub Actions for automatic deployment on pushes to main branch.

## Content Formatting Guidelines

### Markdown Best Practices
- Use proper heading hierarchy (H1 for page title, H2-H6 for sections)
- Include alt text for all images
- Use code blocks with language specification
- Link to internal pages using relative paths

### Example MDX Content
```mdx
---
title: ROS 2 Nodes and Topics
description: Learn about ROS 2 communication patterns
---

import Component from '@site/src/components/Component';

# ROS 2 Nodes and Topics

## Introduction

In ROS 2, nodes communicate through topics using a publish-subscribe pattern.

## Code Example

Here's how to create a simple publisher:

```python
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(String, 'topic_name', 10)
    # Additional code here
```

## Hands-on Exercise

Create your own publisher and subscriber nodes following the example above.
```

## Local Development Tips

- Use `npm run start` for development with hot reloading
- Edit files in `docs/` and see changes immediately
- Use `Ctrl+C` to stop the development server
- Run `npm run build` to test the production build locally

## Troubleshooting

### Common Issues
1. **Port already in use**: Change port with `npm run start -- --port 3001`
2. **Dependency issues**: Run `npm install` to reinstall dependencies
3. **Build failures**: Check for syntax errors in MDX files

### Getting Help
- Check the Docusaurus documentation
- Review the project's issue tracker
- Consult the community forums