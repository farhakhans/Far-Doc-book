# Backend Services

This directory contains backend services for the Physical AI Humanoid Robotics educational platform.

## Services

- **API Server**: REST API for educational content and assessments
- **Database**: Storage for user progress, content metadata, and analytics
- **Authentication**: User management and access control
- **Simulation Services**: Backend for simulation environments

## Setup

```bash
cd backend
npm install
npm start
```

## Configuration

Environment variables are managed in `.env` file:

```env
PORT=3000
DATABASE_URL=mongodb://localhost:27017/robotics-edu
JWT_SECRET=your-jwt-secret
```