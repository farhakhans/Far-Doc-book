# Physical AI & Humanoid Robotics Backend

This is the backend server for the Physical AI & Humanoid Robotics Book project. It provides API endpoints to support the Docusaurus frontend.

## Features

- Basic API endpoints for book metadata
- Learning outcomes API
- Health check endpoint
- Static file serving for Docusaurus build output

## Setup

1. Install dependencies:
```bash
cd backend
npm install
```

2. Start the server:
```bash
npm start
```

Or for development with auto-restart:
```bash
npm run dev
```

## API Endpoints

- `GET /api/health` - Server health check
- `GET /api/book/metadata` - Book metadata
- `GET /api/learning-outcomes` - Learning outcomes

## Integration with Docusaurus

The backend is designed to serve the Docusaurus build output from the `public/` directory. When you build your Docusaurus site with `npm run build`, the output should be placed in the `backend/public/` directory to be served by this backend.